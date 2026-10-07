/*
 * Zigbee roller shutter controller for Home Assistant (ZHA)
 *
 * Hardware: ESP32-H2 board (see hardware/) in parallel to the membrane keypad
 *           of a Heim & Haus solar roller shutter controller
 * Zigbee:   sleepy end device (battery powered),
 *           HA Window Covering Device (0x0202), endpoint 1
 *
 * ZHA recognises the device automatically as a "cover" entity (up / down /
 * stop / position). The position is calculated from time (see shutter.c).
 * In addition: battery voltage and charge level (Power Configuration) and the
 * solar panel voltage (Analog Input, in volts).
 */
#include <stdatomic.h>
#include <string.h>

#include "esp_app_desc.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#if CONFIG_PM_ENABLE
#include "esp_pm.h"
#include "esp_sleep.h"
#endif

#include "esp_zigbee.h"
#include "ezbee/zha.h"

#include "inputs.h"
#include "ota.h"
#include "shutter.h"

static const char *TAG = "MAIN";

#define SHUTTER_ENDPOINT                  1
#define ESP_ZIGBEE_STORAGE_PARTITION_NAME "nvs"

/* ZCL strings: first byte = length */
#define ZB_MANUFACTURER_NAME "\x03""DIY"
#define ZB_MODEL_IDENTIFIER  "\x0f""ESP32H2-Shutter"

static atomic_bool s_zb_ready;
static atomic_int  s_battery_mv = -1; /* last reading at rest, -1 = none */
static atomic_int  s_solar_mv   = -1;

/* ------------------------------------------------------------------------- */
/* Helpers                                                                   */
/* ------------------------------------------------------------------------- */

static esp_timer_handle_t       s_retry_timer;
static ezb_bdb_comm_mode_mask_t s_retry_mode;

static void retry_timer_cb(void *arg)
{
    esp_zigbee_lock_acquire(portMAX_DELAY);
    (void)ezb_bdb_start_top_level_commissioning(s_retry_mode);
    esp_zigbee_lock_release();
}

/** Restarts a commissioning step after a delay (call from the Zigbee context). */
static void schedule_commissioning(ezb_bdb_comm_mode_mask_t mode, uint32_t delay_ms)
{
    if (!s_retry_timer) {
        const esp_timer_create_args_t args = {
            .callback = retry_timer_cb,
            .name     = "zb_retry",
        };
        ESP_ERROR_CHECK(esp_timer_create(&args, &s_retry_timer));
    }
    esp_timer_stop(s_retry_timer);
    s_retry_mode = mode;
    ESP_ERROR_CHECK(esp_timer_start_once(s_retry_timer, (uint64_t)delay_ms * 1000));
}

/* ------------------------------------------------------------------------- */
/* Shutter -> Zigbee                                                         */
/* ------------------------------------------------------------------------- */

#define UNCHANGED_REPORT_DELAY_MS 1000

static esp_timer_handle_t s_report_timer;

/** Sends the position to the coordinator now, via the binding ZHA set up for reporting. */
static void report_timer_cb(void *arg)
{
    ezb_zcl_report_attr_cmd_t cmd = {
        .cmd_ctrl = {
            .dst_addr   = {.addr_mode = EZB_ADDR_MODE_NONE},
            .src_ep     = SHUTTER_ENDPOINT,
            .cluster_id = EZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
            .manuf_code = EZB_ZCL_STD_MANUF_CODE,
            .fc         = {.direction = EZB_ZCL_CMD_DIRECTION_TO_CLI, .dis_default_rsp = 1},
        },
        .payload.attr_id = EZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID,
    };

    ESP_LOGI(TAG, "Position unchanged: reporting it");
    esp_zigbee_lock_acquire(portMAX_DELAY);
    ezb_err_t err = ezb_zcl_report_attr_cmd_req(&cmd);
    esp_zigbee_lock_release();
    if (err != EZB_ERR_NONE) {
        ESP_LOGW(TAG, "Reporting the position failed (%d)", err);
    }
}

static void shutter_state_changed(uint8_t lift_percent, shutter_motion_t motion)
{
    static const char *motion_str[] = {"stopped", "opening", "closing"};
    static int         published    = -1;

    ESP_LOGI(TAG, "Position %u %% (%s)", lift_percent, motion_str[motion]);
    if (!atomic_load(&s_zb_ready)) {
        return;
    }
    esp_zigbee_lock_acquire(portMAX_DELAY);
    ezb_zcl_status_t status = ezb_zcl_set_attr_value(SHUTTER_ENDPOINT, EZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
                                                     EZB_ZCL_CLUSTER_SERVER,
                                                     EZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID,
                                                     EZB_ZCL_STD_MANUF_CODE, &lift_percent, false);
    esp_zigbee_lock_release();
    if (status != EZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGW(TAG, "Setting the position failed (0x%02x)", status);
        return;
    }

    /* Reporting only sends changes. After a command that ends without a move (motor already at the
     * end switch, already at the target), ZHA would show "opening"/"closing" until its 5 min
     * timeout: send the unchanged position, which tells ZHA that the shutter stands still. ZHA only
     * sets "opening"/"closing" once the command's response has arrived, so wait for that first. */
    if (motion == SHUTTER_MOTION_STOPPED && lift_percent == published) {
        esp_timer_stop(s_report_timer);
        esp_timer_start_once(s_report_timer, UNCHANGED_REPORT_DELAY_MS * 1000);
    }
    published = lift_percent;
}

/* ------------------------------------------------------------------------- */
/* Battery and solar panel -> Zigbee                                         */
/* ------------------------------------------------------------------------- */

/** BatteryVoltage in 100 mV, 0xFF = unknown */
static uint8_t battery_voltage_attr(int mv)
{
    return (mv < 0) ? 0xFF : (uint8_t)((mv > 25400 ? 25400 : mv) / 100);
}

/** BatteryPercentageRemaining in 0.5 %, 0xFF = unknown */
static uint8_t battery_percent_attr(int mv)
{
    const int empty = CONFIG_SHUTTER_BATTERY_EMPTY_MV, full = CONFIG_SHUTTER_BATTERY_FULL_MV;

    if (mv < 0 || full <= empty) {
        return 0xFF;
    }
    if (mv <= empty) {
        return 0;
    }
    if (mv >= full) {
        return 200;
    }
    return (uint8_t)((mv - empty) * 200 / (full - empty));
}

static void set_attr(uint16_t cluster, uint16_t attr, void *value)
{
    ezb_zcl_status_t status = ezb_zcl_set_attr_value(SHUTTER_ENDPOINT, cluster, EZB_ZCL_CLUSTER_SERVER, attr,
                                                     EZB_ZCL_STD_MANUF_CODE, value, false);
    if (status != EZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGW(TAG, "Attribute 0x%04x/0x%04x: error 0x%02x", cluster, attr, status);
    }
}

/** Writes the last reading to the attributes (reported to ZHA via reporting). */
static void power_publish(void)
{
    int     battery_mv = atomic_load(&s_battery_mv), solar_mv = atomic_load(&s_solar_mv);
    uint8_t voltage = battery_voltage_attr(battery_mv), percent = battery_percent_attr(battery_mv);

    esp_zigbee_lock_acquire(portMAX_DELAY);
    if (battery_mv >= 0) {
        set_attr(EZB_ZCL_CLUSTER_ID_POWER_CONFIG, EZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID, &voltage);
        set_attr(EZB_ZCL_CLUSTER_ID_POWER_CONFIG, EZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID, &percent);
    }
    if (solar_mv >= 0) {
        float volts = (float)solar_mv / 1000.0f;
        set_attr(EZB_ZCL_CLUSTER_ID_ANALOG_INPUT, EZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, &volts);
    }
    esp_zigbee_lock_release();
}

static void power_measured(int battery_mv, int solar_mv)
{
    atomic_store(&s_battery_mv, battery_mv);
    atomic_store(&s_solar_mv, solar_mv);
    if (atomic_load(&s_zb_ready)) {
        power_publish();
    }
}

/* ------------------------------------------------------------------------- */
/* Zigbee -> shutter                                                         */
/* ------------------------------------------------------------------------- */

static void window_covering_movement_handler(ezb_zcl_window_covering_movement_message_t *message)
{
    ESP_RETURN_ON_FALSE(message && message->in.header, , TAG, "empty movement message");

    message->out.result = EZB_ZCL_STATUS_SUCCESS;
    if (message->info.dst_ep != SHUTTER_ENDPOINT) {
        return;
    }

    switch (message->in.header->cmd_id) {
    case EZB_ZCL_CMD_WINDOW_COVERING_UP_OPEN_ID:
        ESP_LOGI(TAG, "Zigbee: up");
        shutter_open();
        break;
    case EZB_ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE_ID:
        ESP_LOGI(TAG, "Zigbee: down");
        shutter_close();
        break;
    case EZB_ZCL_CMD_WINDOW_COVERING_STOP_ID:
        ESP_LOGI(TAG, "Zigbee: stop");
        shutter_stop();
        break;
    case EZB_ZCL_CMD_WINDOW_COVERING_GO_TO_LIFT_PERCENTAGE_ID:
        ESP_LOGI(TAG, "Zigbee: go to %u %%", message->in.payload.lift_percentage);
        if (message->in.payload.lift_percentage > 100) {
            message->out.result = EZB_ZCL_STATUS_INVALID_VALUE;
        } else {
            shutter_go_to_percent(message->in.payload.lift_percentage);
        }
        break;
    default:
        ESP_LOGW(TAG, "Unsupported Window Covering command 0x%02x", message->in.header->cmd_id);
        message->out.result = EZB_ZCL_STATUS_UNSUP_CMD;
        break;
    }
}

static void zcl_core_action_handler(ezb_zcl_core_action_callback_id_t callback_id, void *message)
{
    switch (callback_id) {
    case EZB_ZCL_CORE_WINDOW_COVERING_MOVEMENT_CB_ID:
        window_covering_movement_handler((ezb_zcl_window_covering_movement_message_t *)message);
        break;
    case EZB_ZCL_CORE_SET_ATTR_VALUE_CB_ID:
        /* e.g. identify time; not relevant for the shutter */
        break;
    default:
        if (!ota_handle_action(callback_id, message)) {
            ESP_LOGD(TAG, "ZCL Core Action: ID(0x%04lx)", (unsigned long)callback_id);
        }
        break;
    }
}

/* ------------------------------------------------------------------------- */
/* Zigbee network                                                            */
/* ------------------------------------------------------------------------- */

static bool app_signal_handler(const ezb_app_signal_t *app_signal)
{
    ezb_app_signal_type_t signal_type = ezb_app_signal_get_type(app_signal);

    switch (signal_type) {
    case EZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialising the Zigbee stack");
        ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_INITIALIZATION);
        break;
    case EZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case EZB_BDB_SIGNAL_DEVICE_REBOOT: {
        ezb_bdb_comm_status_t status = *((ezb_bdb_comm_status_t *)ezb_app_signal_get_params(app_signal));
        if (status == EZB_BDB_STATUS_SUCCESS) {
            if (ezb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Factory new - searching for a Zigbee network to join (enable pairing in ZHA!)");
                ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "Reboot - already joined to a network");
                ota_network_ready();
            }
        } else {
            ESP_LOGW(TAG, "%s failed (0x%02x), retrying", ezb_app_signal_to_string(signal_type), status);
            schedule_commissioning(EZB_BDB_MODE_INITIALIZATION, 1000);
        }
    } break;
    case EZB_BDB_SIGNAL_STEERING: {
        ezb_bdb_comm_status_t status = *((ezb_bdb_comm_status_t *)ezb_app_signal_get_params(app_signal));
        if (status == EZB_BDB_STATUS_SUCCESS) {
            ezb_extpanid_t extended_pan_id;
            ezb_nwk_get_extended_panid(&extended_pan_id);
            ESP_LOGI(TAG, "Joined network: PAN ID 0x%04hx (EXT 0x%llx), channel %d, address 0x%04hx",
                     ezb_nwk_get_panid(), extended_pan_id.u64, ezb_nwk_get_current_channel(),
                     ezb_nwk_get_short_address());
            ota_network_ready();
        } else {
            ESP_LOGI(TAG, "No network found (0x%02x), retrying in 5 s", status);
            schedule_commissioning(EZB_BDB_MODE_NETWORK_STEERING, 5000);
        }
    } break;
    case EZB_ZDO_SIGNAL_LEAVE: {
        const ezb_zdo_signal_leave_params_t *leave_params = ezb_app_signal_get_params(app_signal);
        ESP_LOGW(TAG, "Left the network (type 0x%02x)", leave_params->leave_type);
        if (leave_params->leave_type == EZB_ZDO_LEAVE_TYPE_RESET) {
            /* e.g. removed in ZHA: be ready for pairing again right away */
            schedule_commissioning(EZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
    } break;
    default:
        ESP_LOGI(TAG, "Zigbee Signal: %s (0x%02x)", ezb_app_signal_to_string(signal_type), signal_type);
        break;
    }
    return true;
}

static esp_err_t create_window_covering_device(void)
{
    ezb_af_device_desc_t             dev_desc = ezb_af_create_device_desc();
    ezb_zha_window_covering_config_t cfg      = EZB_ZHA_WINDOW_COVERING_CONFIG();

    cfg.basic_cfg.power_source = EZB_ZCL_BASIC_POWER_SOURCE_BATTERY;
    cfg.window_covering_cfg.window_covering_type = EZB_ZCL_WINDOW_COVERING_WINDOW_COVERING_TYPE_ROLLERSHADE;
    cfg.window_covering_cfg.config_status =
        EZB_ZCL_WINDOW_COVERING_CONFIG_STATUS_OPERATIONAL | EZB_ZCL_WINDOW_COVERING_CONFIG_STATUS_ONLINE;

    ezb_af_ep_desc_t ep_desc = ezb_zha_create_window_covering(SHUTTER_ENDPOINT, &cfg);

    ezb_zcl_cluster_desc_t basic_desc =
        ezb_af_endpoint_get_cluster_desc(ep_desc, EZB_ZCL_CLUSTER_ID_BASIC, EZB_ZCL_CLUSTER_SERVER);
    ezb_zcl_basic_cluster_desc_add_attr(basic_desc, EZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,
                                        (void *)ZB_MANUFACTURER_NAME);
    ezb_zcl_basic_cluster_desc_add_attr(basic_desc, EZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,
                                        (void *)ZB_MODEL_IDENTIFIER);
    /* Version from PROJECT_VER (CMakeLists.txt), ZCL string with length byte, max. 16 characters */
    static char sw_build_id[17];
    const char *version = esp_app_get_description()->version;
    sw_build_id[0]      = (char)strnlen(version, sizeof(sw_build_id) - 1);
    memcpy(&sw_build_id[1], version, (size_t)sw_build_id[0]);
    ezb_zcl_basic_cluster_desc_add_attr(basic_desc, EZB_ZCL_ATTR_BASIC_SW_BUILD_ID_ID, sw_build_id);

    /* Optional attribute for the position in percent - read and reported by ZHA */
    uint8_t lift_percent = shutter_get_percent();
    ezb_zcl_cluster_desc_t wc_desc =
        ezb_af_endpoint_get_cluster_desc(ep_desc, EZB_ZCL_CLUSTER_ID_WINDOW_COVERING, EZB_ZCL_CLUSTER_SERVER);
    ESP_RETURN_ON_FALSE(wc_desc, ESP_FAIL, TAG, "window covering cluster missing");
    ezb_zcl_window_covering_cluster_desc_add_attr(
        wc_desc, EZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID, &lift_percent);

    /* Battery: voltage and charge level, ZHA creates a battery sensor for it */
    ezb_zcl_cluster_desc_t power_desc = ezb_zcl_power_config_create_cluster_desc(NULL, EZB_ZCL_CLUSTER_SERVER);
    ESP_RETURN_ON_FALSE(power_desc, ESP_FAIL, TAG, "power config cluster failed");
    uint8_t battery_voltage = battery_voltage_attr(atomic_load(&s_battery_mv));
    uint8_t battery_percent = battery_percent_attr(atomic_load(&s_battery_mv));
    uint8_t battery_size    = EZB_ZCL_POWER_CONFIG_BATTERY_SIZE_OTHER;
    ezb_zcl_power_config_cluster_desc_add_attr(power_desc, EZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID,
                                               &battery_voltage);
    ezb_zcl_power_config_cluster_desc_add_attr(power_desc, EZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
                                               &battery_percent);
    ezb_zcl_power_config_cluster_desc_add_attr(power_desc, EZB_ZCL_ATTR_POWER_CONFIG_BATTERY_SIZE_ID, &battery_size);
    /* ZCL defines BatteryVoltage as not reportable, so ZHA's reporting configuration for it fails
     * (UNREPORTABLE_ATTRIBUTE) and the value only changes when ZHA polls: allow reporting */
    ezb_zcl_attr_desc_t voltage_attr = ezb_zcl_cluster_get_attr_desc(
        power_desc, EZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID, EZB_ZCL_STD_MANUF_CODE);
    ESP_RETURN_ON_FALSE(voltage_attr != EZB_INVALID_ZCL_ATTR_DESC, ESP_FAIL, TAG, "battery voltage attribute missing");
    ezb_zcl_attr_desc_set_access(voltage_attr,
                                 ezb_zcl_attr_desc_get_access(voltage_attr) | EZB_ZCL_ATTR_ACCESS_REPORTING);
    ESP_RETURN_ON_ERROR(ezb_af_endpoint_add_cluster_desc(ep_desc, power_desc), TAG, "add power config failed");

    /* Solar panel: voltage in volts */
    int solar_mv = atomic_load(&s_solar_mv);
    ezb_zcl_analog_input_cluster_server_config_t solar_cfg = {
        .out_of_service = false,
        .present_value  = solar_mv < 0 ? 0.0f : (float)solar_mv / 1000.0f,
        .status_flags   = EZB_ZCL_ANALOG_INPUT_STATUS_FLAGS_DEFAULT_VALUE,
    };
    ezb_zcl_cluster_desc_t solar_desc = ezb_zcl_analog_input_create_cluster_desc(&solar_cfg, EZB_ZCL_CLUSTER_SERVER);
    ESP_RETURN_ON_FALSE(solar_desc, ESP_FAIL, TAG, "analog input cluster failed");
    ezb_zcl_analog_input_cluster_desc_add_attr(solar_desc, EZB_ZCL_ATTR_ANALOG_INPUT_DESCRIPTION_ID,
                                               (void *)"\x0a""Solar Volt");
    /* ZHA only creates the sensor if the unit is known as well (BACnet engineering unit 5 = volts) */
    uint16_t solar_units = 5;
    ezb_zcl_analog_input_cluster_desc_add_attr(solar_desc, EZB_ZCL_ATTR_ANALOG_INPUT_ENGINEERING_UNITS_ID,
                                               &solar_units);
    ESP_RETURN_ON_ERROR(ezb_af_endpoint_add_cluster_desc(ep_desc, solar_desc), TAG, "add analog input failed");

    ESP_RETURN_ON_ERROR(ota_add_client_cluster(ep_desc, SHUTTER_ENDPOINT), TAG, "OTA cluster failed");

    ESP_RETURN_ON_ERROR(ezb_af_device_add_endpoint_desc(dev_desc, ep_desc), TAG, "add endpoint failed");
    ESP_RETURN_ON_ERROR(ezb_af_device_desc_register(dev_desc), TAG, "register device failed");
    ota_after_register();

    ezb_zcl_core_action_handler_register(zcl_core_action_handler);
    return ESP_OK;
}

static void factory_reset(void)
{
    esp_zigbee_lock_acquire(portMAX_DELAY);
    esp_zigbee_factory_reset(); /* erases the Zigbee data and restarts */
}

static void zigbee_task(void *pvParameters)
{
    esp_zigbee_config_t config = {
        .device_config =
            {
                .device_type         = EZB_NWK_DEVICE_TYPE_END_DEVICE,
                .install_code_policy = false,
                .zed_config =
                    {
                        .ed_timeout = EZB_NWK_ED_TIMEOUT_64MIN,
                        .keep_alive = CONFIG_SHUTTER_ZB_POLL_INTERVAL_MS,
                    },
            },
        .platform_config =
            {
                .storage_partition_name = ESP_ZIGBEE_STORAGE_PARTITION_NAME,
                .radio_config =
                    {
                        .radio_mode = ESP_ZIGBEE_RADIO_MODE_NATIVE,
                    },
            },
    };

    ESP_ERROR_CHECK(esp_zigbee_init(&config));
    ESP_ERROR_CHECK(ezb_bdb_set_primary_channel_set(CONFIG_SHUTTER_ZB_CHANNEL_MASK));
    ESP_ERROR_CHECK(ezb_bdb_set_secondary_channel_set(CONFIG_SHUTTER_ZB_CHANNEL_MASK));
    ESP_ERROR_CHECK(ezb_app_signal_add_handler(app_signal_handler));
    /* Sleepy end device: radio off between polls */
    ezb_nwk_set_rx_on_when_idle(false);
    /* Battery powered: the node descriptor otherwise reports constant (mains) power, and ZHA then
     * creates no battery sensor and uses the shorter availability timeout for mains devices */
    ezb_af_node_power_desc_t power_desc = {
        .current_power_mode         = EZB_AF_NODE_POWER_MODE_COME_ON_PERIODICALLY,
        .available_power_sources    = EZB_AF_NODE_POWER_SOURCE_RECHARGEABLE_BATTERY,
        .current_power_source       = EZB_AF_NODE_POWER_SOURCE_RECHARGEABLE_BATTERY,
        .current_power_source_level = EZB_AF_NODE_POWER_SOURCE_LEVEL_100_PERCENT,
    };
    ESP_ERROR_CHECK(ezb_af_set_node_power_desc(&power_desc));
    ESP_ERROR_CHECK(create_window_covering_device());
    atomic_store(&s_zb_ready, true);
    power_publish(); /* a reading that finished between creating the clusters and now */

    ESP_ERROR_CHECK(esp_zigbee_start(false));
    esp_zigbee_launch_mainloop();

    esp_zigbee_deinit();
    vTaskDelete(NULL);
}

#if CONFIG_PM_ENABLE
static esp_err_t power_save_init(void)
{
    esp_pm_config_t pm_config = {
        .max_freq_mhz       = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .min_freq_mhz       = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .light_sleep_enable = true,
    };
    ESP_RETURN_ON_ERROR(esp_pm_configure(&pm_config), TAG, "esp_pm_configure failed");
    /* Keys (reset, manual operation) wake from light sleep; with the peripherals
     * powered down, EXT1 takes over (inputs.c) */
#if !CONFIG_PM_POWER_DOWN_PERIPHERAL_IN_LIGHT_SLEEP
    ESP_RETURN_ON_ERROR(esp_sleep_enable_gpio_wakeup(), TAG, "gpio wakeup failed");
#endif
    return ESP_OK;
}
#endif

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
#if CONFIG_PM_ENABLE
    ESP_ERROR_CHECK(power_save_init());
#endif

    const esp_timer_create_args_t report_timer_args = {
        .callback = report_timer_cb,
        .name     = "zb_report",
    };
    ESP_ERROR_CHECK(esp_timer_create(&report_timer_args, &s_report_timer));

    shutter_set_power_cb(power_measured);
    ESP_ERROR_CHECK(shutter_init(shutter_state_changed));
    ESP_ERROR_CHECK(inputs_init(factory_reset));

    ESP_LOGI(TAG, "Starting the Zigbee stack");
    xTaskCreate(zigbee_task, "zigbee_main", 6144, NULL, 5, NULL);
}
