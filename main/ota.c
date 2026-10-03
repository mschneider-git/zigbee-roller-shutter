/*
 * Zigbee OTA: client of the OTA Upgrade cluster
 *
 * ZHA offers updates as OTA server (local folder, see README). The device asks
 * for a new image at start-up and then periodically, downloads it block by
 * block into the free OTA partition and restarts.
 *
 * With CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE a new image only boots on trial.
 * Only ota_network_ready() marks it as valid; without that, the bootloader
 * falls back to the old version after the next reset.
 *
 * The file parser (ota_file_parser.c) comes from the esp-zigbee-sdk example
 * examples/ota_upgrade/ota_client.
 */
#include "esp_check.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_timer.h"
#if CONFIG_PM_ENABLE
#include "esp_pm.h"
#endif

#include "esp_zigbee.h"

#include "ota.h"
#include "ota_file_parser.h"

static const char *TAG = "OTA";

#define OTA_BLOCK_SIZE       223 /* largest block that fits into a Zigbee frame */
#define OTA_COORDINATOR_ADDR 0x0000
#define OTA_LOG_STEP_PCT     10

static uint8_t                   s_ep;
static const esp_partition_t    *s_partition;
static esp_zb_ota_file_parser_t *s_parser;
static esp_ota_handle_t          s_handle;
static uint8_t                   s_next_log_pct;
#if CONFIG_SHUTTER_OTA_QUERY_INTERVAL_H > 0
static esp_timer_handle_t s_query_timer;
#endif
#if CONFIG_PM_ENABLE
static esp_pm_lock_handle_t s_pm_lock;
static bool                 s_pm_locked;
#endif

/* No light sleep during the download: more reliable, takes about 10-20 min */
static void keep_awake(bool awake)
{
#if CONFIG_PM_ENABLE
    if (!s_pm_lock && esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "ota", &s_pm_lock) != ESP_OK) {
        return;
    }
    if (awake && !s_pm_locked) {
        esp_pm_lock_acquire(s_pm_lock);
        s_pm_locked = true;
    } else if (!awake && s_pm_locked) {
        esp_pm_lock_release(s_pm_lock);
        s_pm_locked = false;
    }
#else
    (void)awake;
#endif
}

static void download_cleanup(bool abort)
{
    if (s_handle) {
        if (abort) {
            esp_ota_abort(s_handle);
        }
        s_handle = 0;
    }
    if (s_parser) {
        esp_zb_free_ota_file_parser(s_parser);
        s_parser = NULL;
    }
    keep_awake(false);
}

static esp_err_t write_block(const ezb_zcl_ota_upgrade_client_progress_message_t *message)
{
    esp_err_t ret;

    esp_zb_ota_file_parser_setup(s_parser, message->in.receiving.block_size, message->in.receiving.block);
    do {
        ret = esp_zb_ota_file_parser_process(s_parser);
        if (esp_zb_ota_file_parser_is_element_value(s_parser) && s_parser->element.type == UPGRADE_IMAGE) {
            ESP_RETURN_ON_FALSE(s_parser->element.total <= s_partition->size, ESP_ERR_INVALID_SIZE, TAG,
                                "Image larger than the OTA partition");
            ESP_RETURN_ON_ERROR(esp_ota_write(s_handle, s_parser->element.val, s_parser->element.length), TAG,
                                "Write failed");
        }
    } while (ret == ESP_ERR_NOT_FINISHED);

    uint32_t done = message->in.receiving.file_offset + message->in.receiving.block_size;
    uint8_t  pct  = (uint8_t)((uint64_t)done * 100 / s_parser->total_image_size);
    if (pct >= s_next_log_pct) {
        ESP_LOGI(TAG, "Download %u %% (%lu / %lu Bytes)", pct, (unsigned long)done,
                 (unsigned long)s_parser->total_image_size);
        s_next_log_pct = pct + OTA_LOG_STEP_PCT;
    }
    return ESP_OK;
}

static void client_progress_handler(ezb_zcl_ota_upgrade_client_progress_message_t *message)
{
    esp_err_t ret = ESP_OK;

    switch (message->in.progress) {
    case EZB_ZCL_OTA_UPGRADE_PROGRESS_START:
        ESP_LOGW(TAG, "Update starting: version 0x%08lx, %lu bytes (running: 0x%08x)",
                 (unsigned long)message->in.start.file_version, (unsigned long)message->in.start.image_size,
                 SHUTTER_OTA_FILE_VERSION);
        download_cleanup(true); /* leftovers of an aborted download */
        keep_awake(true);
        s_next_log_pct = 0;
        s_partition    = esp_ota_get_next_update_partition(NULL);
        ESP_GOTO_ON_FALSE(s_partition, ESP_ERR_NOT_FOUND, exit, TAG, "No free OTA partition");
        s_parser = esp_zb_create_ota_file_parser(message->in.start.image_size);
        ESP_GOTO_ON_FALSE(s_parser, ESP_ERR_NO_MEM, exit, TAG, "Could not create the parser");
        ESP_GOTO_ON_ERROR(esp_ota_begin(s_partition, 0, &s_handle), exit, TAG, "esp_ota_begin failed");
        break;
    case EZB_ZCL_OTA_UPGRADE_PROGRESS_RECEIVING:
        ESP_GOTO_ON_FALSE(s_parser && s_handle, ESP_ERR_INVALID_STATE, exit, TAG, "Block without start");
        ret = write_block(message);
        break;
    case EZB_ZCL_OTA_UPGRADE_PROGRESS_CHECK:
        ESP_GOTO_ON_FALSE(s_parser, ESP_ERR_INVALID_STATE, exit, TAG, "Check without download");
        ret = esp_zb_ota_file_parser_check(s_parser);
        ESP_LOGI(TAG, "Check %s", ret == ESP_OK ? "ok" : "failed");
        break;
    case EZB_ZCL_OTA_UPGRADE_PROGRESS_APPLY:
        ret = esp_ota_end(s_handle);
        s_handle = 0; /* esp_ota_end frees the handle in any case */
        ESP_GOTO_ON_ERROR(ret, exit, TAG, "Image invalid: %s", esp_err_to_name(ret));
        ret = esp_ota_set_boot_partition(s_partition);
        ESP_GOTO_ON_ERROR(ret, exit, TAG, "Setting the boot partition failed: %s", esp_err_to_name(ret));
        ESP_LOGW(TAG, "New image in %s, verified after the restart", s_partition->label);
        break;
    case EZB_ZCL_OTA_UPGRADE_PROGRESS_FINISH:
        ESP_LOGW(TAG, "Update finished, restarting");
        download_cleanup(false);
        esp_restart();
        break;
    case EZB_ZCL_OTA_UPGRADE_PROGRESS_ABORT:
        ESP_LOGW(TAG, "Update aborted");
        download_cleanup(true);
        break;
    default:
        break;
    }

exit:
    if (ret != ESP_OK) {
        download_cleanup(true);
    }
    message->out.result = (ret == ESP_OK) ? EZB_ZCL_STATUS_SUCCESS : EZB_ZCL_STATUS_ABORT;
}

static void query_next_image_rsp_handler(ezb_zcl_ota_upgrade_query_next_image_rsp_message_t *message)
{
    if (message->in.image.status == EZB_ZCL_OTA_UPGRADE_STATUS_CODE_SUCCESS) {
        ESP_LOGW(TAG, "Update available: version 0x%08lx, %lu bytes", (unsigned long)message->in.image.file_version,
                 (unsigned long)message->in.image.size);
    } else {
        ESP_LOGI(TAG, "No update available (0x%02x)", message->in.image.status);
    }
    message->out.result = EZB_ZCL_STATUS_SUCCESS;
}

/** Call in the Zigbee context (lock held). */
static void query_next_image(void)
{
    if (s_handle) {
        return; /* download already running */
    }
    ezb_zcl_ota_upgrade_query_next_image_req_cmd_t req = {
        .cmd_ctrl =
            {
                .dst_addr = {.addr_mode = EZB_ADDR_MODE_SHORT, .u.short_addr = OTA_COORDINATOR_ADDR},
                .dst_ep   = 0xFF,
                .src_ep   = s_ep,
            },
        .payload =
            {
                .manuf_code   = CONFIG_SHUTTER_OTA_MANUF_CODE,
                .image_type   = CONFIG_SHUTTER_OTA_IMAGE_TYPE,
                .file_version = SHUTTER_OTA_FILE_VERSION,
            },
    };
    ezb_err_t err = ezb_zcl_ota_upgrade_query_next_image_cmd_req(&req);
    if (err != EZB_ERR_NONE) {
        ESP_LOGW(TAG, "Update query failed (%d)", err);
    }
}

#if CONFIG_SHUTTER_OTA_QUERY_INTERVAL_H > 0
static void query_timer_cb(void *arg)
{
    esp_zigbee_lock_acquire(portMAX_DELAY);
    query_next_image();
    esp_zigbee_lock_release();
}
#endif

esp_err_t ota_add_client_cluster(ezb_af_ep_desc_t ep_desc, uint8_t ep_id)
{
    ezb_zcl_ota_upgrade_cluster_client_config_t cfg = {
        .upgrade_server_id    = EZB_ZCL_OTA_UPGRADE_UPGRADE_SERVER_ID_DEFAULT_VALUE,
        .file_offset          = 0,
        .image_upgrade_status = EZB_ZCL_OTA_UPGRADE_IMAGE_UPGRADE_STATUS_DEFAULT_VALUE,
        .manufacturer_id      = CONFIG_SHUTTER_OTA_MANUF_CODE,
        .image_type_id        = CONFIG_SHUTTER_OTA_IMAGE_TYPE,
    };

    s_ep = ep_id;
    ezb_zcl_cluster_desc_t desc = ezb_zcl_ota_upgrade_create_cluster_desc(&cfg, EZB_ZCL_CLUSTER_CLIENT);
    ESP_RETURN_ON_FALSE(desc, ESP_FAIL, TAG, "Could not create the OTA cluster");
    ESP_RETURN_ON_ERROR(ezb_zcl_ota_upgrade_cluster_desc_add_attr(desc, EZB_ZCL_ATTR_OTA_UPGRADE_CURRENT_FILE_VERSION_ID,
                                                                  &(uint32_t){SHUTTER_OTA_FILE_VERSION}),
                        TAG, "Version attribute failed");
    return ezb_af_endpoint_add_cluster_desc(ep_desc, desc);
}

void ota_after_register(void)
{
    ezb_zcl_ota_upgrade_set_download_block_size(s_ep, OTA_BLOCK_SIZE);
}

bool ota_handle_action(ezb_zcl_core_action_callback_id_t callback_id, void *message)
{
    switch (callback_id) {
    case EZB_ZCL_CORE_OTA_UPGRADE_CLIENT_PROGRESS_CB_ID:
        client_progress_handler(message);
        return true;
    case EZB_ZCL_CORE_OTA_UPGRADE_QUERY_NEXT_IMAGE_RSP_CB_ID:
        query_next_image_rsp_handler(message);
        return true;
    default:
        return false;
    }
}

void ota_network_ready(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t   state;

    if (esp_ota_get_state_partition(running, &state) == ESP_OK && state == ESP_OTA_IMG_PENDING_VERIFY) {
        if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK) {
            ESP_LOGW(TAG, "New image (0x%08x) confirmed", SHUTTER_OTA_FILE_VERSION);
        }
    }

    query_next_image();

#if CONFIG_SHUTTER_OTA_QUERY_INTERVAL_H > 0
    if (!s_query_timer) {
        const esp_timer_create_args_t args = {.callback = query_timer_cb, .name = "ota_query"};
        if (esp_timer_create(&args, &s_query_timer) != ESP_OK) {
            return;
        }
        esp_timer_start_periodic(s_query_timer, (uint64_t)CONFIG_SHUTTER_OTA_QUERY_INTERVAL_H * 3600 * 1000000);
    }
#endif
}
