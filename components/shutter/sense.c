/*
 * Voltage sensing via dedicated dividers (see sense.h).
 *
 * All channels are on ADC1 and share one ADC unit. It is set up on the first
 * reading and released again with sense_release() before light sleep (the SAR
 * ADC has no sleep retention).
 */
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"

#include "sense.h"

static const char *TAG = "SENSE";

#define DIVIDER_TOTAL_OHM ((int64_t)CONFIG_SHUTTER_SENSE_R_TOP_OHM + CONFIG_SHUTTER_SENSE_R_BOTTOM_OHM)

static const int s_gpio[SENSE_COUNT] = {
    [SENSE_BATTERY] = CONFIG_SHUTTER_BATTERY_ADC_GPIO,
    [SENSE_MOTOR_P] = CONFIG_SHUTTER_MOTOR_P_ADC_GPIO,
    [SENSE_MOTOR_N] = CONFIG_SHUTTER_MOTOR_N_ADC_GPIO,
    [SENSE_SOLAR]   = CONFIG_SHUTTER_SOLAR_ADC_GPIO,
};
static const char *const s_name[SENSE_COUNT] = {"Battery", "Motor+", "Motor-", "Solar"};

static SemaphoreHandle_t         s_lock;
static adc_unit_t                s_unit;
static adc_oneshot_unit_handle_t s_adc; /* only between the first reading and sense_release() */
static adc_channel_t             s_chan[SENSE_COUNT];
static adc_cali_handle_t         s_cali[SENSE_COUNT];

/* 12 dB: range up to about 2.5 V */
#define SENSE_ATTEN ADC_ATTEN_DB_12

esp_err_t sense_init(void)
{
    if (s_lock) {
        return ESP_OK; /* already set up */
    }
    bool have_unit = false;

    for (int ch = 0; ch < SENSE_COUNT; ch++) {
        if (s_gpio[ch] < 0) {
            continue;
        }
        adc_unit_t unit;
        ESP_RETURN_ON_ERROR(adc_oneshot_io_to_channel(s_gpio[ch], &unit, &s_chan[ch]), TAG, "GPIO%d (%s) is not an ADC pin",
                            s_gpio[ch], s_name[ch]);
        ESP_RETURN_ON_FALSE(!have_unit || unit == s_unit, ESP_ERR_INVALID_ARG, TAG,
                            "%s: all channels must be on the same ADC unit", s_name[ch]);
        s_unit    = unit;
        have_unit = true;

        adc_cali_curve_fitting_config_t cali_cfg = {
            .unit_id  = unit,
            .chan     = s_chan[ch],
            .atten    = SENSE_ATTEN,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ESP_RETURN_ON_ERROR(adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali[ch]), TAG,
                            "%s: ADC calibration failed", s_name[ch]);
        ESP_LOGI(TAG, "%s on GPIO%d", s_name[ch], s_gpio[ch]);
    }
    ESP_RETURN_ON_FALSE(have_unit, ESP_ERR_NOT_SUPPORTED, TAG, "no channel configured");

    s_lock = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(s_lock, ESP_ERR_NO_MEM, TAG, "mutex alloc failed");
    ESP_LOGI(TAG, "Divider %d / %d Ohm", CONFIG_SHUTTER_SENSE_R_TOP_OHM, CONFIG_SHUTTER_SENSE_R_BOTTOM_OHM);
    return ESP_OK;
}

bool sense_available(sense_ch_t ch)
{
    return s_lock && ch < SENSE_COUNT && s_cali[ch];
}

/** Sets up the ADC unit and all channels if needed. Call with s_lock held. */
static esp_err_t adc_acquire(void)
{
    if (s_adc) {
        return ESP_OK;
    }
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = s_unit,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_cfg, &s_adc), TAG, "adc unit failed");

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = SENSE_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    for (int ch = 0; ch < SENSE_COUNT; ch++) {
        if (s_cali[ch]) {
            esp_err_t err = adc_oneshot_config_channel(s_adc, s_chan[ch], &chan_cfg);
            if (err != ESP_OK) {
                adc_oneshot_del_unit(s_adc);
                s_adc = NULL;
                ESP_RETURN_ON_ERROR(err, TAG, "adc channel failed");
            }
        }
    }
    return ESP_OK;
}

esp_err_t sense_read_mv(sense_ch_t ch, uint8_t samples, int *out_mv)
{
    ESP_RETURN_ON_FALSE(sense_available(ch) && out_mv && samples > 0, ESP_ERR_INVALID_STATE, TAG, "not initialized");

    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t err = adc_acquire();
    int32_t   sum = 0;
    for (uint8_t i = 0; i < samples && err == ESP_OK; i++) {
        int mv;
        err = adc_oneshot_get_calibrated_result(s_adc, s_cali[ch], s_chan[ch], &mv);
        sum += mv;
    }
    xSemaphoreGive(s_lock);
    ESP_RETURN_ON_ERROR(err, TAG, "%s: adc read failed", s_name[ch]);

    *out_mv = (int)((int64_t)sum * DIVIDER_TOTAL_OHM / ((int64_t)samples * CONFIG_SHUTTER_SENSE_R_BOTTOM_OHM));
    return ESP_OK;
}

void sense_release(void)
{
    if (!s_lock) {
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_adc) {
        adc_oneshot_del_unit(s_adc);
        s_adc = NULL;
    }
    xSemaphoreGive(s_lock);
}
