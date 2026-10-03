/*
 * Inputs: reset button (BOOT) and, optionally, detection of manual operation
 * on the membrane keypad.
 *
 * Power saving: all inputs trigger a low-level interrupt that also wakes the
 * chip from light sleep. Only then the task polls and debounces briefly; once
 * all keys are released, it sleeps again.
 *
 * If the peripherals are powered down in light sleep
 * (CONFIG_PM_POWER_DOWN_PERIPHERAL_IN_LIGHT_SLEEP), the GPIO interrupt no
 * longer wakes the chip. The inputs then also wake it via EXT1 (ESP32-H2:
 * GPIO7-14 only); after wake-up the still present low level triggers the
 * normal interrupt.
 */
#include <stdbool.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#if CONFIG_PM_ENABLE
#include "esp_pm.h"
#endif
#if CONFIG_PM_POWER_DOWN_PERIPHERAL_IN_LIGHT_SLEEP
#include "esp_sleep.h"
#endif

#include "inputs.h"
#include "shutter.h"

static const char *TAG = "INPUTS";

#define POLL_MS        20
#define DEBOUNCE_COUNT 3
#define RESET_HOLD_MS  5000
#define STUCK_MS       60000 /* held longer = line stuck, stop monitoring it */

typedef enum { IN_RESET, IN_OPEN, IN_CLOSE, IN_STOP, IN_COUNT } input_id_t;

typedef struct {
    int      gpio;
    bool     pressed;
    uint8_t  counter;
    uint32_t held_ms;
} input_t;

static input_t           s_inputs[IN_COUNT];
static inputs_reset_cb_t s_reset_cb;
static TaskHandle_t      s_task;
#if CONFIG_PM_ENABLE
static esp_pm_lock_handle_t s_pm_lock;
#endif

static void IRAM_ATTR input_isr(void *arg)
{
    BaseType_t woken = pdFALSE;

    gpio_intr_disable((gpio_num_t)(intptr_t)arg);
    vTaskNotifyGiveFromISR(s_task, &woken);
    portYIELD_FROM_ISR(woken);
}

/** Debounces an input; returns true exactly at the moment of the press. */
static bool input_update(input_t *in)
{
    if (in->gpio < 0) {
        return false;
    }
    /* Do not count the firmware's own key presses as manual operation */
    if (shutter_output_is_driving(in->gpio)) {
        in->pressed = false;
        in->counter = 0;
        return false;
    }
    bool raw = gpio_get_level(in->gpio) == 0;
    if (raw == in->pressed) {
        in->counter = 0;
    } else if (++in->counter >= DEBOUNCE_COUNT) {
        in->counter = 0;
        in->pressed = raw;
        in->held_ms = 0;
        return raw;
    }
    if (in->pressed) {
        in->held_ms += POLL_MS;
    }
    return false;
}

static bool any_active(void)
{
    for (int i = 0; i < IN_COUNT; i++) {
        if (s_inputs[i].gpio >= 0 &&
            (s_inputs[i].pressed || s_inputs[i].counter || shutter_output_is_driving(s_inputs[i].gpio))) {
            return true;
        }
    }
    return false;
}

static void inputs_task(void *arg)
{
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
#if CONFIG_PM_ENABLE
        esp_pm_lock_acquire(s_pm_lock);
#endif
        bool reset_fired = false;
        do {
            if (input_update(&s_inputs[IN_OPEN])) {
                shutter_manual_press(SHUTTER_KEY_OPEN);
            }
            if (input_update(&s_inputs[IN_CLOSE])) {
                shutter_manual_press(SHUTTER_KEY_CLOSE);
            }
            if (input_update(&s_inputs[IN_STOP])) {
                shutter_manual_press(SHUTTER_KEY_STOP);
            }
            input_update(&s_inputs[IN_RESET]);
            for (int i = 0; i < IN_COUNT; i++) {
                if (s_inputs[i].gpio >= 0 && s_inputs[i].pressed && s_inputs[i].held_ms >= STUCK_MS) {
                    ESP_LOGE(TAG, "GPIO%d permanently low - input ignored (check the wiring!)",
                             s_inputs[i].gpio);
                    gpio_intr_disable(s_inputs[i].gpio);
#if CONFIG_PM_POWER_DOWN_PERIPHERAL_IN_LIGHT_SLEEP
                    esp_sleep_disable_ext1_wakeup_io(1ULL << s_inputs[i].gpio); /* otherwise it would wake constantly */
#endif
                    s_inputs[i].gpio = -1;
                }
            }
            if (s_inputs[IN_RESET].pressed && !reset_fired && s_inputs[IN_RESET].held_ms >= RESET_HOLD_MS) {
                reset_fired = true;
                ESP_LOGW(TAG, "Reset button held -> Zigbee factory reset");
                if (s_reset_cb) {
                    s_reset_cb();
                }
            }
            vTaskDelay(pdMS_TO_TICKS(POLL_MS));
        } while (any_active());

        for (int i = 0; i < IN_COUNT; i++) {
            if (s_inputs[i].gpio >= 0) {
                gpio_intr_enable(s_inputs[i].gpio);
            }
        }
#if CONFIG_PM_ENABLE
        esp_pm_lock_release(s_pm_lock);
#endif
    }
}

static esp_err_t input_setup(input_t *in, bool pull_up)
{
    if (in->gpio < 0) {
        return ESP_OK;
    }
    ESP_RETURN_ON_FALSE(GPIO_IS_VALID_GPIO(in->gpio), ESP_ERR_INVALID_ARG, TAG, "invalid gpio %d", in->gpio);
    if (pull_up) {
        /* The keypad outputs are already configured by shutter.c (open drain with input) */
        gpio_config_t cfg = {
            .pin_bit_mask = 1ULL << in->gpio,
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        ESP_RETURN_ON_ERROR(gpio_config(&cfg), TAG, "gpio_config failed");
    }
    ESP_RETURN_ON_ERROR(gpio_set_intr_type(in->gpio, GPIO_INTR_LOW_LEVEL), TAG, "intr type failed");
#if CONFIG_PM_ENABLE
    ESP_RETURN_ON_ERROR(gpio_wakeup_enable(in->gpio, GPIO_INTR_LOW_LEVEL), TAG, "wakeup enable failed");
#endif
#if CONFIG_PM_POWER_DOWN_PERIPHERAL_IN_LIGHT_SLEEP
    /* For EXT1, ESP-IDF holds the pad including the pull-up from gpio_config(),
     * so the input stays defined during sleep. */
    ESP_RETURN_ON_ERROR(esp_sleep_enable_ext1_wakeup_io(1ULL << in->gpio, ESP_EXT1_WAKEUP_ANY_LOW), TAG,
                        "GPIO%d cannot wake via EXT1", in->gpio);
#endif
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(in->gpio, input_isr, (void *)(intptr_t)in->gpio), TAG,
                        "isr add failed");
    return ESP_OK;
}

esp_err_t inputs_init(inputs_reset_cb_t reset_cb)
{
    s_reset_cb = reset_cb;
    for (int i = 0; i < IN_COUNT; i++) {
        s_inputs[i].gpio = -1;
    }
    s_inputs[IN_RESET].gpio = CONFIG_SHUTTER_GPIO_RESET_BUTTON;
#if CONFIG_SHUTTER_SENSE_MANUAL_PRESS
    s_inputs[IN_OPEN].gpio  = SHUTTER_GPIO_OPEN;
    s_inputs[IN_CLOSE].gpio = SHUTTER_GPIO_CLOSE;
    s_inputs[IN_STOP].gpio  = SHUTTER_GPIO_STOP;
#endif

#if CONFIG_PM_ENABLE
    ESP_RETURN_ON_ERROR(esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "inputs", &s_pm_lock), TAG,
                        "pm lock create failed");
#endif
    ESP_RETURN_ON_FALSE(xTaskCreate(inputs_task, "inputs", 2560, NULL, 4, &s_task) == pdPASS, ESP_ERR_NO_MEM, TAG,
                        "task create failed");

    esp_err_t err = gpio_install_isr_service(0);
    ESP_RETURN_ON_FALSE(err == ESP_OK || err == ESP_ERR_INVALID_STATE, err, TAG, "isr service failed");

    for (int i = 0; i < IN_COUNT; i++) {
        ESP_RETURN_ON_ERROR(input_setup(&s_inputs[i], i == IN_RESET), TAG, "input setup failed");
        if (s_inputs[i].gpio >= 0) {
            /* Keys already pressed at start-up do not count as a new press */
            s_inputs[i].pressed = gpio_get_level(s_inputs[i].gpio) == 0;
        }
    }
    for (int i = 0; i < IN_COUNT; i++) {
        if (s_inputs[i].gpio >= 0) {
            gpio_intr_enable(s_inputs[i].gpio);
        }
    }
    return ESP_OK;
}
