/*
 * Roller shutter drive with time-based position tracking.
 *
 * All state changes happen exclusively in the module's own FreeRTOS task.
 * Other tasks (Zigbee, key detection) only send commands through a queue.
 * When idle the task blocks completely so the chip can sleep.
 *
 * With CONFIG_SHUTTER_BATTERY_SENSE the motor current is detected from the
 * sag of the battery voltage: after each key press the firmware checks that
 * the motor starts (otherwise it presses again, e.g. because the controller
 * keeps running for about 30 s after an end position and takes the press as
 * stop), and the end position is detected when the motor switches off.
 *
 * If both motor terminals are connected as well, their voltage shows directly
 * whether the controller switched the motor on: a key press taken as stop is
 * detected immediately, as is a move to an end position where the shutter
 * already is (voltage, but no motor current).
 *
 * At rest the battery and the solar panel are measured every
 * CONFIG_SHUTTER_MONITOR_INTERVAL_S seconds and reported via shutter_set_power_cb().
 */
#include <stdatomic.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs.h"
#if CONFIG_PM_ENABLE
#include "esp_pm.h"
#endif

#include "sense.h"
#include "shutter.h"

static const char *TAG = "SHUTTER";

/* Internal position in 1/1000 percent: 0 = open, POS_MAX = closed */
#define POS_MAX     100000
#define POS_PER_PCT (POS_MAX / 100)
#define TICK_MS     50

#define NVS_NAMESPACE "shutter"
#define NVS_KEY_POS   "pos"

#if CONFIG_SHUTTER_OUTPUT_OPEN_DRAIN
#define LEVEL_ON        0
#define LEVEL_OFF       1 /* open drain: 1 = high impedance */
#define OUTPUT_GPIO_MODE GPIO_MODE_INPUT_OUTPUT_OD
#else
#define LEVEL_ON        1
#define LEVEL_OFF       0
#define OUTPUT_GPIO_MODE GPIO_MODE_OUTPUT
#endif

#if CONFIG_SHUTTER_OUTPUT_MODE_HOLD
#define OUTPUT_MODE_STR "HOLD"
#else
#define OUTPUT_MODE_STR "PULSE"
#endif

typedef enum {
    CMD_GOTO,
    CMD_STOP,
    CMD_MANUAL_OPEN,
    CMD_MANUAL_CLOSE,
    CMD_MANUAL_STOP,
} shutter_cmd_type_t;

typedef struct {
    shutter_cmd_type_t type;
    int32_t            target; /* CMD_GOTO: position units */
} shutter_cmd_t;

typedef enum {
    PHASE_IDLE,    /* motor stopped */
    PHASE_PAUSE,   /* motor stopped, waiting before reversing */
    PHASE_MOVING,  /* motor moving towards the target */
    PHASE_OVERRUN, /* end position reached by calculation, safety margin */
} shutter_phase_t;

static QueueHandle_t      s_queue;
static shutter_state_cb_t s_state_cb;
static shutter_power_cb_t s_power_cb;
static atomic_uint        s_percent_cache;
static atomic_ullong      s_driving_mask; /* GPIOs currently being driven */
#if CONFIG_PM_ENABLE
static esp_pm_lock_handle_t s_pm_lock;
static bool                 s_pm_locked;
#endif

/* Only used in the shutter task */
static int32_t          s_pos;
static int32_t          s_target;
static shutter_motion_t s_motion = SHUTTER_MOTION_STOPPED;
static shutter_phase_t  s_phase  = PHASE_IDLE;
static int64_t          s_last_update_us;
static int64_t          s_phase_until_us;
static int64_t          s_last_report_us;
static uint8_t          s_last_reported_pct    = 0xFF;
static shutter_motion_t s_last_reported_motion = (shutter_motion_t)0xFF;

static inline uint8_t pos_to_pct(int32_t pos)
{
    return (uint8_t)((pos + POS_PER_PCT / 2) / POS_PER_PCT);
}

static inline bool is_end_position(int32_t pos)
{
    return pos <= 0 || pos >= POS_MAX;
}

/* ------------------------------------------------------------------------- */
/* Power saving: no light sleep while moving (outputs + timing)            */
/* ------------------------------------------------------------------------- */

static void pm_keep_awake(bool awake)
{
#if CONFIG_PM_ENABLE
    if (awake && !s_pm_locked) {
        esp_pm_lock_acquire(s_pm_lock);
        s_pm_locked = true;
    } else if (!awake && s_pm_locked) {
        sense_release(); /* the ADC registers do not survive sleep */
        esp_pm_lock_release(s_pm_lock);
        s_pm_locked = false;
    }
#else
    (void)awake;
#endif
}

/* ------------------------------------------------------------------------- */
/* Outputs                                                                   */
/* ------------------------------------------------------------------------- */

static void output_set(int gpio, bool on)
{
    if (gpio < 0) {
        return;
    }
    if (on) {
        atomic_fetch_or(&s_driving_mask, 1ULL << gpio);
        gpio_set_level(gpio, LEVEL_ON);
    } else {
        gpio_set_level(gpio, LEVEL_OFF);
        /* Only read the line as input again once it has safely gone high */
        vTaskDelay(pdMS_TO_TICKS(30));
        atomic_fetch_and(&s_driving_mask, ~(1ULL << gpio));
    }
}

#if CONFIG_SHUTTER_OUTPUT_MODE_PULSE
static void output_pulse(int gpio)
{
    output_set(gpio, true);
    vTaskDelay(pdMS_TO_TICKS(CONFIG_SHUTTER_PULSE_MS));
    output_set(gpio, false);
}
#endif

/* ------------------------------------------------------------------------- */
/* Motor current from the battery voltage                                    */
/* ------------------------------------------------------------------------- */

#if CONFIG_SHUTTER_BATTERY_SENSE && CONFIG_SHUTTER_OUTPUT_MODE_PULSE
#define MOTOR_SENSE 1
#else
#define MOTOR_SENSE 0
#endif

#if MOTOR_SENSE
#define SENSE_SAMPLES      10   /* one block: 10 x 10 ms */
#define SENSE_SAMPLE_MS    10
#define SENSE_CONFIRM      3    /* a step must last 3 blocks (about 300 ms) */
#define SENSE_END_GRACE_MS 5000 /* at the end position, wait longer for the switch-off */

static bool s_sense_ok;   /* divider connected, voltage plausible */
static int  s_run_mv;     /* battery voltage while moving (tracked) */
static int  s_rise_count; /* consecutive blocks above the running level */

/** Average over one block (about 100 ms), -1 on error. */
static int sense_block(void)
{
    int32_t sum = 0;

    for (int i = 0; i < SENSE_SAMPLES; i++) {
        int mv;
        if (sense_read_mv(SENSE_BATTERY, 4, &mv) != ESP_OK) {
            return -1;
        }
        sum += mv;
        vTaskDelay(pdMS_TO_TICKS(SENSE_SAMPLE_MS));
    }
    return (int)(sum / SENSE_SAMPLES);
}

/** Idle level before a key press; also checks whether there is a reading at all. */
static int sense_idle(void)
{
    if (!sense_available(SENSE_BATTERY)) {
        s_sense_ok = false;
        return -1;
    }
    int mv = sense_block();

    s_sense_ok = mv >= CONFIG_SHUTTER_BATTERY_MIN_MV;
    if (!s_sense_ok) {
        ESP_LOGW(TAG, "Battery voltage implausible (%d mV), time-based only", mv);
    }
    return mv;
}

/** After a key press: waits for the battery voltage to sag. */
static bool sense_wait_start(int idle_mv)
{
    int64_t until = esp_timer_get_time() + (int64_t)CONFIG_SHUTTER_MOTOR_START_TIMEOUT_MS * 1000;
    int     hits  = 0;

    while (esp_timer_get_time() < until) {
        int mv = sense_block();
        if (mv >= 0 && idle_mv - mv >= CONFIG_SHUTTER_MOTOR_STEP_MV) {
            if (++hits >= SENSE_CONFIRM) {
                s_run_mv     = mv;
                s_rise_count = 0;
                return true;
            }
        } else {
            hits = 0;
        }
    }
    return false;
}

/** While moving: true as soon as the battery voltage has risen again. */
static bool sense_motor_stopped(void)
{
    int mv = sense_block();

    if (mv < 0) {
        return false;
    }
    if (mv - s_run_mv >= CONFIG_SHUTTER_MOTOR_STEP_MV) {
        return ++s_rise_count >= SENSE_CONFIRM;
    }
    s_rise_count = 0;
    s_run_mv += (mv - s_run_mv) / 4;
    return false;
}

/** After a stop press: waits for the battery voltage to rise. */
static bool sense_wait_stop(void)
{
    int64_t until = esp_timer_get_time() + (int64_t)CONFIG_SHUTTER_MOTOR_START_TIMEOUT_MS * 1000;

    while (esp_timer_get_time() < until) {
        if (sense_motor_stopped()) {
            return true;
        }
    }
    return false;
}
#endif /* MOTOR_SENSE */

/* ------------------------------------------------------------------------- */
/* Motor terminal voltage: has the controller switched the motor on?        */
/* ------------------------------------------------------------------------- */

#define VOLT_POLL_MS 50

static bool s_volt_ok; /* both motor terminals connected */

static void volt_init(void)
{
    s_volt_ok = sense_available(SENSE_MOTOR_P) && sense_available(SENSE_MOTOR_N);
}

/** At rest both terminals are without voltage; when powered, one is at battery voltage. */
static bool motor_powered(void)
{
    int p = 0, n = 0;

    if (sense_read_mv(SENSE_MOTOR_P, 4, &p) != ESP_OK || sense_read_mv(SENSE_MOTOR_N, 4, &n) != ESP_OK) {
        return false;
    }
    return p >= CONFIG_SHUTTER_MOTOR_VOLTAGE_MIN_MV || n >= CONFIG_SHUTTER_MOTOR_VOLTAGE_MIN_MV;
}

/** Waits until the motor voltage reaches state @p on; false on timeout. */
static bool wait_powered(bool on, uint32_t timeout_ms)
{
    int64_t until = esp_timer_get_time() + (int64_t)timeout_ms * 1000;

    do {
        if (motor_powered() == on) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(VOLT_POLL_MS));
    } while (esp_timer_get_time() < until);
    return false;
}

/* How long the controller may take to react to a key press */
#if CONFIG_SHUTTER_BATTERY_SENSE
#define REACT_TIMEOUT_MS CONFIG_SHUTTER_MOTOR_START_TIMEOUT_MS
#define START_RETRIES    CONFIG_SHUTTER_MOTOR_START_RETRIES
#else
#define REACT_TIMEOUT_MS 1500
#define START_RETRIES    2
#endif

static int64_t s_motor_start_us; /* start of the key press that started the motor */
static bool    s_start_at_end;   /* motor_start(): powered, but the motor is already at the end position */

/** @return false if the motor demonstrably did not start */
static bool motor_start(shutter_motion_t dir)
{
    int         gpio_on = (dir == SHUTTER_MOTION_OPENING) ? SHUTTER_GPIO_OPEN : SHUTTER_GPIO_CLOSE;
    const char *name    = (dir == SHUTTER_MOTION_OPENING) ? "UP" : "DOWN";

#if CONFIG_SHUTTER_OUTPUT_MODE_HOLD
    /* Interlock: never both directions at once */
    output_set(dir == SHUTTER_MOTION_OPENING ? SHUTTER_GPIO_CLOSE : SHUTTER_GPIO_OPEN, false);
    s_motor_start_us = esp_timer_get_time();
    output_set(gpio_on, true);
    ESP_LOGI(TAG, "Key %s", name);
    return true;
#else
    s_start_at_end = false;
#if MOTOR_SENSE
    int idle_mv = sense_idle();
#endif
    if (s_volt_ok) {
        for (int attempt = 0; attempt <= START_RETRIES; attempt++) {
            bool was_on = motor_powered();
            ESP_LOGI(TAG, "Key %s%s", name, attempt ? " again" : "");
            s_motor_start_us = esp_timer_get_time();
            output_pulse(gpio_on);
            if (!wait_powered(true, REACT_TIMEOUT_MS)) {
                /* If the controller was still running (run-on after an end position), the press was a stop */
                ESP_LOGW(TAG, "%s", was_on ? "Press was taken as stop" : "Controller does not react");
                continue;
            }
#if MOTOR_SENSE
            if (s_sense_ok) {
                if (sense_wait_start(idle_mv)) {
                    ESP_LOGI(TAG, "Motor running (%d -> %d mV)", idle_mv, s_run_mv);
                    return true;
                }
                /* Powered but no motor current: the motor's end switch has already tripped */
                ESP_LOGI(TAG, "Motor powered but not running: already %s",
                         dir == SHUTTER_MOTION_OPENING ? "up" : "down");
                s_start_at_end = true;
                return false;
            }
#endif
            ESP_LOGI(TAG, "Motor powered");
            return true;
        }
        ESP_LOGW(TAG, "Motor cannot be switched on");
        return false;
    }
#if MOTOR_SENSE
    for (int attempt = 0; s_sense_ok && attempt <= CONFIG_SHUTTER_MOTOR_START_RETRIES; attempt++) {
        if (attempt == 0) {
            ESP_LOGI(TAG, "Key %s", name);
        } else {
            ESP_LOGW(TAG, "Motor not running, key %s again (retry %d)", name, attempt);
        }
        s_motor_start_us = esp_timer_get_time();
        output_pulse(gpio_on);
        if (sense_wait_start(idle_mv)) {
            ESP_LOGI(TAG, "Motor running (%d -> %d mV)", idle_mv, s_run_mv);
            return true;
        }
    }
    if (s_sense_ok) {
        ESP_LOGW(TAG, "Motor does not start");
        return false;
    }
#endif
    ESP_LOGI(TAG, "Key %s", name);
    s_motor_start_us = esp_timer_get_time();
    output_pulse(gpio_on);
    return true;
#endif
}

#if CONFIG_SHUTTER_OUTPUT_MODE_PULSE
/** A key press that stops a move in direction @p dir. */
static void motor_stop_press(shutter_motion_t dir)
{
#if CONFIG_SHUTTER_STOP_DEDICATED_KEY
    (void)dir;
    output_pulse(SHUTTER_GPIO_STOP);
#elif CONFIG_SHUTTER_STOP_OPPOSITE_KEY
    output_pulse(dir == SHUTTER_MOTION_OPENING ? SHUTTER_GPIO_CLOSE : SHUTTER_GPIO_OPEN);
#elif CONFIG_SHUTTER_STOP_SAME_KEY
    output_pulse(dir == SHUTTER_MOTION_OPENING ? SHUTTER_GPIO_OPEN : SHUTTER_GPIO_CLOSE);
#else
    (void)dir; /* the drive cannot be stopped */
#endif
}
#endif

/**
 * @param at_end true if the drive has reached an end position. In PULSE mode
 *               the drive stops there by itself; another key press would
 *               start it again.
 */
static void motor_stop(shutter_motion_t dir, bool at_end)
{
#if CONFIG_SHUTTER_OUTPUT_MODE_HOLD
    (void)dir;
    output_set(SHUTTER_GPIO_OPEN, false);
    output_set(SHUTTER_GPIO_CLOSE, false);
#else
    if (!at_end) {
        motor_stop_press(dir);
        if (s_volt_ok) {
            /* If the voltage is still there, the press was not registered: another
             * press is safe then, the controller is demonstrably still running. */
            if (!wait_powered(false, REACT_TIMEOUT_MS)) {
                ESP_LOGW(TAG, "Motor still powered, stop again");
                motor_stop_press(dir);
                if (!wait_powered(false, REACT_TIMEOUT_MS)) {
                    ESP_LOGE(TAG, "Motor cannot be stopped");
                }
            }
        }
#if MOTOR_SENSE
        /* Without motor voltage, never press again: if the motor had already
         * stopped, another press on the opposite key would start it that way. */
        else if (s_sense_ok && !sense_wait_stop()) {
            ESP_LOGW(TAG, "No rise after the stop (motor already stopped?)");
        }
#endif
    }
#endif
    ESP_LOGI(TAG, "Stop%s", at_end ? " (end position)" : "");
}

static esp_err_t outputs_init(void)
{
    uint64_t mask = (1ULL << SHUTTER_GPIO_OPEN) | (1ULL << SHUTTER_GPIO_CLOSE);
#if CONFIG_SHUTTER_STOP_DEDICATED_KEY
    mask |= (1ULL << SHUTTER_GPIO_STOP);
#endif
    /* Set the level before switching to output so no key is "pressed" at start-up */
    for (int gpio = 0; gpio < GPIO_NUM_MAX; gpio++) {
        if (mask & (1ULL << gpio)) {
            gpio_set_level(gpio, LEVEL_OFF);
        }
    }
    gpio_config_t cfg = {
        .pin_bit_mask = mask,
        .mode         = OUTPUT_GPIO_MODE,
        /* No internal pull-up: the keypad electronics have their own */
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&cfg), TAG, "gpio_config failed");
    for (int gpio = 0; gpio < GPIO_NUM_MAX; gpio++) {
        if (mask & (1ULL << gpio)) {
            gpio_set_level(gpio, LEVEL_OFF);
        }
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------------- */
/* Persistence                                                               */
/* ------------------------------------------------------------------------- */

static void position_load(void)
{
    nvs_handle_t handle;
    uint32_t     value = 0;

    s_pos = 0;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
        if (nvs_get_u32(handle, NVS_KEY_POS, &value) == ESP_OK && value <= POS_MAX) {
            s_pos = (int32_t)value;
        }
        nvs_close(handle);
    }
    ESP_LOGI(TAG, "Stored position: %u %%", pos_to_pct(s_pos));
}

static void position_save(void)
{
    nvs_handle_t handle;

    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
        ESP_LOGW(TAG, "Could not open NVS");
        return;
    }
    if (nvs_set_u32(handle, NVS_KEY_POS, (uint32_t)s_pos) == ESP_OK) {
        nvs_commit(handle);
    }
    nvs_close(handle);
}

/* ------------------------------------------------------------------------- */
/* State machine                                                             */
/* ------------------------------------------------------------------------- */

static void report(bool force)
{
    uint8_t pct = pos_to_pct(s_pos);

    atomic_store(&s_percent_cache, pct);
    s_last_report_us = esp_timer_get_time();
    if (!force && pct == s_last_reported_pct && s_motion == s_last_reported_motion) {
        return;
    }
    s_last_reported_pct    = pct;
    s_last_reported_motion = s_motion;
    if (s_state_cb) {
        s_state_cb(pct, s_motion);
    }
}

/** Adds the distance travelled since the last call to the position. */
static void integrate_position(int64_t now_us)
{
    int64_t elapsed_us = now_us - s_last_update_us;
    s_last_update_us   = now_us;

    if (s_motion == SHUTTER_MOTION_STOPPED || elapsed_us <= 0) {
        return;
    }
    if (s_motion == SHUTTER_MOTION_CLOSING) {
        s_pos += (int32_t)(elapsed_us * POS_MAX / ((int64_t)CONFIG_SHUTTER_TRAVEL_TIME_DOWN_MS * 1000));
    } else {
        s_pos -= (int32_t)(elapsed_us * POS_MAX / ((int64_t)CONFIG_SHUTTER_TRAVEL_TIME_UP_MS * 1000));
    }
    if (s_pos < 0) {
        s_pos = 0;
    } else if (s_pos > POS_MAX) {
        s_pos = POS_MAX;
    }
}

/**
 * @param at_end  end position reached (no stop key press needed in PULSE mode)
 * @param outputs false if the stop was triggered by hand on the keypad
 */
static void do_stop(bool at_end, bool outputs)
{
    if (s_motion != SHUTTER_MOTION_STOPPED) {
        integrate_position(esp_timer_get_time());
        if (outputs) {
            motor_stop(s_motion, at_end);
        }
        s_motion = SHUTTER_MOTION_STOPPED;
        position_save();
    }
    s_phase = PHASE_IDLE;
    report(true);
}

static void do_start(bool outputs)
{
    shutter_motion_t dir;

    if (s_target == 0) {
        dir = SHUTTER_MOTION_OPENING;
    } else if (s_target == POS_MAX) {
        dir = SHUTTER_MOTION_CLOSING;
    } else if (abs(s_target - s_pos) < POS_PER_PCT) {
        s_phase = PHASE_IDLE; /* target already reached */
        report(true);
        return;
    } else {
        dir = (s_target < s_pos) ? SHUTTER_MOTION_OPENING : SHUTTER_MOTION_CLOSING;
    }
    if (outputs) {
        if (!motor_start(dir)) {
            if (s_start_at_end) {
                /* The controller switched on, but the motor was already at its end switch */
                s_pos = (dir == SHUTTER_MOTION_OPENING) ? 0 : POS_MAX;
                position_save();
            } else if (is_end_position(s_target)) {
                /* No motor current towards an end position: the shutter is probably already there */
                ESP_LOGI(TAG, "Shutter is probably already %s", s_target == 0 ? "up" : "down");
                s_pos = s_target;
                position_save();
            }
            s_phase = PHASE_IDLE;
            report(true);
            return;
        }
        s_last_update_us = s_motor_start_us;
    } else {
        s_last_update_us = esp_timer_get_time();
#if MOTOR_SENSE
        /* Manual operation: the motor is probably already running, current level = running level */
        if (s_sense_ok) {
            s_run_mv     = sense_block();
            s_rise_count = 0;
        }
#endif
    }
    s_motion = dir;
    s_phase  = PHASE_MOVING;
    report(true);
}

/**
 * The motor has stopped without the firmware stopping it.
 * @param at_end true: the motor's end switch cut off (motor current gone while moving
 *               towards an end position). false: the controller switched off, the
 *               shutter stays where it is.
 */
static void motor_stopped_detected(bool at_end)
{
    integrate_position(esp_timer_get_time());
    if (at_end) {
        ESP_LOGI(TAG, "Motor stopped: end position %s reached", s_target == 0 ? "up" : "down");
        s_pos = s_target;
    } else {
        ESP_LOGW(TAG, "Motor stopped before the target, at %u %%", pos_to_pct(s_pos));
    }
    s_motion = SHUTTER_MOTION_STOPPED;
    s_phase  = PHASE_IDLE;
    position_save();
    report(true);
}

static void handle_goto(int32_t target)
{
    if (target < 0) {
        target = 0;
    } else if (target > POS_MAX) {
        target = POS_MAX;
    }
#if CONFIG_SHUTTER_STOP_NONE
    /* Without a way to stop, only end positions can be reached */
    target = (target < POS_MAX / 2) ? 0 : POS_MAX;
#endif

    /* Ignore intermediate positions that are already (almost) reached.
     * End positions are always driven to, to resynchronise the position. */
    if (s_phase == PHASE_IDLE && !is_end_position(target) && abs(target - s_pos) < POS_PER_PCT) {
        report(true);
        return;
    }

    s_target = target;

    switch (s_phase) {
    case PHASE_IDLE:
        do_start(true);
        break;
    case PHASE_PAUSE:
        /* The new target is applied after the pause */
        break;
    case PHASE_MOVING:
    case PHASE_OVERRUN: {
        integrate_position(esp_timer_get_time());
        bool want_open  = (target == 0) || (target != POS_MAX && target < s_pos);
        bool want_close = (target == POS_MAX) || (target != 0 && target > s_pos);
        if ((s_motion == SHUTTER_MOTION_OPENING && want_open) || (s_motion == SHUTTER_MOTION_CLOSING && want_close)) {
            s_phase = PHASE_MOVING; /* same direction: only update the target */
        } else {
            /* Reversing: stop, wait briefly, then start again */
            motor_stop(s_motion, false);
            s_motion         = SHUTTER_MOTION_STOPPED;
            s_phase          = PHASE_PAUSE;
            s_phase_until_us = esp_timer_get_time() + (int64_t)CONFIG_SHUTTER_REVERSE_DELAY_MS * 1000;
            report(true);
        }
    } break;
    }
}

/**
 * Key pressed by hand, with motor voltage sensing: the controller's state is
 * directly visible. Powered = moving in the key's direction, otherwise stopped.
 */
static void handle_manual_powered(shutter_cmd_type_t type)
{
    /* The press was about 60 ms ago (debouncing); wait for the controller to react */
    bool    on    = motor_powered();
    int64_t until = esp_timer_get_time() + (int64_t)REACT_TIMEOUT_MS * 1000;
    while (esp_timer_get_time() < until) {
        vTaskDelay(pdMS_TO_TICKS(VOLT_POLL_MS * 2));
        bool now = motor_powered();
        if (now != on) {
            on = now;
            break;
        }
    }
    if (!on || type == CMD_MANUAL_STOP) {
        if (s_motion != SHUTTER_MOTION_STOPPED || s_phase == PHASE_PAUSE) {
            ESP_LOGI(TAG, "Manual: stop");
            do_stop(false, false);
        }
        return;
    }
    shutter_motion_t dir = (type == CMD_MANUAL_OPEN) ? SHUTTER_MOTION_OPENING : SHUTTER_MOTION_CLOSING;
    if (s_motion == dir) {
        return; /* already moving that way */
    }
    if (s_motion != SHUTTER_MOTION_STOPPED) {
        do_stop(false, false);
    }
    ESP_LOGI(TAG, "Manual: %s", dir == SHUTTER_MOTION_OPENING ? "UP" : "DOWN");
    s_target = (dir == SHUTTER_MOTION_OPENING) ? 0 : POS_MAX;
    do_start(false);
}

/** Key pressed by hand: only track the model, output nothing. */
static void handle_manual(shutter_cmd_type_t type)
{
    if (s_volt_ok) {
        handle_manual_powered(type);
        return;
    }
    if (s_motion != SHUTTER_MOTION_STOPPED || s_phase == PHASE_PAUSE) {
        ESP_LOGI(TAG, "Manual: stop");
        do_stop(false, false);
        return;
    }
    if (type == CMD_MANUAL_STOP) {
        return;
    }
    ESP_LOGI(TAG, "Manual: %s", type == CMD_MANUAL_OPEN ? "UP" : "DOWN");
    s_target = (type == CMD_MANUAL_OPEN) ? 0 : POS_MAX;
    do_start(false);
}

static void handle_tick(void)
{
    int64_t now = esp_timer_get_time();

    switch (s_phase) {
    case PHASE_IDLE:
        break;
    case PHASE_PAUSE:
        if (now >= s_phase_until_us) {
            do_start(true);
        }
        break;
    case PHASE_MOVING: {
        if (s_volt_ok && !motor_powered()) {
            ESP_LOGI(TAG, "Controller switched the motor off");
            motor_stopped_detected(false);
            break;
        }
#if MOTOR_SENSE
        if (s_sense_ok && sense_motor_stopped()) {
            motor_stopped_detected(is_end_position(s_target));
            break;
        }
        now = esp_timer_get_time();
#endif
        integrate_position(now);
        bool reached = (s_motion == SHUTTER_MOTION_OPENING) ? (s_pos <= s_target) : (s_pos >= s_target);
        if (reached) {
            if (is_end_position(s_target)) {
                int64_t overrun_ms = CONFIG_SHUTTER_END_OVERRUN_MS;
#if MOTOR_SENSE
                if (s_sense_ok) {
                    overrun_ms += SENSE_END_GRACE_MS; /* wait for the motor to switch off */
                }
#endif
                s_pos            = s_target;
                s_phase          = PHASE_OVERRUN;
                s_phase_until_us = now + overrun_ms * 1000;
                report(false);
            } else {
                do_stop(false, true);
            }
        } else if (now - s_last_report_us >= (int64_t)CONFIG_SHUTTER_REPORT_INTERVAL_MS * 1000) {
            report(false);
        }
    } break;
    case PHASE_OVERRUN:
        if (s_volt_ok && !motor_powered()) {
            motor_stopped_detected(true); /* the end position was already reached by calculation */
            break;
        }
#if MOTOR_SENSE
        if (s_sense_ok && sense_motor_stopped()) {
            motor_stopped_detected(is_end_position(s_target));
            break;
        }
        now = esp_timer_get_time();
#endif
        s_last_update_us = now;
        if (now >= s_phase_until_us) {
#if MOTOR_SENSE
            if (s_sense_ok) {
                ESP_LOGW(TAG, "No motor switch-off detected, assuming end position");
            }
#endif
            do_stop(true, true);
        }
        break;
    }
}

/** Measure and report battery and solar panel at rest (under load the battery value would be too low). */
static void measure_power(void)
{
    int battery_mv = -1, solar_mv = -1;

    if (!s_power_cb) {
        return;
    }
    if (sense_available(SENSE_BATTERY) && sense_read_mv(SENSE_BATTERY, 16, &battery_mv) == ESP_OK &&
        battery_mv < CONFIG_SHUTTER_BATTERY_MIN_MV) {
        battery_mv = -1; /* divider not connected */
    }
    if (sense_available(SENSE_SOLAR) && sense_read_mv(SENSE_SOLAR, 16, &solar_mv) != ESP_OK) {
        solar_mv = -1;
    }
#if CONFIG_PM_ENABLE
    sense_release();
#endif
    ESP_LOGI(TAG, "Battery %d mV, solar %d mV", battery_mv, solar_mv);
    s_power_cb(battery_mv, solar_mv);
}

static void shutter_task(void *arg)
{
    shutter_cmd_t cmd;

    report(true);
    measure_power();
    for (;;) {
        TickType_t wait = (s_phase == PHASE_IDLE) ? pdMS_TO_TICKS(CONFIG_SHUTTER_MONITOR_INTERVAL_S * 1000ULL)
                                                  : pdMS_TO_TICKS(TICK_MS);
        if (xQueueReceive(s_queue, &cmd, wait) != pdTRUE) {
            if (s_phase == PHASE_IDLE) {
                measure_power();
                continue;
            }
        } else {
            pm_keep_awake(true);
            switch (cmd.type) {
            case CMD_GOTO:
                ESP_LOGI(TAG, "Moving to %u %%", pos_to_pct(cmd.target));
                handle_goto(cmd.target);
                break;
            case CMD_STOP:
#if CONFIG_SHUTTER_STOP_NONE
                ESP_LOGW(TAG, "Stop not possible (the drive runs to the end position)");
                report(true);
#else
                do_stop(false, true);
#endif
                break;
            case CMD_MANUAL_OPEN:
            case CMD_MANUAL_CLOSE:
            case CMD_MANUAL_STOP:
                handle_manual(cmd.type);
                break;
            }
        }
        handle_tick();
        pm_keep_awake(s_phase != PHASE_IDLE);
    }
}

/* ------------------------------------------------------------------------- */
/* Public API                                                                */
/* ------------------------------------------------------------------------- */

static void send_cmd(shutter_cmd_type_t type, int32_t target)
{
    shutter_cmd_t cmd = {.type = type, .target = target};

    if (!s_queue || xQueueSend(s_queue, &cmd, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Command %d dropped", type);
    }
}

void shutter_open(void)
{
    send_cmd(CMD_GOTO, 0);
}

void shutter_close(void)
{
    send_cmd(CMD_GOTO, POS_MAX);
}

void shutter_stop(void)
{
    send_cmd(CMD_STOP, 0);
}

void shutter_set_power_cb(shutter_power_cb_t cb)
{
    s_power_cb = cb;
}

void shutter_go_to_percent(uint8_t lift_percent)
{
    if (lift_percent > 100) {
        lift_percent = 100;
    }
    send_cmd(CMD_GOTO, (int32_t)lift_percent * POS_PER_PCT);
}

void shutter_manual_press(shutter_key_t key)
{
    static const shutter_cmd_type_t map[] = {
        [SHUTTER_KEY_OPEN]  = CMD_MANUAL_OPEN,
        [SHUTTER_KEY_CLOSE] = CMD_MANUAL_CLOSE,
        [SHUTTER_KEY_STOP]  = CMD_MANUAL_STOP,
    };
    send_cmd(map[key], 0);
}

bool shutter_output_is_driving(int gpio)
{
    return gpio >= 0 && (atomic_load(&s_driving_mask) & (1ULL << gpio));
}

uint8_t shutter_get_percent(void)
{
    return (uint8_t)atomic_load(&s_percent_cache);
}

esp_err_t shutter_init(shutter_state_cb_t cb)
{
    ESP_RETURN_ON_FALSE(GPIO_IS_VALID_OUTPUT_GPIO(CONFIG_SHUTTER_GPIO_UP), ESP_ERR_INVALID_ARG, TAG, "invalid UP gpio");
    ESP_RETURN_ON_FALSE(GPIO_IS_VALID_OUTPUT_GPIO(CONFIG_SHUTTER_GPIO_DOWN), ESP_ERR_INVALID_ARG, TAG,
                        "invalid DOWN gpio");
    ESP_RETURN_ON_FALSE(CONFIG_SHUTTER_GPIO_UP != CONFIG_SHUTTER_GPIO_DOWN, ESP_ERR_INVALID_ARG, TAG,
                        "UP and DOWN must use different gpios");

    s_state_cb = cb;
#if CONFIG_PM_ENABLE
    ESP_RETURN_ON_ERROR(esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "shutter", &s_pm_lock), TAG,
                        "pm lock create failed");
#endif
    ESP_RETURN_ON_ERROR(outputs_init(), TAG, "outputs init failed");
    if (sense_init() != ESP_OK) {
        ESP_LOGW(TAG, "No voltage sensing, time-based only");
    }
    volt_init();
#if MOTOR_SENSE
    if (sense_available(SENSE_BATTERY)) {
        ESP_LOGI(TAG, "Motor current detection: battery %d mV, threshold %d mV", sense_idle(), CONFIG_SHUTTER_MOTOR_STEP_MV);
    }
#endif
    ESP_LOGI(TAG, "Motor voltage %s", s_volt_ok ? "sensed" : "not connected");
    position_load();
    s_target = s_pos;
    atomic_store(&s_percent_cache, pos_to_pct(s_pos));

    s_queue = xQueueCreate(8, sizeof(shutter_cmd_t));
    ESP_RETURN_ON_FALSE(s_queue, ESP_ERR_NO_MEM, TAG, "queue alloc failed");
    ESP_RETURN_ON_FALSE(xTaskCreate(shutter_task, "shutter", 3072, NULL, 6, NULL) == pdPASS, ESP_ERR_NO_MEM, TAG,
                        "task create failed");
    ESP_LOGI(TAG, "Initialised (%s, UP=GPIO%d, DOWN=GPIO%d, STOP=GPIO%d, travel time up/down %d/%d ms)",
             OUTPUT_MODE_STR, SHUTTER_GPIO_OPEN, SHUTTER_GPIO_CLOSE, SHUTTER_GPIO_STOP,
             CONFIG_SHUTTER_TRAVEL_TIME_UP_MS, CONFIG_SHUTTER_TRAVEL_TIME_DOWN_MS);
    return ESP_OK;
}
