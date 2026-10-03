/*
 * Roller shutter drive with time-based position tracking.
 *
 * Positions follow the Zigbee Window Covering convention:
 *   0 %   = fully open (shutter up)
 *   100 % = fully closed (shutter down)
 * Home Assistant (ZHA) converts this to its own representation automatically.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Key lines that "open" and "close" (takes the direction swap into account) */
#if CONFIG_SHUTTER_SWAP_DIRECTION
#define SHUTTER_GPIO_OPEN  CONFIG_SHUTTER_GPIO_DOWN
#define SHUTTER_GPIO_CLOSE CONFIG_SHUTTER_GPIO_UP
#else
#define SHUTTER_GPIO_OPEN  CONFIG_SHUTTER_GPIO_UP
#define SHUTTER_GPIO_CLOSE CONFIG_SHUTTER_GPIO_DOWN
#endif

#if CONFIG_SHUTTER_STOP_DEDICATED_KEY
#define SHUTTER_GPIO_STOP CONFIG_SHUTTER_GPIO_STOP
#else
#define SHUTTER_GPIO_STOP (-1)
#endif

typedef enum {
    SHUTTER_MOTION_STOPPED = 0,
    SHUTTER_MOTION_OPENING,
    SHUTTER_MOTION_CLOSING,
} shutter_motion_t;

/**
 * Called from the shutter task when the position or motion changes.
 * @param lift_percent current position, 0 = open, 100 = closed
 */
typedef void (*shutter_state_cb_t)(uint8_t lift_percent, shutter_motion_t motion);

esp_err_t shutter_init(shutter_state_cb_t cb);

/**
 * Called from the shutter task after start-up and then at rest every
 * CONFIG_SHUTTER_MONITOR_INTERVAL_S seconds.
 * @param battery_mv battery voltage, -1 = not measured
 * @param solar_mv   solar panel voltage, -1 = not measured
 */
typedef void (*shutter_power_cb_t)(int battery_mv, int solar_mv);

/** Call before shutter_init(). */
void shutter_set_power_cb(shutter_power_cb_t cb);

void shutter_open(void);
void shutter_close(void);
void shutter_stop(void);
void shutter_go_to_percent(uint8_t lift_percent);

typedef enum {
    SHUTTER_KEY_OPEN,
    SHUTTER_KEY_CLOSE,
    SHUTTER_KEY_STOP,
} shutter_key_t;

/**
 * Reports a key pressed by hand on the membrane keypad. Nothing is output,
 * only the calculated position/motion is tracked.
 */
void shutter_manual_press(shutter_key_t key);

/** true while the firmware itself drives the output (for the key detection). */
bool shutter_output_is_driving(int gpio);

/** Last known position (0..100), thread-safe. */
uint8_t shutter_get_percent(void);

#ifdef __cplusplus
}
#endif
