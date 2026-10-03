/*
 * Voltage sensing via dedicated dividers on ADC pins: battery, both motor terminals, solar panel.
 *
 * The controller keeps the motor powered for about 30 s after the end position,
 * but the motor switches itself off there. The voltage on the motor terminals
 * therefore shows whether the controller switched the motor on; whether it
 * really runs is only visible from the sag of the battery voltage.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SENSE_BATTERY, /* battery+ (after the reverse polarity protection) */
    SENSE_MOTOR_P, /* motor terminal M+ */
    SENSE_MOTOR_N, /* motor terminal M- */
    SENSE_SOLAR,   /* solar panel */
    SENSE_COUNT,
} sense_ch_t;

/** Sets up all configured channels. May be called more than once. */
esp_err_t sense_init(void);

/** true if the channel is configured (GPIO >= 0) and set up. */
bool sense_available(sense_ch_t ch);

/**
 * Averages @p samples calibrated ADC readings and converts them via the
 * divider to the voltage at the divider input. Thread-safe.
 */
esp_err_t sense_read_mv(sense_ch_t ch, uint8_t samples, int *out_mv);

/**
 * Releases the ADC unit. Call before light sleep: the SAR ADC has no sleep
 * retention, so with the peripherals powered down its registers would be lost
 * after wake-up. The next reading sets it up again.
 */
void sense_release(void);

#ifdef __cplusplus
}
#endif
