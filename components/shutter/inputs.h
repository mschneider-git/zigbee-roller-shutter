/*
 * Inputs: reset button (BOOT) and, optionally, detection of manual operation
 * on the membrane keypad.
 */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Called when the reset button has been held long enough. */
typedef void (*inputs_reset_cb_t)(void);

esp_err_t inputs_init(inputs_reset_cb_t reset_cb);

#ifdef __cplusplus
}
#endif
