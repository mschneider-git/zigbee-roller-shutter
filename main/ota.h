/*
 * Zigbee OTA: client of the OTA Upgrade cluster (see ota.c).
 */
#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "ezbee/zha.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Adds the OTA Upgrade client cluster to the endpoint (before ezb_af_device_desc_register). */
esp_err_t ota_add_client_cluster(ezb_af_ep_desc_t ep_desc, uint8_t ep_id);

/** Call after ezb_af_device_desc_register. */
void ota_after_register(void);

/**
 * Forward from the ZCL core action handler.
 * @return true if the message belonged to OTA
 */
bool ota_handle_action(ezb_zcl_core_action_callback_id_t callback_id, void *message);

/**
 * The device is in the network (again): mark the new image as valid
 * (cancel the rollback) and ask for updates. Call in the Zigbee context.
 */
void ota_network_ready(void);

#ifdef __cplusplus
}
#endif
