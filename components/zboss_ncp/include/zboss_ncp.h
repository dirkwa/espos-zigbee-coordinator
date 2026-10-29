/*
 * SPDX-FileCopyrightText: 2026 Dirk Wahrheit
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Start the Zigbee coordinator and serve its ZBOSS NCP protocol to one host
 * (Zigbee2MQTT's zboss adapter) on TCP port CONFIG_ZBOSS_NCP_TCP_PORT.
 * Call once, after espos_start(): the NCP keeps its data in the "zb_nvs",
 * "zb_storage" and "zb_fct" partitions and leaves espOS's "nvs" alone.
 * Returns once the stack and the server tasks run.
 */
esp_err_t zboss_ncp_start(void);

#ifdef __cplusplus
}
#endif
