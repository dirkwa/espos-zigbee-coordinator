/*
 * SPDX-FileCopyrightText: 2026 Dirk Wahrheit
 * SPDX-License-Identifier: Apache-2.0
 *
 * espos-zigbee-coordinator: the chip's 802.15.4 radio as a Zigbee
 * coordinator for Zigbee2MQTT, reached over the boat's Wi-Fi instead of a
 * USB stick on the server. The device decodes nothing: it speaks the ZBOSS
 * NCP protocol, the one Zigbee2MQTT's "zboss" adapter drives, and Zigbee2MQTT
 * (with the Signal K plugin that runs it) owns the network, the devices and
 * what their values mean.
 *
 * Zigbee2MQTT finds it with `serial.port: mdns://espos-zigbee`, or takes
 * `tcp://<device>:6638` directly, with `serial.adapter: zboss`.
 */
#include <cstdio>

#include "esp_err.h"
#include "esp_log.h"
#include "espos.h"
#include "espos_mdns.h"
#include "espos_net.h"
#include "sdkconfig.h"
#include "zboss_ncp.h"

namespace {

const char* TAG = "zigbee_coordinator";

}  // namespace

extern "C" void app_main(void) {
  ESP_ERROR_CHECK(espos_start(nullptr));

  esp_err_t err = zboss_ncp_start();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "coordinator did not start: %s", esp_err_to_name(err));
    return;
  }

  /* herdsman's mDNS discovery takes the first service of the type it is
   * given and reads the adapter from radio_type; the Zigbee2MQTT frontend
   * lists every service that carries it. id is the espOS device id that
   * _espos._tcp carries too: the signalk-zigbee plugin keys coordinators by
   * it and matches a configured one by it, so a renamed device or a new DHCP
   * lease is still the same coordinator. espos_mdns copies the strings. */
  const char* id = espos_net_short_id();
  char id_kv[24];
  snprintf(id_kv, sizeof(id_kv), "id=%s", id);
  const char* const txt[] = {"radio_type=zboss", id_kv};
  err = espos_mdns_add_service("_espos-zigbee", "_tcp", CONFIG_ZBOSS_NCP_TCP_PORT, txt,
                               id[0] != '\0' ? 2 : 1);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "not advertised over mDNS (%s): use tcp://<device>:%d", esp_err_to_name(err),
             CONFIG_ZBOSS_NCP_TCP_PORT);
  }
}
