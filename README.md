# espos-zigbee-coordinator

Puts a Zigbee coordinator for Zigbee2MQTT on the boat's Wi-Fi, so the Zigbee
radio can sit wherever the sensors are instead of in a USB port on the Signal
K server.

The coordinator is a small ESP32-C5 or ESP32-C6 board, the two chips with a
Zigbee radio next to their Wi-Fi. It does not decode anything: Zigbee2MQTT
connects to it over the network and runs the Zigbee network through it,
exactly as it would through a USB stick. Pairing, devices and their values
stay in Zigbee2MQTT and the Signal K plugin that runs it.

**Status:** runs on a Waveshare ESP32-C5 kit: Zigbee2MQTT connects over
Wi-Fi and forms the network. Pairing devices through it is still to be
tried; the ESP32-C6 build has not met a board yet.

## Hardware

| Board | Flash | Status |
|---|---|---|
| Waveshare ESP32-C5-WIFI6-KIT (N16R8) | 16 MB | runs |
| ESP32-C6 dev kits (ESP32-C6-DevKitC-1 and similar) | 8 MB or 16 MB | builds only |

Boards with 4 MB of flash are too small: the firmware does not fit their app
slots. An ESP32-C5 also needs PSRAM, which every C5 module seen so far
carries: without it the chip's internal RAM is too small for Wi-Fi and the
Zigbee stack together, and the coordinator does not start.

Wi-Fi and Zigbee share one 2.4 GHz radio on these chips, and the chip
switches between them. For a handful of sensors that is fine. A large network
or a busy Wi-Fi is better served by a USB stick on the server.

## Install

Flash it from the
[hosted flasher](https://signalk-espos.github.io/signalk-espos-manager/flash/)
in Chrome or Edge over USB, or from the Flash page of the
signalk-espos-manager plugin. For manual flashing, write the `-merged.bin` at
offset `0x0`.

## First setup

1. Join the `espOS-xxxx` Wi-Fi network the board opens and enter your boat's
   Wi-Fi.
2. Point Zigbee2MQTT at it:

   ```yaml
   serial:
     adapter: zboss
     port: tcp://<device>:6638
   ```

   `port: mdns://espos-zigbee` finds it by itself when there is only one on
   the network.

Zigbee2MQTT's `zboss` adapter is still marked experimental. Over a network
link it needs longer timeouts than the official image sets, and backup and
restore of the network need the
[`ghcr.io/tostmann/zigbee2mqtt-esp32`](https://github.com/tostmann/esp-coordinator/blob/main/ZIGBEE2MQTT.md)
image, which carries both.

One Zigbee2MQTT talks to one coordinator. A second board is a second Zigbee
network, with its own Zigbee2MQTT.

## Updates

Over the air through signalk-espos-manager. The Zigbee network (its key, PAN
and paired devices) lives in its own partitions and survives updates. A board
that ran another project's firmware needs one USB flash first.

## When it misbehaves

The device log is at `http://<device>/api/v1/logs`. Lines tagged `TRNPT` show
Zigbee2MQTT connecting and disconnecting, `NCP` and `APP` the coordinator
itself. Report issues at
https://github.com/dirkwa/espos-zigbee-coordinator/issues.

## Building

```bash
git clone --recurse-submodules https://github.com/dirkwa/espos-zigbee-coordinator
. <ESP-IDF v6.0.3>/export.sh       # the version in .idf-version
scripts/build.sh -B build-esp32c5 -DSDKCONFIG=build-esp32c5/sdkconfig -DIDF_TARGET=esp32c5 build
```

The coordinator protocol (ZBOSS NCP) comes from
[busware's esp-coordinator](https://github.com/tostmann/esp-coordinator),
MIT-licensed; `components/zboss_ncp/UPSTREAM.md` says what changed for espOS.
The rest is Apache-2.0.
