# SPDX-FileCopyrightText: 2026 Dirk Wahrheit
# SPDX-License-Identifier: Apache-2.0
#
# espos-zigbee-coordinator: a pure ESP-IDF 6 project. WiFi, config, the web
# UI, mDNS and OTA come from the espOS submodule (espos/); the Zigbee
# coordinator and its NCP protocol are components/zboss_ncp, served to
# Zigbee2MQTT over TCP.
#
# Build:  . <IDF v6.0.3>/export.sh
#         scripts/build.sh -B build-esp32c5 -DSDKCONFIG=build-esp32c5/sdkconfig -DIDF_TARGET=esp32c5 build
cmake_minimum_required(VERSION 3.22)

# espOS's tables plus the coordinator's own partitions (partitions/*.csv).
# The image is 1.73 MB, just past the 1728 KB slots of a 4 MB table, so the
# smallest board is 8 MB; the C5 kit carries 16 MB. A device keeps the table
# it was first flashed with (OTA replaces app slots, never the table), so a
# change here reaches only boards flashed over USB afterwards.
include("${CMAKE_CURRENT_LIST_DIR}/espos/cmake/espos_project.cmake")
set(_partitions "partitions/8mb.csv")
if(IDF_TARGET STREQUAL "esp32c5")
    set(_partitions "partitions/16mb.csv")
endif()
espos_project_prologue(NAME "zigbee-coordinator" PARTITIONS "${_partitions}")

project(zigbee_coordinator)

# The committed espOS web UI (espos/ui/dist-gz) -> LittleFS "storage" image,
# flashed with `idf.py flash`. No Node step anywhere.
espos_project_ui_partition(NAME "zigbee-coordinator")
