# SPDX-FileCopyrightText: 2026 Dirk Wahrheit
# SPDX-License-Identifier: Apache-2.0
#
# zboss_ncp is the coordinator. espos_ota is listed for the build, not for
# main.cpp: espOS consumers build with IDF's MINIMAL_BUILD, so a component
# nothing requires is not compiled, and espos_start() starts only what is in
# the build. Without espos_ota the coordinator could never be updated in the
# field.
idf_component_register(
    SRCS main.cpp
    PRIV_REQUIRES espos_core espos_net espos_ota zboss_ncp esp_coex)
