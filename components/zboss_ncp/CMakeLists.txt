# The vendored NCP sources (UPSTREAM.md) plus ncp_boot.cpp, upstream's
# main.cpp turned into zboss_ncp_start().
idf_component_register(
    SRCS app.cpp ncp_boot.cpp protocol.cpp transport.cpp utils.cpp zb_ncp.cpp
    INCLUDE_DIRS include
    PRIV_INCLUDE_DIRS .
    PRIV_REQUIRES esp_app_format esp_partition esp_timer lwip nvs_flash)

# libzboss_stack's Green Power cluster calls back into libesp_zb_api, which the
# linker has already passed by then under IDF 6's library order, so the
# handler is pulled in up front.
target_link_libraries(${COMPONENT_LIB} INTERFACE
    "-Wl,--undefined=zb_zcl_green_power_cluster_handler")
