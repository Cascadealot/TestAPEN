# TestAPEN Main Component CMakeLists.txt
# Fork of TestAPEN using ESP-NOW instead of CAN bus

# Check if e-Paper test mode is enabled
if(CONFIG_TESTAPEN_EPAPER_TEST)
    # Minimal test mode - only e-Paper partial refresh test
    set(MAIN_SRCS "epaper_test.c")
else()
    # Normal mode - full node implementation
    set(MAIN_SRCS
        "main.c"
        "master_node.c"
        "rudder_node.c"
        "ui_node.c"
    )
endif()

idf_component_register(
    SRCS
        ${MAIN_SRCS}
    INCLUDE_DIRS
        "."
    REQUIRES
        autopilot_common
        espnow_protocol
        state_machine
        icm20948
        ssd1306
        as5600
        epaper
        network_manager
        cmd_console
        param_store
        ble_manager
        gnss_driver
        driver
        nvs_flash
        esp_wifi
        esp_event
        bt
        console
)
