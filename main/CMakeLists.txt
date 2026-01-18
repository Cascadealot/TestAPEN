# TestAPEN Main Component CMakeLists.txt
# Fork of TestAP2 using ESP-NOW instead of CAN bus

idf_component_register(
    SRCS
        "main.c"
        "master_node.c"
        "rudder_node.c"
        "ui_node.c"
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
)
