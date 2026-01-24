# BLE Manager component
# Only compile for Master node which has BLE support

if(CONFIG_TESTAPEN_NODE_MASTER AND CONFIG_BT_ENABLED)
    idf_component_register(
        SRCS
            "ble_manager.c"
        INCLUDE_DIRS
            "include"
        REQUIRES
            bt
            nvs_flash
            log
            param_store
            state_machine
            icm20948
    )
else()
    # Register empty component for other node types
    idf_component_register(
        INCLUDE_DIRS
            "include"
    )
endif()
