# BLE Manager component
# Only compile for Master node which has BLE support
#
# Note: Uses conditional SRCS pattern to avoid cmake two-pass issue where
# CONFIG_* variables are empty on first pass, causing REQUIRES to not be set.
# Fix: Always register with full dependencies, conditionally include sources.

if(CONFIG_TESTAPEN_NODE_MASTER AND CONFIG_BT_ENABLED)
    set(BLE_SRCS "ble_manager.c")
else()
    set(BLE_SRCS "")
endif()

# Always register with full dependencies
idf_component_register(
    SRCS ${BLE_SRCS}
    INCLUDE_DIRS "include"
    REQUIRES bt nvs_flash log param_store state_machine icm20948
)
