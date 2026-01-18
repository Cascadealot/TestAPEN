# Command Console Component

idf_component_register(
    SRCS
        "cmd_console.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        freertos
        state_machine
        param_store
        gnss_driver
        can_protocol
)
