# State Machine Component

idf_component_register(
    SRCS
        "state_machine.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        autopilot_common
    PRIV_REQUIRES
        esp_timer
)
