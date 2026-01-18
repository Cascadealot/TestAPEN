# AS5600 Magnetic Rotary Encoder Driver Component

idf_component_register(
    SRCS
        "as5600.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        driver
        freertos
)
