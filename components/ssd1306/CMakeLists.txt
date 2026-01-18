# SSD1306 OLED Display Driver Component

idf_component_register(
    SRCS
        "ssd1306.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        driver
        freertos
)
