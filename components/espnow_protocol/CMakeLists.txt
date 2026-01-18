idf_component_register(
    SRCS "espnow_protocol.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi nvs_flash log
)
