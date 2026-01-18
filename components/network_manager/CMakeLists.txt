# Network Manager Component

idf_component_register(
    SRCS
        "network_manager.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        esp_wifi
        esp_event
        esp_netif
        nvs_flash
        esp_http_server
        esp_app_format
        app_update
        lwip
)
