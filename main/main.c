/**
 * @file main.c
 * @brief TestAP2 Autopilot Main Entry Point
 *
 * This file selects between Master and Rudder node based on Kconfig.
 *
 * FSD Reference: TestAP2.FSD.v1.0.0.md
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "autopilot_common.h"
#include "can_protocol.h"
#include "state_machine.h"

static const char *TAG = "MAIN";

// External node initialization functions
extern void master_node_init(void);
extern void rudder_node_init(void);
extern void ui_node_init(void);

void app_main(void)
{
    ESP_LOGI(TAG, "=================================================");
    ESP_LOGI(TAG, "TestAP2 Autopilot");
    ESP_LOGI(TAG, "FSD Version: %s", TESTAP2_FSD_VERSION);
    ESP_LOGI(TAG, "FW Version:  %s", TESTAP2_FW_VERSION);
    ESP_LOGI(TAG, "=================================================");

    // Initialize NVS (required for WiFi, calibration storage)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition needs erase");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized");

    // Initialize CAN bus (common to both nodes)
    ret = can_init(CONFIG_TESTAP2_CAN_TX_GPIO, CONFIG_TESTAP2_CAN_RX_GPIO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CAN initialization failed!");
        return;
    }

    // Select node type based on Kconfig
#if CONFIG_TESTAP2_NODE_MASTER
    ESP_LOGI(TAG, "Node Type: MASTER");
    master_node_init();
#elif CONFIG_TESTAP2_NODE_RUDDER
    ESP_LOGI(TAG, "Node Type: RUDDER");
    rudder_node_init();
#elif CONFIG_TESTAP2_NODE_UI
    ESP_LOGI(TAG, "Node Type: UI");
    ui_node_init();
#else
    #error "No node type selected in Kconfig!"
#endif

    ESP_LOGI(TAG, "Initialization complete");
}
