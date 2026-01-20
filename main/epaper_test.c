/**
 * @file epaper_test.c
 * @brief Interactive e-Paper Display Test with Console Commands
 *
 * Console commands for debugging e-paper display:
 *   ep clear    - Clear display to white
 *   ep black    - Fill display black
 *   ep pixel X Y - Draw single pixel at (X,Y)
 *   ep hline Y   - Draw horizontal line at Y
 *   ep vline X   - Draw vertical line at X
 *   ep rect      - Draw hollow rectangle 10px from edges
 *   ep text X Y MSG - Draw text at (X,Y)
 *   ep update    - Refresh display
 *   ep test      - Run test pattern
 *
 * Build: Enable CONFIG_TESTAP2_EPAPER_TEST in menuconfig
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_console.h"
#include "nvs_flash.h"

#include "network_manager.h"
#include "epaper.h"

static const char *TAG = "EPAPER_TEST";

// Test state
static float g_heading = 0.0f;
static int g_update_count = 0;
static bool g_use_partial = false;  // DISABLED - use full refresh only until stable

// Heading display area (must be 8-pixel aligned for partial refresh)
// Using large font size 2 (16x16 pixels per char)
// "000.0" = 5 chars * 16 = 80 pixels wide, 16 pixels tall
// Centered horizontally: (296 - 80) / 2 = 108, round to multiple of 8 = 104
// Centered vertically: (128 - 16) / 2 = 56
#define HEADING_X       104
#define HEADING_Y       56
#define HEADING_W       88   // 80 + 8 padding, multiple of 8
#define HEADING_H       24   // 16 + 8 padding, multiple of 8

/**
 * @brief Clear a rectangular region to white
 */
static void clear_region(int x, int y, int w, int h) {
    for (int py = y; py < y + h && py < EPAPER_HEIGHT; py++) {
        for (int px = x; px < x + w && px < EPAPER_WIDTH; px++) {
            epaper_set_pixel(px, py, false);  // false = white
        }
    }
}

/**
 * @brief Draw the heading value
 */
static void draw_heading(float heading) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%05.1f", heading);

    // Clear the heading area first
    clear_region(HEADING_X, HEADING_Y, HEADING_W, HEADING_H);

    // Draw heading with large font (size 2 = 16x16)
    epaper_draw_string(HEADING_X + 4, HEADING_Y + 4, buf, 2);
}

/**
 * @brief Draw static labels (only on full refresh)
 */
static void draw_static_content(void) {
    // Title at top
    epaper_draw_string(8, 4, "EPAPER PARTIAL TEST", 1);

    // Label above heading
    epaper_draw_string(HEADING_X + 16, HEADING_Y - 16, "HDG", 1);

    // Instructions at bottom
    epaper_draw_string(8, 108, "Partial refresh every 1s", 1);
    epaper_draw_string(8, 118, "Full refresh every 10 updates", 1);
}

/**
 * @brief Full display update (with flash)
 */
static void do_full_update(void) {
    ESP_LOGI(TAG, "=== FULL UPDATE ===");

    epaper_clear();
    draw_static_content();
    draw_heading(g_heading);

    uint32_t start = xTaskGetTickCount();
    epaper_update();
    uint32_t elapsed = (xTaskGetTickCount() - start) * portTICK_PERIOD_MS;

    ESP_LOGI(TAG, "Full update took %lu ms", elapsed);

    // Reset partial counter
    epaper_reset_partial();
}

/**
 * @brief Update display (using full refresh until partial is debugged)
 */
static void do_partial_update(void) {
    ESP_LOGI(TAG, "--- Update #%d, heading=%.1f (full refresh mode) ---", g_update_count, g_heading);

    // Update heading in framebuffer
    draw_heading(g_heading);

    // Use full refresh for now - partial refresh needs more debugging
    uint32_t start = xTaskGetTickCount();
    epaper_update();
    uint32_t elapsed = (xTaskGetTickCount() - start) * portTICK_PERIOD_MS;

    ESP_LOGI(TAG, "Full update took %lu ms", elapsed);
}

/**
 * @brief Console status task
 */
static void console_task(void *pvParameters) {
    ESP_LOGI(TAG, "Console ready - connect via telnet port 2323");
    ESP_LOGI(TAG, "OTA available at http://<ip>/update");

    // Just keep the task alive, network_manager handles console
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));

        // Print periodic status
        ESP_LOGI(TAG, "Status: heading=%.1f, updates=%d, partial=%s",
                 g_heading, g_update_count, g_use_partial ? "ON" : "OFF");
    }
}

/**
 * @brief Console command handler for e-paper testing
 */
static int ep_cmd_handler(int argc, char **argv) {
    if (argc < 2) {
        printf("Usage: ep <command> [args]\n");
        printf("Commands:\n");
        printf("  clear     - Clear display to white\n");
        printf("  black     - Fill display black\n");
        printf("  pixel X Y - Draw pixel at (X,Y)\n");
        printf("  hline Y   - Draw horizontal line at Y\n");
        printf("  vline X   - Draw vertical line at X\n");
        printf("  rect      - Draw hollow rectangle 10px from edges\n");
        printf("  fill X Y W H - Fill rectangle\n");
        printf("  text X Y MSG - Draw text\n");
        printf("  update    - Refresh display\n");
        printf("  test      - Run test pattern\n");
        printf("  info      - Show display info\n");
        return 0;
    }

    const char *cmd = argv[1];

    if (strcmp(cmd, "clear") == 0) {
        epaper_clear();
        printf("Display cleared to white\n");
    }
    else if (strcmp(cmd, "black") == 0) {
        epaper_clear_black();
        printf("Display filled black\n");
    }
    else if (strcmp(cmd, "pixel") == 0 && argc >= 4) {
        int x = atoi(argv[2]);
        int y = atoi(argv[3]);
        epaper_set_pixel(x, y, true);
        printf("Pixel set at (%d, %d)\n", x, y);
    }
    else if (strcmp(cmd, "hline") == 0 && argc >= 3) {
        int y = atoi(argv[2]);
        for (int x = 0; x < EPAPER_WIDTH; x++) {
            epaper_set_pixel(x, y, true);
        }
        printf("Horizontal line at y=%d\n", y);
    }
    else if (strcmp(cmd, "vline") == 0 && argc >= 3) {
        int x = atoi(argv[2]);
        for (int y = 0; y < EPAPER_HEIGHT; y++) {
            epaper_set_pixel(x, y, true);
        }
        printf("Vertical line at x=%d\n", x);
    }
    else if (strcmp(cmd, "rect") == 0) {
        // Hollow rectangle 10px from edges
        for (int x = 10; x < EPAPER_WIDTH - 10; x++) {
            epaper_set_pixel(x, 10, true);
            epaper_set_pixel(x, EPAPER_HEIGHT - 11, true);
        }
        for (int y = 10; y < EPAPER_HEIGHT - 10; y++) {
            epaper_set_pixel(10, y, true);
            epaper_set_pixel(EPAPER_WIDTH - 11, y, true);
        }
        printf("Rectangle drawn 10px from edges\n");
    }
    else if (strcmp(cmd, "fill") == 0 && argc >= 6) {
        int x = atoi(argv[2]);
        int y = atoi(argv[3]);
        int w = atoi(argv[4]);
        int h = atoi(argv[5]);
        for (int py = y; py < y + h; py++) {
            for (int px = x; px < x + w; px++) {
                epaper_set_pixel(px, py, true);
            }
        }
        printf("Filled rect at (%d,%d) size %dx%d\n", x, y, w, h);
    }
    else if (strcmp(cmd, "text") == 0 && argc >= 5) {
        int x = atoi(argv[2]);
        int y = atoi(argv[3]);
        epaper_draw_string(x, y, argv[4], 1);
        printf("Text '%s' at (%d,%d)\n", argv[4], x, y);
    }
    else if (strcmp(cmd, "update") == 0) {
        printf("Updating display...\n");
        epaper_update();
        printf("Display updated\n");
    }
    else if (strcmp(cmd, "test") == 0) {
        printf("Running test pattern...\n");
        epaper_test_pattern();
        printf("Test complete\n");
    }
    else if (strcmp(cmd, "info") == 0) {
        printf("Display: %dx%d\n", EPAPER_WIDTH, EPAPER_HEIGHT);
        printf("Framebuffer: %d bytes\n", EPAPER_FB_SIZE);
    }
    else {
        printf("Unknown command: %s\n", cmd);
    }

    return 0;
}

static void register_ep_command(void) {
    const esp_console_cmd_t cmd = {
        .command = "ep",
        .help = "E-paper display commands",
        .hint = NULL,
        .func = &ep_cmd_handler,
    };
    esp_console_cmd_register(&cmd);
}

/**
 * @brief Application entry point
 */
void app_main(void) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  E-PAPER PARTIAL REFRESH TEST");
    ESP_LOGI(TAG, "  SSD1680 2.9\" 296x128");
    ESP_LOGI(TAG, "  159-byte LUT, 0x0F partial cmd");
    ESP_LOGI(TAG, "========================================");

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize network (WiFi + OTA + telnet)
    ESP_LOGI(TAG, "Initializing network...");
    ret = network_manager_init(
        CONFIG_TESTAP2_WIFI_SSID,
        CONFIG_TESTAP2_WIFI_PASSWORD,
        "testapen-epaper-test",
        CONFIG_TESTAP2_DEBUG_PORT
    );
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Network init failed: %s (continuing anyway)", esp_err_to_name(ret));
    }

    // Wait for WiFi to connect
    vTaskDelay(pdMS_TO_TICKS(5000));

    // Initialize e-Paper display
    ESP_LOGI(TAG, "Initializing e-Paper display...");
    ret = epaper_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize e-Paper: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "Display initialized successfully");
    ESP_LOGI(TAG, "  Width:  %d", EPAPER_WIDTH);
    ESP_LOGI(TAG, "  Height: %d", EPAPER_HEIGHT);

    // Register console commands
    register_ep_command();
    ESP_LOGI(TAG, "Console commands registered - use 'ep help' for commands");

    // Run initial test pattern
    ESP_LOGI(TAG, "Running initial test pattern...");
    epaper_test_pattern();

    // Create console status task
    xTaskCreate(console_task, "console", 2048, NULL, 3, NULL);

    ESP_LOGI(TAG, "Ready for interactive testing via telnet");
    ESP_LOGI(TAG, "Try: ep clear && ep hline 64 && ep update");
}
