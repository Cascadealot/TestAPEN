/**
 * @file ui_node.c
 * @brief UI Node Implementation for TestAPEN
 *
 * FSD Reference: TestAP2.FSD.v1.0.0.md Section 6.4 (TBD)
 * Modified for ESP-NOW communication instead of CAN bus.
 *
 * UI Node Tasks:
 * - Task_ESPNOW: ESP-NOW message handling, heartbeat receive (Priority 5, 50Hz)
 * - Task_Buttons: Button input with debouncing (Priority 4, 50Hz)
 * - Task_Display: e-Paper display updates (Priority 2, variable rate)
 * - Task_Network: WiFi/OTA/Console support (Priority 1, 10Hz)
 */

#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include "autopilot_common.h"
#include "espnow_protocol.h"
#include "state_machine.h"
#include "network_manager.h"
#include "cmd_console.h"
#include "param_store.h"
#include "epaper.h"

#ifdef CONFIG_TESTAP2_NODE_UI

static const char *TAG = "UI";

// Version
#define FIRMWARE_VERSION "1.1.0-espnow"

/*============================================================================
 * Button Definitions
 *============================================================================*/

typedef enum {
    BTN_DEC10 = 0,    // -10 degrees
    BTN_DEC1,         // -1 degree
    BTN_ENGAGE,       // Engage/Disengage
    BTN_MODE,         // Mode/Page select
    BTN_INC1,         // +1 degree
    BTN_INC10,        // +10 degrees
    BTN_COUNT
} button_id_t;

typedef struct {
    gpio_num_t gpio;
    bool pressed;
    bool last_state;
    uint32_t press_start_ms;
    uint32_t last_action_ms;
    bool long_press_active;
} button_state_t;

static button_state_t g_buttons[BTN_COUNT];

// Button timing constants
#define DEBOUNCE_MS         20
#define LONG_PRESS_MS       1000
#define REPEAT_RATE_MS      200     // 5Hz repeat rate for long press

/*============================================================================
 * Node Status Tracking
 *============================================================================*/

typedef struct {
    uint32_t last_heartbeat_ms;
    uint8_t state;
    uint8_t fault_code;
    bool connected;
} node_status_t;

static node_status_t g_master_status = {0};
static node_status_t g_rudder_status = {0};

#define NODE_TIMEOUT_MS     500

// Data from Master heartbeat
static float g_heading = 0.0f;
static float g_target_heading = 0.0f;
static uint8_t g_master_flags = 0;

// Data from Rudder heartbeat
static float g_rudder_angle = 0.0f;
static uint8_t g_motor_status = 0;

/*============================================================================
 * Display State
 *============================================================================*/

typedef enum {
    PAGE_AUTOPILOT = 0,   // Primary: heading, target, rudder, state
    PAGE_NAVIGATION,       // GNSS: lat, lon, speed, COG
    PAGE_SYSTEM,           // System status: node connectivity, IP
    PAGE_COUNT
} display_page_t;

static display_page_t g_current_page = PAGE_AUTOPILOT;
static bool g_display_needs_update = true;
static uint32_t g_last_full_refresh_ms __attribute__((unused)) = 0;

#define PARTIAL_REFRESH_MS      2000    // Partial refresh every 2 seconds
#define FULL_REFRESH_MS         60000   // Full refresh every 60 seconds

/*============================================================================
 * ESP-NOW Command State
 *============================================================================*/

static uint8_t g_ui_cmd_sequence = 0;

/*============================================================================
 * Task Handles
 *============================================================================*/

static TaskHandle_t h_task_espnow = NULL;
static TaskHandle_t h_task_buttons = NULL;
static TaskHandle_t h_task_display = NULL;
static TaskHandle_t h_task_network = NULL;

/*============================================================================
 * Forward Declarations
 *============================================================================*/

static void task_espnow(void *pvParameters);
static void task_buttons(void *pvParameters);
static void task_display(void *pvParameters);
static void task_network(void *pvParameters);
static void send_ui_command(uint8_t cmd, int16_t value);
static void process_button_action(button_id_t btn, bool long_press);

/*============================================================================
 * Button Input Functions
 *============================================================================*/

/**
 * @brief Initialize button GPIOs
 */
static void buttons_init(void) {
    const gpio_num_t button_gpios[BTN_COUNT] = {
        CONFIG_TESTAP2_BTN_DEC10_GPIO,
        CONFIG_TESTAP2_BTN_DEC1_GPIO,
        CONFIG_TESTAP2_BTN_ENGAGE_GPIO,
        CONFIG_TESTAP2_BTN_MODE_GPIO,
        CONFIG_TESTAP2_BTN_INC1_GPIO,
        CONFIG_TESTAP2_BTN_INC10_GPIO
    };

    for (int i = 0; i < BTN_COUNT; i++) {
        g_buttons[i].gpio = button_gpios[i];
        g_buttons[i].pressed = false;
        g_buttons[i].last_state = true;  // Active low, so idle = high
        g_buttons[i].press_start_ms = 0;
        g_buttons[i].last_action_ms = 0;
        g_buttons[i].long_press_active = false;

        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << button_gpios[i]),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };

        // Note: GPIO 34-39 don't have internal pull-ups
        if (button_gpios[i] >= 34) {
            io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        }

        gpio_config(&io_conf);
        ESP_LOGI(TAG, "Button %d configured on GPIO %d", i, button_gpios[i]);
    }
}

/**
 * @brief Read and debounce a button
 */
static bool button_update(button_id_t btn) {
    bool current = (gpio_get_level(g_buttons[btn].gpio) == 0);  // Active low
    uint32_t now = esp_log_timestamp();

    // Debounce check
    if (current != g_buttons[btn].last_state) {
        g_buttons[btn].last_state = current;

        if (current) {
            // Button just pressed
            g_buttons[btn].press_start_ms = now;
            g_buttons[btn].pressed = true;
            g_buttons[btn].long_press_active = false;
            return true;
        } else {
            // Button just released
            if (g_buttons[btn].pressed && !g_buttons[btn].long_press_active) {
                // Short press - trigger action on release
                process_button_action(btn, false);
            }
            g_buttons[btn].pressed = false;
            g_buttons[btn].long_press_active = false;
            return true;
        }
    }

    // Check for long press
    if (g_buttons[btn].pressed) {
        uint32_t held_time = now - g_buttons[btn].press_start_ms;

        if (held_time >= LONG_PRESS_MS) {
            if (!g_buttons[btn].long_press_active) {
                g_buttons[btn].long_press_active = true;
                g_buttons[btn].last_action_ms = now;
                process_button_action(btn, true);
            } else if (now - g_buttons[btn].last_action_ms >= REPEAT_RATE_MS) {
                g_buttons[btn].last_action_ms = now;
                process_button_action(btn, true);
            }
        }
    }

    return false;
}

/**
 * @brief Process button action
 */
static void process_button_action(button_id_t btn, bool long_press) {
    ESP_LOGI(TAG, "Button %d %s", btn, long_press ? "LONG" : "SHORT");
    g_display_needs_update = true;

    switch (btn) {
        case BTN_DEC10:
            send_ui_command(UI_CMD_HEADING_ADJUST, -100);  // -10.0 * 10
            break;

        case BTN_DEC1:
            send_ui_command(UI_CMD_HEADING_ADJUST, -10);   // -1.0 * 10
            break;

        case BTN_ENGAGE:
            if (!long_press) {
                if (g_master_status.state == STATE_ENGAGED) {
                    send_ui_command(UI_CMD_DISENGAGE, 0);
                } else {
                    send_ui_command(UI_CMD_ENGAGE, 0);
                }
            }
            break;

        case BTN_MODE:
            if (!long_press) {
                g_current_page = (g_current_page + 1) % PAGE_COUNT;
                ESP_LOGI(TAG, "Switched to page %d", g_current_page);
            }
            break;

        case BTN_INC1:
            send_ui_command(UI_CMD_HEADING_ADJUST, 10);    // +1.0 * 10
            break;

        case BTN_INC10:
            send_ui_command(UI_CMD_HEADING_ADJUST, 100);   // +10.0 * 10
            break;

        default:
            break;
    }
}

/*============================================================================
 * ESP-NOW Functions
 *============================================================================*/

/**
 * @brief Send UI command to Master via ESP-NOW
 */
static void send_ui_command(uint8_t cmd, int16_t value) {
    espnow_ui_command_t ui_cmd = {
        .command = cmd,
        .sequence = g_ui_cmd_sequence++,
        .reserved = 0
    };
    int16_to_be(value, (uint8_t*)&ui_cmd.value_x10);

    esp_err_t err = espnow_send(MSG_UI_COMMAND, (uint8_t*)&ui_cmd, sizeof(ui_cmd),
                                 espnow_get_master_mac());
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send UI command: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Sent UI cmd=%d value=%d", cmd, value);
    }
}

/**
 * @brief ESP-NOW receive callback
 */
static void espnow_message_handler(uint8_t msg_type, const uint8_t *data,
                                    size_t len, const uint8_t *src_mac) {
    uint32_t now = esp_log_timestamp();

    switch (msg_type) {
        case MSG_MASTER_HEARTBEAT: {
            if (len < sizeof(espnow_master_heartbeat_t)) break;
            const espnow_master_heartbeat_t *hb = (const espnow_master_heartbeat_t *)data;

            g_master_status.last_heartbeat_ms = now;
            g_master_status.connected = true;
            g_master_status.state = hb->state;
            g_master_status.fault_code = hb->fault_code;
            g_heading = be_to_int16((const uint8_t*)&hb->heading_x10) / 10.0f;
            g_target_heading = be_to_int16((const uint8_t*)&hb->target_x10) / 10.0f;
            g_master_flags = hb->flags;
            break;
        }

        case MSG_RUDDER_HEARTBEAT: {
            if (len < sizeof(espnow_rudder_heartbeat_t)) break;
            const espnow_rudder_heartbeat_t *hb = (const espnow_rudder_heartbeat_t *)data;

            g_rudder_status.last_heartbeat_ms = now;
            g_rudder_status.connected = true;
            g_rudder_status.state = hb->state;
            g_rudder_status.fault_code = hb->fault_code;
            g_rudder_angle = be_to_int16((const uint8_t*)&hb->angle_x10) / 10.0f;
            g_motor_status = hb->motor_status;
            break;
        }

        default:
            // Ignore other messages
            break;
    }
}

/**
 * @brief Check node connectivity timeouts
 */
static void check_node_timeouts(void) {
    uint32_t now = esp_log_timestamp();

    if (g_master_status.connected) {
        if (now - g_master_status.last_heartbeat_ms > NODE_TIMEOUT_MS) {
            g_master_status.connected = false;
            ESP_LOGW(TAG, "Master heartbeat lost");
            g_display_needs_update = true;
        }
    }

    if (g_rudder_status.connected) {
        if (now - g_rudder_status.last_heartbeat_ms > NODE_TIMEOUT_MS) {
            g_rudder_status.connected = false;
            ESP_LOGW(TAG, "Rudder heartbeat lost");
            g_display_needs_update = true;
        }
    }
}

/*============================================================================
 * Display Functions
 *============================================================================*/

static bool g_epaper_initialized = false;

static void draw_status_bar(void) {
    const char *m_status = g_master_status.connected ? "OK" : "--";
    const char *r_status = g_rudder_status.connected ? "OK" : "--";
    const char *g_status = (g_master_flags & FLAG_GNSS_VALID) ? "OK" : "--";

    epaper_printf(0, 0, 1, "M:%s R:%s G:%s  Pg%d",
                  m_status, r_status, g_status, g_current_page + 1);
}

static void draw_page_autopilot(void) {
    epaper_printf(0, 12, 2, "HDG:%5.1f", g_heading);
    epaper_printf(168, 12, 2, "TGT:%5.1f", g_target_heading);

    const char *dir = "";
    if (g_rudder_angle > 0.5f) dir = "STBD";
    else if (g_rudder_angle < -0.5f) dir = "PORT";
    epaper_printf(0, 44, 2, "RUD:%+5.1f %s", g_rudder_angle, dir);

    const char *state_str = state_to_string((system_state_t)g_master_status.state);
    float error = g_target_heading - g_heading;
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    epaper_printf(0, 76, 1, "State: %s  Err: %+.1f", state_str, error);

    if (g_master_status.fault_code != 0) {
        epaper_printf(0, 90, 1, "FAULT: 0x%02X", g_master_status.fault_code);
    }
}

static void draw_page_navigation(void) {
    epaper_printf(0, 20, 2, "NAVIGATION");

    if (g_master_flags & FLAG_GNSS_VALID) {
        epaper_printf(0, 50, 1, "LAT: (data pending)");
        epaper_printf(0, 62, 1, "LON: (data pending)");
        epaper_printf(0, 80, 1, "SOG: -- kts");
        epaper_printf(0, 92, 1, "COG: -- deg");
    } else {
        epaper_printf(0, 50, 2, "NO GPS FIX");
        epaper_printf(0, 84, 1, "Waiting for satellites...");
    }
}

static void draw_page_system(void) {
    epaper_printf(0, 16, 1, "=== SYSTEM STATUS ===");

    if (g_master_status.connected) {
        uint32_t age = esp_log_timestamp() - g_master_status.last_heartbeat_ms;
        epaper_printf(0, 32, 1, "Master: CONNECTED  HB: %lums", age);
        epaper_printf(0, 44, 1, "  State: %s  Fault: %d",
                      state_to_string((system_state_t)g_master_status.state),
                      g_master_status.fault_code);
    } else {
        epaper_printf(0, 32, 1, "Master: DISCONNECTED");
    }

    if (g_rudder_status.connected) {
        uint32_t age = esp_log_timestamp() - g_rudder_status.last_heartbeat_ms;
        epaper_printf(0, 62, 1, "Rudder: CONNECTED  HB: %lums", age);
        epaper_printf(0, 74, 1, "  Angle: %.1f deg", g_rudder_angle);
    } else {
        epaper_printf(0, 62, 1, "Rudder: DISCONNECTED");
    }

    char ip[32];
    network_manager_get_ip(ip, sizeof(ip));
    epaper_printf(0, 92, 1, "IP: %s", ip);
    epaper_printf(0, 104, 1, "Version: %s", FIRMWARE_VERSION);
}

static void display_update(void) {
    if (!g_epaper_initialized) {
        ESP_LOGW(TAG, "e-Paper not initialized");
        g_display_needs_update = false;
        return;
    }

    ESP_LOGI(TAG, "Updating display (page %d)...", g_current_page);

    epaper_clear();
    draw_status_bar();

    switch (g_current_page) {
        case PAGE_AUTOPILOT:
            draw_page_autopilot();
            break;

        case PAGE_NAVIGATION:
            draw_page_navigation();
            break;

        case PAGE_SYSTEM:
            draw_page_system();
            break;

        default:
            break;
    }

    epaper_update();
    g_display_needs_update = false;
    ESP_LOGI(TAG, "Display update complete");
}

/*============================================================================
 * Tasks
 *============================================================================*/

static void task_espnow(void *pvParameters) {
    ESP_LOGI(TAG, "Task_ESPNOW started");

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20);  // 50 Hz

    while (1) {
        // Check for node timeouts
        check_node_timeouts();

        // ESP-NOW receive is handled via callback
        vTaskDelayUntil(&last_wake, period);
    }
}

static void task_buttons(void *pvParameters) {
    ESP_LOGI(TAG, "Task_Buttons started");

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20);  // 50 Hz

    while (1) {
        for (int i = 0; i < BTN_COUNT; i++) {
            button_update((button_id_t)i);
        }

        vTaskDelayUntil(&last_wake, period);
    }
}

static void task_display(void *pvParameters) {
    ESP_LOGI(TAG, "Task_Display started");

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(PARTIAL_REFRESH_MS);

    // Initial display update
    display_update();

    while (1) {
        vTaskDelayUntil(&last_wake, period);

        if (g_display_needs_update) {
            display_update();
        }
    }
}

static void task_network(void *pvParameters) {
    ESP_LOGI(TAG, "Task_Network started");

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(100);  // 10 Hz

    while (1) {
        console_handle();
        vTaskDelayUntil(&last_wake, period);
    }
}

/*============================================================================
 * Public Interface Functions (for console)
 *============================================================================*/

uint8_t ui_get_state(void) {
    return g_master_status.state;
}

uint8_t ui_get_fault_code(void) {
    return g_master_status.fault_code;
}

float ui_get_heading(void) {
    return g_heading;
}

float ui_get_target_heading(void) {
    return g_target_heading;
}

float ui_get_rudder_angle(void) {
    return g_rudder_angle;
}

bool ui_get_master_connected(void) {
    return g_master_status.connected;
}

bool ui_get_rudder_connected(void) {
    return g_rudder_status.connected;
}

void ui_set_page(int page) {
    if (page >= 0 && page < PAGE_COUNT) {
        g_current_page = (display_page_t)page;
        g_display_needs_update = true;
    }
}

int ui_get_page(void) {
    return (int)g_current_page;
}

bool ui_is_wifi_connected(void) {
    return network_manager_is_connected();
}

void ui_get_ip_address(char *buf, size_t buf_size) {
    network_manager_get_ip(buf, buf_size);
}

const char* ui_get_version(void) {
    return FIRMWARE_VERSION;
}

bool ui_is_epaper_initialized(void) {
    return g_epaper_initialized;
}

void ui_force_display_refresh(void) {
    g_display_needs_update = true;
    display_update();
}

size_t ui_get_button_states(char *buf, size_t size) {
    const char *btn_names[] = {"-10", "-1", "ENG", "MOD", "+1", "+10"};
    size_t len = 0;

    len += snprintf(buf + len, size - len, "Button States:\r\n");
    len += snprintf(buf + len, size - len, "  %-6s %-6s %-8s %-8s %-8s\r\n",
                    "BTN", "GPIO", "PRESSED", "LONG", "GPIO_LVL");

    for (int i = 0; i < BTN_COUNT; i++) {
        int level = gpio_get_level(g_buttons[i].gpio);
        len += snprintf(buf + len, size - len, "  %-6s %-6d %-8s %-8s %d\r\n",
                       btn_names[i],
                       g_buttons[i].gpio,
                       g_buttons[i].pressed ? "YES" : "no",
                       g_buttons[i].long_press_active ? "YES" : "no",
                       level);
    }

    len += snprintf(buf + len, size - len, "\r\nGPIO LOW (0) = pressed, HIGH (1) = released\r\n");
    return len;
}

/*============================================================================
 * Initialization
 *============================================================================*/

void ui_node_init(void) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  TestAPEN UI Node");
    ESP_LOGI(TAG, "  Version: %s", FIRMWARE_VERSION);
    ESP_LOGI(TAG, "========================================");

    // ========== PHASE 1: Buttons ==========
    ESP_LOGI(TAG, "Initializing buttons...");
    buttons_init();
    ESP_LOGI(TAG, "Buttons initialized");

    // ========== PHASE 2: Parameter Store ==========
    ESP_LOGI(TAG, "Initializing param store...");
    if (param_store_init() != ESP_OK) {
        ESP_LOGW(TAG, "Parameter store init failed (using defaults)");
    } else {
        ESP_LOGI(TAG, "Parameter store initialized");
    }

    // ========== PHASE 3: Network ==========
    ESP_LOGI(TAG, "Initializing network...");
    esp_err_t net_err = network_manager_init(
        CONFIG_TESTAP2_WIFI_SSID,
        CONFIG_TESTAP2_WIFI_PASSWORD,
        "testapen-ui",
        CONFIG_TESTAP2_DEBUG_PORT
    );
    if (net_err != ESP_OK) {
        ESP_LOGW(TAG, "Network initialization failed (continuing without network)");
    } else {
        char ip[16];
        network_manager_get_ip(ip, sizeof(ip));
        ESP_LOGI(TAG, "Network ready - IP: %s, Debug port: %d", ip, CONFIG_TESTAP2_DEBUG_PORT);
    }

    // Initialize console
    console_init();
    network_manager_set_command_callback(console_process_command);

    // ========== PHASE 4: ESP-NOW ==========
    ESP_LOGI(TAG, "Initializing ESP-NOW...");
    if (espnow_init() != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW initialization failed!");
    } else {
        ESP_LOGI(TAG, "ESP-NOW initialized");
        espnow_register_recv_callback(espnow_message_handler);
    }

    // ========== PHASE 5: e-Paper Display ==========
    ESP_LOGI(TAG, "Initializing e-Paper display...");
    if (epaper_init() == ESP_OK) {
        g_epaper_initialized = true;
        ESP_LOGI(TAG, "e-Paper display initialized");

        // Clear and show initial screen
        epaper_clear();
        epaper_printf(50, 40, 2, "TestAPEN");
        epaper_printf(50, 70, 1, "UI Node v%s", FIRMWARE_VERSION);
        epaper_printf(50, 90, 1, "Initializing...");
        epaper_update();
    } else {
        ESP_LOGE(TAG, "e-Paper initialization FAILED");
        g_epaper_initialized = false;
    }

    // ========== PHASE 6: Create Tasks ==========
    ESP_LOGI(TAG, "Creating tasks...");

    xTaskCreate(task_espnow, "Task_ESPNOW", 4096, NULL, 5, &h_task_espnow);
    xTaskCreate(task_buttons, "Task_Buttons", 2048, NULL, 4, &h_task_buttons);
    xTaskCreate(task_display, "Task_Display", 4096, NULL, 2, &h_task_display);
    xTaskCreate(task_network, "Task_Network", 4096, NULL, 1, &h_task_network);

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  UI Node initialization complete");
    ESP_LOGI(TAG, "========================================");
}

#endif // CONFIG_TESTAP2_NODE_UI
