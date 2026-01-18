/**
 * @file ui_node.c
 * @brief UI Node Implementation for TestAP2
 *
 * FSD Reference: TestAP2.FSD.v1.0.0.md Section 6.4 (TBD)
 *
 * UI Node Tasks:
 * - Task_CAN: CAN message handling, heartbeat receive (Priority 5, 50Hz)
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
#include "can_protocol.h"
#include "state_machine.h"
#include "network_manager.h"
#include "cmd_console.h"
#include "param_store.h"
#include "epaper.h"

#ifdef CONFIG_TESTAP2_NODE_UI

static const char *TAG = "UI";

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
static uint32_t g_last_full_refresh_ms __attribute__((unused)) = 0;  // For e-Paper full refresh timing

#define PARTIAL_REFRESH_MS      2000    // Partial refresh every 2 seconds
#define FULL_REFRESH_MS         60000   // Full refresh every 60 seconds

/*============================================================================
 * CAN Command State
 *============================================================================*/

static uint8_t g_ui_cmd_sequence = 0;

/*============================================================================
 * Task Handles
 *============================================================================*/

static TaskHandle_t h_task_can = NULL;
static TaskHandle_t h_task_buttons = NULL;
static TaskHandle_t h_task_display = NULL;
static TaskHandle_t h_task_network = NULL;

/*============================================================================
 * Forward Declarations
 *============================================================================*/

static void task_can(void *pvParameters);
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
    // Configure button GPIOs
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
        // They need external pull-ups on the hardware
        if (button_gpios[i] >= 34) {
            io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        }

        gpio_config(&io_conf);
        ESP_LOGI(TAG, "Button %d configured on GPIO %d", i, button_gpios[i]);
    }
}

/**
 * @brief Read and debounce a button
 * @return true if button state changed
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
            // Long press detected
            if (!g_buttons[btn].long_press_active) {
                // First long press trigger
                g_buttons[btn].long_press_active = true;
                g_buttons[btn].last_action_ms = now;
                process_button_action(btn, true);
            } else if (now - g_buttons[btn].last_action_ms >= REPEAT_RATE_MS) {
                // Repeat action
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
            // Decrease heading by 10 degrees
            send_ui_command(UI_CMD_HEADING_ADJUST, -100);  // -10.0 * 10
            break;

        case BTN_DEC1:
            // Decrease heading by 1 degree
            send_ui_command(UI_CMD_HEADING_ADJUST, -10);   // -1.0 * 10
            break;

        case BTN_ENGAGE:
            // Toggle engage/disengage (only on short press)
            if (!long_press) {
                if (g_master_status.state == STATE_ENGAGED) {
                    send_ui_command(UI_CMD_DISENGAGE, 0);
                } else {
                    send_ui_command(UI_CMD_ENGAGE, 0);
                }
            }
            break;

        case BTN_MODE:
            // Cycle display page (only on short press)
            if (!long_press) {
                g_current_page = (g_current_page + 1) % PAGE_COUNT;
                ESP_LOGI(TAG, "Switched to page %d", g_current_page);
            }
            break;

        case BTN_INC1:
            // Increase heading by 1 degree
            send_ui_command(UI_CMD_HEADING_ADJUST, 10);    // +1.0 * 10
            break;

        case BTN_INC10:
            // Increase heading by 10 degrees
            send_ui_command(UI_CMD_HEADING_ADJUST, 100);   // +10.0 * 10
            break;

        default:
            break;
    }
}

/*============================================================================
 * CAN Functions
 *============================================================================*/

/**
 * @brief Send UI command to Master via CAN
 */
static void send_ui_command(uint8_t cmd, int16_t value) {
    uint8_t data[8] = {0};

    data[0] = cmd;
    int16_to_be(value, &data[1]);
    data[3] = g_ui_cmd_sequence++;

    esp_err_t err = can_send(CAN_ID_UI_COMMAND, data, 8, 10);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send UI command: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Sent UI cmd=%d value=%d", cmd, value);
    }
}

/**
 * @brief Process received CAN messages
 */
static void process_can_message(const twai_message_t *msg) {
    uint32_t now = esp_log_timestamp();

    switch (msg->identifier) {
        case CAN_ID_MASTER_HEARTBEAT: {
            // Parse Master heartbeat
            g_master_status.last_heartbeat_ms = now;
            g_master_status.connected = true;
            g_master_status.state = msg->data[0];
            g_master_status.fault_code = msg->data[1];
            g_heading = be_to_int16(&msg->data[2]) / 10.0f;
            g_target_heading = be_to_int16(&msg->data[4]) / 10.0f;
            g_master_flags = msg->data[7];
            break;
        }

        case CAN_ID_RUDDER_HEARTBEAT: {
            // Parse Rudder heartbeat
            g_rudder_status.last_heartbeat_ms = now;
            g_rudder_status.connected = true;
            g_rudder_status.state = msg->data[0];
            g_rudder_status.fault_code = msg->data[1];
            g_rudder_angle = be_to_int16(&msg->data[2]) / 10.0f;
            g_motor_status = msg->data[4];
            break;
        }

        // TODO: Handle CAN_ID_GNSS_POSITION and CAN_ID_GNSS_VELOCITY

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

/**
 * @brief Draw status bar at top of display
 */
static void draw_status_bar(void) {
    // Status bar showing node connectivity
    const char *m_status = g_master_status.connected ? "OK" : "--";
    const char *r_status = g_rudder_status.connected ? "OK" : "--";
    const char *g_status = (g_master_flags & FLAG_GNSS_VALID) ? "OK" : "--";

    epaper_printf(0, 0, 1, "M:%s R:%s G:%s  Pg%d",
                  m_status, r_status, g_status, g_current_page + 1);
}

/**
 * @brief Draw Page 1: Autopilot Status
 */
static void draw_page_autopilot(void) {
    // Heading (large) and target (right aligned)
    // Display is 296x128, size 2 = 16px/char
    epaper_printf(0, 12, 2, "HDG:%5.1f", g_heading);
    epaper_printf(168, 12, 2, "TGT:%5.1f", g_target_heading);

    // Rudder angle
    const char *dir = "";
    if (g_rudder_angle > 0.5f) dir = "STBD";
    else if (g_rudder_angle < -0.5f) dir = "PORT";
    epaper_printf(0, 44, 2, "RUD:%+5.1f %s", g_rudder_angle, dir);

    // State and error on smaller lines
    const char *state_str = state_to_string((system_state_t)g_master_status.state);
    float error = g_target_heading - g_heading;
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    epaper_printf(0, 76, 1, "State: %s  Err: %+.1f", state_str, error);

    // Fault code if any
    if (g_master_status.fault_code != 0) {
        epaper_printf(0, 90, 1, "FAULT: 0x%02X", g_master_status.fault_code);
    }
}

/**
 * @brief Draw Page 2: Navigation
 */
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

/**
 * @brief Draw Page 3: System Status
 */
static void draw_page_system(void) {
    epaper_printf(0, 16, 1, "=== SYSTEM STATUS ===");

    // Master status
    if (g_master_status.connected) {
        uint32_t age = esp_log_timestamp() - g_master_status.last_heartbeat_ms;
        epaper_printf(0, 32, 1, "Master: CONNECTED  HB: %lums", age);
        epaper_printf(0, 44, 1, "  State: %s  Fault: %d",
                      state_to_string((system_state_t)g_master_status.state),
                      g_master_status.fault_code);
    } else {
        epaper_printf(0, 32, 1, "Master: DISCONNECTED");
    }

    // Rudder status
    if (g_rudder_status.connected) {
        uint32_t age = esp_log_timestamp() - g_rudder_status.last_heartbeat_ms;
        epaper_printf(0, 62, 1, "Rudder: CONNECTED  HB: %lums", age);
        epaper_printf(0, 74, 1, "  Angle: %.1f deg", g_rudder_angle);
    } else {
        epaper_printf(0, 62, 1, "Rudder: DISCONNECTED");
    }

    // Network info
    char ip[32];
    network_manager_get_ip(ip, sizeof(ip));
    epaper_printf(0, 92, 1, "IP: %s", ip);
    epaper_printf(0, 104, 1, "Version: %s", TESTAP2_FW_VERSION);
}

/**
 * @brief Update display content
 */
static void display_update(void) {
    if (!g_epaper_initialized) {
        ESP_LOGW(TAG, "e-Paper not initialized");
        g_display_needs_update = false;
        return;
    }

    ESP_LOGI(TAG, "Updating display (page %d)...", g_current_page);

    // Clear framebuffer
    epaper_clear();

    // Draw status bar (all pages)
    draw_status_bar();

    // Draw page content
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

    // Update the physical display
    epaper_update();

    g_display_needs_update = false;
    ESP_LOGI(TAG, "Display update complete");
}

/*============================================================================
 * Tasks
 *============================================================================*/

/**
 * @brief CAN task - receive heartbeats and send commands
 */
static void task_can(void *pvParameters) {
    ESP_LOGI(TAG, "Task_CAN started");

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20);  // 50 Hz

    while (1) {
        // Receive CAN messages
        twai_message_t msg;
        while (can_receive(&msg, 0) == ESP_OK) {
            process_can_message(&msg);
        }

        // Check for node timeouts
        check_node_timeouts();

        vTaskDelayUntil(&last_wake, period);
    }
}

/**
 * @brief Button task - handle user input
 */
static void task_buttons(void *pvParameters) {
    ESP_LOGI(TAG, "Task_Buttons started");

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20);  // 50 Hz

    while (1) {
        // Update all buttons
        for (int i = 0; i < BTN_COUNT; i++) {
            button_update((button_id_t)i);
        }

        vTaskDelayUntil(&last_wake, period);
    }
}

/**
 * @brief Display task - update e-Paper display
 */
static void task_display(void *pvParameters) {
    ESP_LOGI(TAG, "Task_Display started");

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(PARTIAL_REFRESH_MS);

    // Initial display update
    display_update();

    while (1) {
        vTaskDelayUntil(&last_wake, period);

        // Update display if needed
        if (g_display_needs_update) {
            display_update();
        }

        // TODO: Implement actual e-Paper refresh logic
        // - Track last full refresh time
        // - Use partial refresh normally
        // - Force full refresh every FULL_REFRESH_MS
    }
}

/**
 * @brief Network task - handle console commands
 */
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
    return TESTAP2_FW_VERSION;
}

bool ui_is_epaper_initialized(void) {
    return g_epaper_initialized;
}

void ui_force_display_refresh(void) {
    g_display_needs_update = true;
    display_update();
}

/**
 * @brief Get button state information for console display
 * @param buf Output buffer
 * @param size Buffer size
 * @return Number of bytes written
 */
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
    ESP_LOGI(TAG, "Initializing UI Node");

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
        "testap2-ui",
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

    // ========== PHASE 4: e-Paper Display ==========
    ESP_LOGI(TAG, "Initializing e-Paper display...");
    if (epaper_init() == ESP_OK) {
        g_epaper_initialized = true;
        ESP_LOGI(TAG, "e-Paper display initialized");

        // Clear and show initial screen
        epaper_clear();
        epaper_printf(50, 40, 2, "TestAP2");
        epaper_printf(50, 70, 1, "UI Node v%s", TESTAP2_FW_VERSION);
        epaper_printf(50, 90, 1, "Initializing...");
        epaper_update();
    } else {
        ESP_LOGE(TAG, "e-Paper initialization FAILED");
        g_epaper_initialized = false;
    }

    // ========== PHASE 5: Create Tasks ==========
    ESP_LOGI(TAG, "Creating tasks...");

    xTaskCreate(task_can, "Task_CAN", 4096, NULL, 5, &h_task_can);
    xTaskCreate(task_buttons, "Task_Buttons", 2048, NULL, 4, &h_task_buttons);
    xTaskCreate(task_display, "Task_Display", 4096, NULL, 2, &h_task_display);
    xTaskCreate(task_network, "Task_Network", 4096, NULL, 1, &h_task_network);

    ESP_LOGI(TAG, "UI Node initialization complete");
}

#endif // CONFIG_TESTAP2_NODE_UI
