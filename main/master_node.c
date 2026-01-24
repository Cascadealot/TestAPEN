/**
 * @file master_node.c
 * @brief Master Node Implementation for TestAPEN
 *
 * FSD Reference: TestAPEN.FSD.v1.0.0.md Section 6.2
 * Modified for ESP-NOW communication instead of CAN bus.
 *
 * Master Node Tasks:
 * - Task_ESPNOW: ESP-NOW message handling (Priority 5)
 * - Task_IMU: IMU reading and heading computation (Priority 4, 50Hz)
 * - Task_Autopilot: Heading control (Priority 3, 10Hz)
 * - Task_Display: OLED display update (Priority 2, 5Hz)
 * - Task_Network: Network service (Priority 1)
 */

#include <math.h>
#include <stdarg.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/uart.h"

#include "autopilot_common.h"
#include "espnow_protocol.h"
#include "state_machine.h"
#include "icm20948.h"
#include "ssd1306.h"
#include "network_manager.h"
#include "cmd_console.h"
#include "param_store.h"
#include "ble_manager.h"
#include "gnss_driver.h"

#ifdef CONFIG_TESTAPEN_NODE_MASTER

static const char *TAG = "MASTER";

// Version
#define FIRMWARE_VERSION "1.1.0-espnow"

/*============================================================================
 * Global State
 *============================================================================*/

static state_machine_t g_state_machine;
static SemaphoreHandle_t g_i2c_mutex = NULL;

// Heading data
static float g_target_heading = 0.0f;
static SemaphoreHandle_t g_heading_mutex = NULL;

// Sequence counters
static uint8_t g_heartbeat_seq = 0;
static uint8_t g_command_seq = 0;

/*============================================================================
 * Task Handles
 *============================================================================*/

static TaskHandle_t h_task_espnow = NULL;
static TaskHandle_t h_task_imu = NULL;
static TaskHandle_t h_task_gnss = NULL;
static TaskHandle_t h_task_autopilot = NULL;
static TaskHandle_t h_task_display = NULL;
static TaskHandle_t h_task_network = NULL;

// GNSS status
static bool g_gnss_available = false;

/*============================================================================
 * Interface Functions (for console)
 *============================================================================*/

system_state_t master_get_state(void) {
    return state_machine_get_state(&g_state_machine);
}

uint8_t master_get_fault_code(void) {
    return g_state_machine.fault_code;
}

bool master_engage(void) {
    if (state_machine_get_state(&g_state_machine) == STATE_IDLE) {
        // Capture current heading as target
        g_target_heading = icm20948_get_heading();

        // Build engage preconditions
        engage_preconditions_t pre = {
            .heading_valid = icm20948_is_valid(),
            .rudder_feedback_valid = true,  // Rudder validates itself
            .calibration_valid = true,      // Master doesn't need calibration
            .no_active_faults = (g_state_machine.fault_code == 0)
        };

        state_machine_process(&g_state_machine, EVENT_ENGAGE, &pre);
        return state_machine_get_state(&g_state_machine) == STATE_ENGAGED;
    }
    return false;
}

bool master_disengage(void) {
    if (state_machine_get_state(&g_state_machine) == STATE_ENGAGED) {
        state_machine_process(&g_state_machine, EVENT_DISENGAGE, NULL);
        return state_machine_get_state(&g_state_machine) == STATE_IDLE;
    }
    return false;
}

bool master_clear_fault(void) {
    if (state_machine_get_state(&g_state_machine) == STATE_FAULTED) {
        state_machine_process(&g_state_machine, EVENT_FAULT_CLEAR, NULL);
        return state_machine_get_state(&g_state_machine) == STATE_IDLE;
    }
    return false;
}

bool master_enter_calibration(void) {
    if (state_machine_get_state(&g_state_machine) == STATE_IDLE) {
        state_machine_process(&g_state_machine, EVENT_CAL_ENTER, NULL);
        return state_machine_get_state(&g_state_machine) == STATE_CALIBRATION;
    }
    return false;
}

bool master_exit_calibration(void) {
    if (state_machine_get_state(&g_state_machine) == STATE_CALIBRATION) {
        state_machine_process(&g_state_machine, EVENT_CAL_EXIT, NULL);
        return state_machine_get_state(&g_state_machine) == STATE_IDLE;
    }
    return false;
}

float master_get_heading_filtered(void) {
    return icm20948_get_heading();
}

float master_get_heading_raw(void) {
    return icm20948_get_heading_raw();
}

float master_get_target_heading(void) {
    return g_target_heading;
}

float master_get_heading_error(void) {
    return wrap180(g_target_heading - icm20948_get_heading());
}

void master_set_target_heading(float heading) {
    while (heading < 0.0f) heading += 360.0f;
    while (heading >= 360.0f) heading -= 360.0f;
    g_target_heading = heading;
}

void master_adjust_target_heading(float delta) {
    float heading = g_target_heading + delta;
    while (heading < 0.0f) heading += 360.0f;
    while (heading >= 360.0f) heading -= 360.0f;
    g_target_heading = heading;
}

float master_get_yaw_rate(void) {
    return icm20948_get_yaw_rate();
}

float master_get_roll(void) {
    icm20948_data_t data;
    icm20948_get_data(&data);
    return data.roll;
}

float master_get_pitch(void) {
    icm20948_data_t data;
    icm20948_get_data(&data);
    return data.pitch;
}

bool master_imu_is_valid(void) {
    return icm20948_is_valid();
}

void master_set_heading_simulation(bool enable) {
    icm20948_set_simulation(enable);
}

bool master_is_heading_simulation(void) {
    return icm20948_is_simulation();
}

void master_set_simulated_heading(float heading) {
    icm20948_set_simulated_heading(heading);
}

void master_set_mag_cal(float x, float y, float z) {
    icm20948_set_mag_cal(x, y, z);
}

void master_get_mag_cal(float *x, float *y, float *z) {
    icm20948_get_mag_cal(x, y, z);
}

/*============================================================================
 * IMU Calibration Interface Functions
 *============================================================================*/

esp_err_t master_magcal_start(void) {
    return icm20948_magcal_start();
}

esp_err_t master_magcal_stop(void) {
    return icm20948_magcal_stop();
}

bool master_magcal_is_running(void) {
    return icm20948_magcal_is_running();
}

size_t master_magcal_get_status(char *buf, size_t size) {
    return icm20948_magcal_get_status(buf, size);
}

esp_err_t master_magcal_save(void) {
    return icm20948_save_mag_cal_to_nvs();
}

esp_err_t master_accel_cal_level(void) {
    return icm20948_accel_cal_level();
}

void master_get_accel_cal(float *x, float *y, float *z) {
    icm20948_get_accel_cal(x, y, z);
}

esp_err_t master_gyro_cal(void) {
    return icm20948_calibrate_gyro();
}

void master_get_gyro_bias(float *x, float *y, float *z) {
    icm20948_get_gyro_bias(x, y, z);
}

void master_set_fusion_enabled(bool enable) {
    icm20948_set_fusion_enabled(enable);
}

bool master_is_fusion_enabled(void) {
    return icm20948_is_fusion_enabled();
}

void master_set_fusion_beta(float beta) {
    icm20948_set_fusion_beta(beta);
}

float master_get_fusion_beta(void) {
    return icm20948_get_fusion_beta();
}

bool master_is_fusion_converged(void) {
    return icm20948_is_fusion_converged();
}

float master_get_heading_offset(void) {
    return icm20948_get_heading_offset();
}

uint32_t master_get_gps_sample_count(void) {
    return icm20948_get_gps_sample_count();
}

void master_reset_gps_offset(void) {
    icm20948_reset_gps_offset();
}

esp_err_t master_apply_gps_offset(void) {
    return icm20948_apply_gps_offset();
}

/*============================================================================
 * GNSS Interface Functions
 *============================================================================*/

bool master_gnss_available(void) {
    return g_gnss_available;
}

bool master_gnss_has_fix(void) {
    return g_gnss_available && gnss_has_fix();
}

bool master_gnss_cog_valid(void) {
    return g_gnss_available && gnss_cog_valid();
}

float master_gnss_get_cog(void) {
    if (!g_gnss_available) return NAN;
    return gnss_get_cog();
}

float master_gnss_get_speed(void) {
    if (!g_gnss_available) return 0.0f;
    return gnss_get_speed_kts();
}

esp_err_t master_gnss_get_position(float *lat, float *lon) {
    if (!g_gnss_available) return ESP_ERR_INVALID_STATE;
    return gnss_get_position(lat, lon);
}

void master_gnss_get_status(gnss_data_t *data) {
    if (g_gnss_available) {
        gnss_get_data(data);
    } else {
        memset(data, 0, sizeof(gnss_data_t));
    }
}

float master_get_blended_heading(void) {
    float compass = icm20948_get_heading();
    if (!g_gnss_available) return compass;
    return gnss_blend_heading(compass);
}

bool master_is_wifi_connected(void) {
    return network_manager_is_connected();
}

void master_get_ip_address(char *buf, size_t buf_size) {
    network_manager_get_ip(buf, buf_size);
}

const char* master_get_version(void) {
    return FIRMWARE_VERSION;
}

/*============================================================================
 * Calibration Commands (Master -> Rudder via ESP-NOW)
 *============================================================================*/

/**
 * @brief Send calibration command to Rudder node via ESP-NOW
 */
static void send_calibration_cmd(uint8_t cal_cmd) {
    espnow_calibration_command_t cmd = {
        .command = cal_cmd,
        .reserved = {0}
    };

    esp_err_t err = espnow_send(MSG_CALIBRATION_CMD, (uint8_t*)&cmd, sizeof(cmd),
                                 espnow_get_rudder_mac());
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send cal cmd 0x%02X via ESP-NOW", cal_cmd);
    } else {
        ESP_LOGI(TAG, "Sent calibration cmd: 0x%02X", cal_cmd);
    }
}

bool master_calibrate_center(void) {
    if (state_machine_get_state(&g_state_machine) == STATE_CALIBRATION) {
        send_calibration_cmd(CAL_CMD_CENTER);
        return true;
    }
    return false;
}

bool master_calibrate_port(void) {
    if (state_machine_get_state(&g_state_machine) == STATE_CALIBRATION) {
        send_calibration_cmd(CAL_CMD_PORT);
        return true;
    }
    return false;
}

bool master_calibrate_starboard(void) {
    if (state_machine_get_state(&g_state_machine) == STATE_CALIBRATION) {
        send_calibration_cmd(CAL_CMD_STARBOARD);
        return true;
    }
    return false;
}

bool master_calibrate_save(void) {
    if (state_machine_get_state(&g_state_machine) == STATE_CALIBRATION) {
        send_calibration_cmd(CAL_CMD_SAVE);
        return true;
    }
    return false;
}

/*============================================================================
 * Parameter Sync (Master -> Rudder via ESP-NOW)
 *============================================================================*/

/**
 * @brief Send parameter update to Rudder node via ESP-NOW
 */
static void master_send_param_to_rudder(param_id_t id, float value, bool save_nvs) {
    espnow_param_config_t cfg = {
        .param_id = (uint8_t)id,
        .flags = save_nvs ? PARAM_FLAG_SAVE_NVS : 0,
        .value = value,
        .reserved = 0
    };

    esp_err_t err = espnow_send(MSG_PARAM_CONFIG, (uint8_t*)&cfg, sizeof(cfg),
                                 espnow_get_rudder_mac());
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send param %d via ESP-NOW", id);
    } else {
        const param_meta_t *meta = param_get_meta(id);
        ESP_LOGI(TAG, "Sent param via ESP-NOW: %s = %.3f",
                 meta ? meta->name : "?", value);
    }
}

/**
 * @brief Callback when any parameter changes
 *
 * If the parameter belongs to Rudder node, forward via ESP-NOW.
 */
static void on_param_change(param_id_t id, float value) {
    // If this is a Rudder parameter (not a Master parameter), send via ESP-NOW
    const param_meta_t *meta = param_get_meta(id);
    if (meta && !meta->is_master_param) {
        // This is a Rudder parameter - forward to Rudder node
        master_send_param_to_rudder(id, value, false);
    }
}

/*============================================================================
 * ESP-NOW Receive Callback
 *============================================================================*/

static void espnow_message_handler(uint8_t msg_type, const uint8_t *data,
                                    size_t len, const uint8_t *src_mac) {
    char mac_str[18];
    espnow_mac_to_str(src_mac, mac_str);

    switch (msg_type) {
        case MSG_RUDDER_HEARTBEAT:
            // Process rudder heartbeat
            ESP_LOGD(TAG, "Rudder heartbeat from %s", mac_str);
            break;

        case MSG_E_STOP:
            ESP_LOGW(TAG, "E_STOP received from %s!", mac_str);
            state_machine_set_fault(&g_state_machine, ERR_UNKNOWN);
            break;

        case MSG_UI_COMMAND: {
            // Process UI command from UiNode
            if (len < sizeof(espnow_ui_command_t)) {
                ESP_LOGW(TAG, "UI command too short: %d bytes", (int)len);
                break;
            }
            const espnow_ui_command_t *cmd = (const espnow_ui_command_t *)data;
            int16_t value = be_to_int16((const uint8_t*)&cmd->value_x10);

            ESP_LOGI(TAG, "UI command from %s: cmd=%d value=%d seq=%d",
                     mac_str, cmd->command, value, cmd->sequence);

            switch (cmd->command) {
                case UI_CMD_ENGAGE:
                    master_engage();
                    break;
                case UI_CMD_DISENGAGE:
                    master_disengage();
                    break;
                case UI_CMD_HEADING_ADJUST:
                    // value is degrees * 10
                    master_adjust_target_heading(value / 10.0f);
                    break;
                case UI_CMD_HEADING_SET:
                    // value is degrees * 10
                    master_set_target_heading(value / 10.0f);
                    break;
                default:
                    ESP_LOGW(TAG, "Unknown UI command: %d", cmd->command);
                    break;
            }
            break;
        }

        default:
            ESP_LOGD(TAG, "Unknown message type 0x%02X from %s", msg_type, mac_str);
            break;
    }
}

/*============================================================================
 * Task: ESP-NOW Message Handler
 *============================================================================*/

static void task_espnow(void *pvParameters) {
    ESP_LOGI(TAG, "Task_ESPNOW started");
    TickType_t last_heartbeat = xTaskGetTickCount();

    while (1) {
        // Send heartbeat at 10 Hz (100ms interval)
        if ((xTaskGetTickCount() - last_heartbeat) >= pdMS_TO_TICKS(MASTER_HEARTBEAT_INTERVAL_MS)) {
            espnow_master_heartbeat_t hb = {
                .state = (uint8_t)state_machine_get_state(&g_state_machine),
                .fault_code = g_state_machine.fault_code,
                .sequence = g_heartbeat_seq++,
                .flags = 0
            };

            float filtered = icm20948_get_heading();

            // Convert heading values to big-endian
            int16_to_be((int16_t)(filtered * 10), (uint8_t*)&hb.heading_x10);
            int16_to_be((int16_t)(g_target_heading * 10), (uint8_t*)&hb.target_x10);

            // Set flags
            if (g_gnss_available && gnss_has_fix()) {
                hb.flags |= FLAG_GNSS_VALID;
            }

            // Broadcast heartbeat to all nodes
            esp_err_t err = espnow_send(MSG_MASTER_HEARTBEAT, (uint8_t*)&hb, sizeof(hb), NULL);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "ESP-NOW send failed: %s", esp_err_to_name(err));
            }
            last_heartbeat = xTaskGetTickCount();
        }

        // ESP-NOW receive is handled via callback, just yield here
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/*============================================================================
 * Task: IMU Reader
 *============================================================================*/

static void task_imu(void *pvParameters) {
    ESP_LOGI(TAG, "Task_IMU started");
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / RATE_IMU_HZ);

    while (1) {
        // Read IMU (driver handles I2C mutex)
        esp_err_t err = icm20948_update();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "IMU update failed: %s", esp_err_to_name(err));
        }

        vTaskDelayUntil(&last_wake, period);
    }
}

/*============================================================================
 * Task: GNSS Reader
 *============================================================================*/

static void task_gnss(void *pvParameters) {
    ESP_LOGI(TAG, "Task_GNSS started");
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20);  // 50Hz polling for 10Hz data

    while (1) {
        // Process incoming GNSS data
        gnss_update();

        vTaskDelayUntil(&last_wake, period);
    }
}

/*============================================================================
 * Task: Autopilot Control
 *============================================================================*/

static void task_autopilot(void *pvParameters) {
    ESP_LOGI(TAG, "Task_Autopilot started");
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / RATE_AUTOPILOT_HZ);
    const float dt = 1.0f / RATE_AUTOPILOT_HZ;

    static float integral = 0.0f;

    while (1) {
        // Only run control when ENGAGED
        if (state_machine_get_state(&g_state_machine) == STATE_ENGAGED) {
            float filtered_heading = icm20948_get_heading();
            float yaw_rate = icm20948_get_yaw_rate();

            // PID Control (FSD Section 7.2)
            float error = wrap180(g_target_heading - filtered_heading);

            // Get runtime PID gains from param_store
            float kp = param_get(PARAM_KP_HEADING);
            float ki = param_get(PARAM_KI_HEADING);
            float kd = param_get(PARAM_KD_HEADING);

            // Proportional
            float P = kp * error;

            // Integral with anti-windup
            integral += ki * error * dt;
            integral = clampf(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
            if (fabsf(error) > INTEGRAL_RESET_THRESH) {
                integral = 0.0f;
            }

            // Derivative (using gyro)
            float D = -kd * yaw_rate;

            // Output
            float rudder_cmd = P + integral + D;
            rudder_cmd = clampf(rudder_cmd, -RUDDER_CMD_MAX, RUDDER_CMD_MAX);

            // Send rudder command via ESP-NOW
            espnow_rudder_command_t cmd = {
                .flags = 0,
                .sequence = g_command_seq++,
                .reserved = 0
            };
            int16_to_be((int16_t)(rudder_cmd * 10), (uint8_t*)&cmd.cmd_angle_x10);
            espnow_send(MSG_RUDDER_COMMAND, (uint8_t*)&cmd, sizeof(cmd),
                        espnow_get_rudder_mac());

            ESP_LOGD(TAG, "Autopilot: error=%.1f, cmd=%.1f", error, rudder_cmd);
        } else {
            // Reset integral when not engaged
            integral = 0.0f;
        }

        // Update GPS-aided calibration monitoring when COG is valid
        if (g_gnss_available && gnss_cog_valid()) {
            float cog = gnss_get_cog();
            float speed = gnss_get_speed_kts();
            icm20948_update_gps_reference(cog, speed);
        }

        vTaskDelayUntil(&last_wake, period);
    }
}

/*============================================================================
 * Task: Display Update
 *============================================================================*/

static void task_display(void *pvParameters) {
    ESP_LOGI(TAG, "Task_Display started");
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / RATE_DISPLAY_HZ);

    while (1) {
        system_state_t state = state_machine_get_state(&g_state_machine);
        float heading = icm20948_get_heading();
        float target = g_target_heading;

        // Line 0: State
        ssd1306_printf(0, "%-16s", state_to_string(state));

        // Line 1: Heading
        ssd1306_printf(1, "HDG: %5.1f", heading);

        // Line 2: Target (if engaged)
        if (state == STATE_ENGAGED) {
            float error = wrap180(target - heading);
            ssd1306_printf(2, "TGT: %5.1f E%+.0f", target, error);
        } else {
            ssd1306_printf(2, "TGT: ---.-");
        }

        // Line 3: Status
        if (state == STATE_FAULTED) {
            ssd1306_printf(3, "FAULT: %d", g_state_machine.fault_code);
        } else if (icm20948_is_simulation()) {
            ssd1306_printf(3, "** SIMULATION **");
        } else {
            char ip[16];
            network_manager_get_ip(ip, sizeof(ip));
            ssd1306_printf(3, "%s", ip);
        }

        ssd1306_update();

        vTaskDelayUntil(&last_wake, period);
    }
}

/*============================================================================
 * Task: Network Service
 *============================================================================*/

static void task_network(void *pvParameters) {
    ESP_LOGI(TAG, "Task_Network started");

    while (1) {
        console_handle();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/*============================================================================
 * I2C Initialization
 *============================================================================*/

static esp_err_t i2c_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = CONFIG_TESTAPEN_I2C_SDA_GPIO,
        .scl_io_num = CONFIG_TESTAPEN_I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000
    };

    esp_err_t err = i2c_param_config(I2C_NUM_0, &conf);
    if (err != ESP_OK) return err;

    return i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}

/*============================================================================
 * Master Node Initialization
 *============================================================================*/

// Track init status for boot display
static bool g_display_available = false;

static void boot_status(int line, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    char buf[17];
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    ESP_LOGI(TAG, "Boot: %s", buf);
    if (g_display_available) {
        ssd1306_printf(line, "%-16s", buf);
        ssd1306_update();
    }
}

void master_node_init(void) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  TestAPEN Master Node");
    ESP_LOGI(TAG, "  Version: %s", FIRMWARE_VERSION);
    ESP_LOGI(TAG, "========================================");

    // ========== PHASE 1: I2C and Display (FIRST) ==========
    // Create I2C mutex (FSD Section 6.4)
    g_i2c_mutex = xSemaphoreCreateMutex();
    if (g_i2c_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create I2C mutex!");
        return;
    }

    // Create heading mutex
    g_heading_mutex = xSemaphoreCreateMutex();

    // Initialize I2C
    if (i2c_init() != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed!");
        return;
    }
    ESP_LOGI(TAG, "I2C initialized (SDA=%d, SCL=%d)",
             CONFIG_TESTAPEN_I2C_SDA_GPIO, CONFIG_TESTAPEN_I2C_SCL_GPIO);

    // Initialize display FIRST so we can show boot progress
    esp_err_t disp_err = ssd1306_init(I2C_NUM_0, g_i2c_mutex, 32);
    if (disp_err == ESP_OK) {
        g_display_available = true;
        ssd1306_clear();
        ssd1306_printf(0, "TestAPEN Master");
        ssd1306_printf(1, "v%s", FIRMWARE_VERSION);
        ssd1306_printf(2, "Booting...");
        ssd1306_update();
        ESP_LOGI(TAG, "Display initialized (128x32)");
    } else {
        ESP_LOGW(TAG, "Display init failed (continuing without display)");
    }

    // ========== PHASE 2: State Machine ==========
    boot_status(2, "State machine...");
    state_machine_init(&g_state_machine);
    boot_status(2, "State: OK");

    // ========== PHASE 2.5: Parameter Store ==========
    boot_status(2, "Params...");
    if (param_store_init() != ESP_OK) {
        boot_status(2, "Params: FAIL");
        ESP_LOGW(TAG, "Parameter store init failed (using defaults)");
    } else {
        boot_status(2, "Params: OK");
        ESP_LOGI(TAG, "Parameter store initialized");
        // Register callback to sync Rudder params via ESP-NOW
        param_set_change_callback(on_param_change);
    }

    // ========== PHASE 3: Network (takes longest) ==========
    boot_status(2, "WiFi...");
    esp_err_t net_err = network_manager_init(
        CONFIG_TESTAPEN_WIFI_SSID,
        CONFIG_TESTAPEN_WIFI_PASSWORD,
        "testapen-master",
        CONFIG_TESTAPEN_DEBUG_PORT
    );
    if (net_err != ESP_OK) {
        boot_status(2, "WiFi: FAIL");
        ESP_LOGW(TAG, "Network initialization failed (continuing without network)");
    } else {
        char ip[16];
        network_manager_get_ip(ip, sizeof(ip));
        boot_status(2, "WiFi: %s", ip);
        ESP_LOGI(TAG, "Network ready - IP: %s, Debug port: %d", ip, CONFIG_TESTAPEN_DEBUG_PORT);
    }

    // Initialize console
    console_init();
    network_manager_set_command_callback(console_process_command);

    // ========== PHASE 4: IMU ==========
    boot_status(3, "IMU...");
    if (icm20948_init(I2C_NUM_0, g_i2c_mutex) != ESP_OK) {
        boot_status(3, "IMU: FAIL");
        ESP_LOGW(TAG, "IMU initialization failed!");
        // Don't fault - may work after retry or in simulation mode
    } else {
        boot_status(3, "IMU: OK");
        ESP_LOGI(TAG, "ICM-20948 initialized");
        // Load magnetometer calibration from NVS
        icm20948_load_mag_cal_from_nvs();
    }

    // ========== PHASE 5: ESP-NOW ==========
    boot_status(3, "ESP-NOW...");
    if (espnow_init() != ESP_OK) {
        boot_status(3, "ESP-NOW: FAIL");
        ESP_LOGE(TAG, "ESP-NOW initialization failed!");
    } else {
        boot_status(3, "ESP-NOW: OK");
        ESP_LOGI(TAG, "ESP-NOW initialized");
        // Register receive callback
        espnow_register_recv_callback(espnow_message_handler);
    }

    // ========== PHASE 5.3: GNSS ==========
    boot_status(3, "GNSS...");
    // GNSS on UART2: TX=17, RX=16 (per FSD Section 4.1)
    if (gnss_init(UART_NUM_2, 17, 16) != ESP_OK) {
        boot_status(3, "GNSS: FAIL");
        ESP_LOGW(TAG, "GNSS initialization failed (continuing without GNSS)");
        g_gnss_available = false;
    } else {
        boot_status(3, "GNSS: OK");
        ESP_LOGI(TAG, "GNSS initialized on UART2 (TX=17, RX=16)");
        g_gnss_available = true;
    }

    // ========== PHASE 5.5: BLE ==========
    boot_status(3, "BLE...");
    if (ble_manager_init() != ESP_OK) {
        boot_status(3, "BLE: FAIL");
        ESP_LOGW(TAG, "BLE initialization failed (continuing without BLE)");
    } else {
        boot_status(3, "BLE: OK");
        ESP_LOGI(TAG, "BLE initialized - advertising as 'TestAPEN'");
    }

    // ========== PHASE 6: Summary Screen ==========
    if (g_display_available) {
        ssd1306_clear();
        ssd1306_printf(0, "TestAPEN Master");
        ssd1306_printf(1, "All systems OK");
        char ip[16];
        network_manager_get_ip(ip, sizeof(ip));
        ssd1306_printf(2, "%s", ip);
        ssd1306_printf(3, "Starting...");
        ssd1306_update();
        vTaskDelay(pdMS_TO_TICKS(1500));  // Pause to show summary
    }

    // ========== PHASE 7: Create Tasks ==========
    xTaskCreate(task_espnow, "Task_ESPNOW", TASK_CAN_STACK_SIZE,
                NULL, TASK_CAN_PRIORITY, &h_task_espnow);

    xTaskCreate(task_imu, "Task_IMU", TASK_IMU_STACK_SIZE,
                NULL, TASK_IMU_PRIORITY, &h_task_imu);

    if (g_gnss_available) {
        xTaskCreate(task_gnss, "Task_GNSS", TASK_GNSS_STACK_SIZE,
                    NULL, TASK_GNSS_PRIORITY, &h_task_gnss);
    }

    xTaskCreate(task_autopilot, "Task_Autopilot", TASK_AUTOPILOT_STACK_SIZE,
                NULL, TASK_AUTOPILOT_PRIORITY, &h_task_autopilot);

    xTaskCreate(task_display, "Task_Display", TASK_DISPLAY_STACK_SIZE,
                NULL, TASK_DISPLAY_PRIORITY, &h_task_display);

    xTaskCreate(task_network, "Task_Network", 4096,
                NULL, 1, &h_task_network);

    // Transition from BOOT to IDLE
    state_machine_process(&g_state_machine, EVENT_POST_PASS, NULL);

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  Master Node initialized");
    ESP_LOGI(TAG, "  State: %s", state_to_string(state_machine_get_state(&g_state_machine)));
    ESP_LOGI(TAG, "========================================");
}

#endif // CONFIG_TESTAPEN_NODE_MASTER
