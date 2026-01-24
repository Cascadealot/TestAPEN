/**
 * @file rudder_node.c
 * @brief Rudder Node Implementation for TestAPEN
 *
 * FSD Reference: TestAPEN.FSD.v1.0.0.md Section 6.3
 * Modified for ESP-NOW communication instead of CAN bus.
 *
 * Rudder Node Tasks:
 * - Task_ESPNOW: ESP-NOW message handling (Priority 5)
 * - Task_Rudder: Servo control and motor drive (Priority 4, 50Hz)
 */

#include <math.h>
#include <stdarg.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

#include "autopilot_common.h"
#include "espnow_protocol.h"
#include "state_machine.h"
#include "as5600.h"
#include "ssd1306.h"
#include "network_manager.h"
#include "cmd_console.h"
#include "param_store.h"

#ifdef CONFIG_TESTAPEN_NODE_RUDDER

static const char *TAG = "RUDDER";

// Version
#define FIRMWARE_VERSION "1.1.0-espnow"

/*============================================================================
 * Global State
 *============================================================================*/

static state_machine_t g_state_machine;
static SemaphoreHandle_t g_i2c_mutex = NULL;

// Rudder data
static float g_commanded_angle = 0.0f;
static float g_actual_angle = 0.0f;
static uint32_t g_last_command_time_ms = 0;
static bool g_in_deadband = true;

// Calibration - rudder angle limits (degrees)
static float g_cal_port = -35.0f;
static float g_cal_stbd = 35.0f;

/*============================================================================
 * Multi-Turn Encoder Tracking
 *
 * The AS5600 magnet is on the motor shaft. Lock-to-lock is 2.25 motor turns.
 * Since AS5600 only reads 0-360 (one rotation), we must track rotations
 * to handle the full 810 (2.25 turns) of motor travel.
 *
 * At boot, the rudder is assumed to be centered.
 *============================================================================*/

// Physical constants
#define MOTOR_TURNS_LOCK_TO_LOCK    2.25f    // Motor turns for full rudder travel
#define RUDDER_RANGE_DEGREES        70.0f    // Total rudder range (+/-35)
#define ENCODER_COUNTS_PER_REV      4096     // AS5600 resolution

// Derived constants
// Motor degrees per rudder degree = 810 / 70 = 11.57
#define MOTOR_DEG_PER_RUDDER_DEG    (MOTOR_TURNS_LOCK_TO_LOCK * 360.0f / RUDDER_RANGE_DEGREES)
// Encoder counts from center to each limit = 1.125 turns * 4096 = 4608
#define COUNTS_CENTER_TO_LIMIT      ((int32_t)(MOTOR_TURNS_LOCK_TO_LOCK * ENCODER_COUNTS_PER_REV / 2.0f))

// Multi-turn tracking state
static int32_t g_rotation_count = 0;         // Number of full rotations from boot
static uint16_t g_last_raw = 0;              // Previous raw reading for wrap detection
static int32_t g_center_position = 0;        // Boot position (virtual counts)
static int32_t g_current_position = 0;       // Current position (virtual counts)
static bool g_calibration_valid = true;      // Auto-calibrated at boot
static bool g_as5600_available = false;      // AS5600 initialized successfully

// Motor state
static bool g_motor_enabled = false;
static bool g_motor_running = false;
static int g_motor_direction = 0;  // -1=port, 0=stop, 1=stbd

// Sequence counters
static uint8_t g_heartbeat_seq = 0;

/*============================================================================
 * Safety Monitoring State
 *============================================================================*/

// Master heartbeat monitoring
static uint32_t g_last_master_heartbeat_ms = 0;
static bool g_master_heartbeat_received = false;  // Set to true on first heartbeat

// Safety features removed for debugging

/*============================================================================
 * Task Handles
 *============================================================================*/

static TaskHandle_t h_task_espnow = NULL;
static TaskHandle_t h_task_rudder = NULL;
static TaskHandle_t h_task_network = NULL;
static TaskHandle_t h_task_display = NULL;

// Forward declarations
static void motor_stop(void);
static float position_to_rudder_angle(void);
static void calibration_recenter(void);

/*============================================================================
 * Multi-Turn Encoder Position Tracking
 *============================================================================*/

/**
 * @brief Update multi-turn position from raw encoder reading
 */
static void update_multi_turn_position(uint16_t raw_value) {
    // Detect wrap-around
    int32_t delta = (int32_t)raw_value - (int32_t)g_last_raw;

    if (delta > 2048) {
        // Wrapped backwards (e.g., 100 -> 4000)
        g_rotation_count--;
    } else if (delta < -2048) {
        // Wrapped forwards (e.g., 4000 -> 100)
        g_rotation_count++;
    }

    g_last_raw = raw_value;

    // Calculate virtual position (can be negative or > 4096)
    g_current_position = (g_rotation_count * ENCODER_COUNTS_PER_REV) + raw_value;
}

/**
 * @brief Initialize multi-turn tracking at boot
 */
static void init_multi_turn_tracking(uint16_t raw_value) {
    g_last_raw = raw_value;

    // Check for stored calibration
    float stored_center = param_get(PARAM_CAL_RAW_CENTER);

    if (stored_center >= 0.0f && stored_center <= 4095.0f) {
        // Use stored calibration
        uint16_t cal_center_raw = (uint16_t)stored_center;

        // Detect if we've wrapped around since calibration
        int32_t diff = (int32_t)raw_value - (int32_t)cal_center_raw;

        if (diff < -2048) {
            g_rotation_count = 1;
        } else if (diff > 2048) {
            g_rotation_count = -1;
        } else {
            g_rotation_count = 0;
        }

        g_center_position = cal_center_raw;
        g_current_position = (g_rotation_count * ENCODER_COUNTS_PER_REV) + raw_value;
        g_calibration_valid = true;

        int32_t offset = g_current_position - g_center_position;
        ESP_LOGI(TAG, "Loaded calibration: stored_center=%u, current_raw=%u, rotations=%d, offset=%d",
                 cal_center_raw, raw_value, (int)g_rotation_count, (int)offset);
        ESP_LOGI(TAG, "Estimated rudder position: %.1f deg", position_to_rudder_angle());
    } else {
        // No stored calibration - assume current position is center
        g_rotation_count = 0;
        g_center_position = raw_value;
        g_current_position = raw_value;
        g_calibration_valid = true;

        ESP_LOGI(TAG, "No stored calibration, assuming boot position is center");
    }

    ESP_LOGI(TAG, "Multi-turn tracking initialized: center=%d, limits=+/-%d counts",
             (int)g_center_position, (int)COUNTS_CENTER_TO_LIMIT);
}

/**
 * @brief Convert current multi-turn position to rudder angle
 */
static float position_to_rudder_angle(void) {
    int32_t offset = g_current_position - g_center_position;
    float counts_per_rudder_deg = (float)COUNTS_CENTER_TO_LIMIT / 35.0f;
    float rudder_angle = (float)offset / counts_per_rudder_deg;
    return clampf(rudder_angle, g_cal_port, g_cal_stbd);
}

/**
 * @brief Convert rudder angle to target encoder position
 */
static int32_t rudder_angle_to_position(float rudder_angle) {
    rudder_angle = clampf(rudder_angle, g_cal_port, g_cal_stbd);
    float counts_per_rudder_deg = (float)COUNTS_CENTER_TO_LIMIT / 35.0f;
    return g_center_position + (int32_t)(rudder_angle * counts_per_rudder_deg);
}

/**
 * @brief Re-center calibration at current position
 */
static void calibration_recenter(void) {
    g_center_position = g_current_position;
    g_calibration_valid = true;
    ESP_LOGI(TAG, "Calibration: re-centered at position %d (raw=%u, rotations=%d)",
             (int)g_center_position, g_last_raw, (int)g_rotation_count);
    ESP_LOGI(TAG, "Use 'cal save' to persist this calibration");
}

/*============================================================================
 * Interface Functions (for console)
 *============================================================================*/

system_state_t rudder_get_state(void) {
    return state_machine_get_state(&g_state_machine);
}

uint8_t rudder_get_fault_code(void) {
    return g_state_machine.fault_code;
}

float rudder_get_angle(void) {
    return g_actual_angle;
}

float rudder_get_commanded_angle(void) {
    return g_commanded_angle;
}

uint16_t rudder_get_raw_angle(void) {
    uint16_t raw = 0;
    as5600_read_raw_angle(&raw);
    return raw;
}

bool rudder_get_motor_enabled(void) {
    return g_motor_enabled;
}

bool rudder_get_motor_running(void) {
    return g_motor_running;
}

int rudder_get_motor_direction(void) {
    return g_motor_direction;
}

bool rudder_get_calibration_valid(void) {
    return g_calibration_valid;
}

bool rudder_engage(void) {
    if (state_machine_get_state(&g_state_machine) == STATE_IDLE) {
        engage_preconditions_t pre = {
            .heading_valid = true,
            .rudder_feedback_valid = as5600_is_valid(),
            .calibration_valid = g_calibration_valid,
            .no_active_faults = (g_state_machine.fault_code == 0)
        };
        // Reset command timestamp to prevent immediate timeout fault
        g_last_command_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        state_machine_process(&g_state_machine, EVENT_ENGAGE, &pre);
        return state_machine_get_state(&g_state_machine) == STATE_ENGAGED;
    }
    return false;
}

bool rudder_disengage(void) {
    if (state_machine_get_state(&g_state_machine) == STATE_ENGAGED) {
        state_machine_process(&g_state_machine, EVENT_DISENGAGE, NULL);
        motor_stop();
        return state_machine_get_state(&g_state_machine) == STATE_IDLE;
    }
    return false;
}

bool rudder_clear_fault(void) {
    if (state_machine_get_state(&g_state_machine) == STATE_FAULTED) {
        g_state_machine.fault_code = 0;
        state_machine_process(&g_state_machine, EVENT_FAULT_CLEAR, NULL);
        return state_machine_get_state(&g_state_machine) == STATE_IDLE;
    }
    return false;
}

bool rudder_enter_calibration(void) {
    if (state_machine_get_state(&g_state_machine) == STATE_IDLE) {
        state_machine_process(&g_state_machine, EVENT_CAL_ENTER, NULL);
        return state_machine_get_state(&g_state_machine) == STATE_CALIBRATION;
    }
    return false;
}

bool rudder_exit_calibration(void) {
    if (state_machine_get_state(&g_state_machine) == STATE_CALIBRATION) {
        state_machine_process(&g_state_machine, EVENT_CAL_EXIT, NULL);
        return state_machine_get_state(&g_state_machine) == STATE_IDLE;
    }
    return false;
}

void rudder_calibrate_center(void) {
    calibration_recenter();
    ESP_LOGI(TAG, "Calibration: center set at current position");
}

void rudder_calibrate_port(void) {
    ESP_LOGI(TAG, "Port limit: %.1f deg (fixed by geometry)", g_cal_port);
}

void rudder_calibrate_starboard(void) {
    ESP_LOGI(TAG, "Starboard limit: %.1f deg (fixed by geometry)", g_cal_stbd);
}

bool rudder_calibrate_save(void) {
    uint16_t current_raw = 0;
    if (as5600_read_raw_angle(&current_raw) != ESP_OK) {
        ESP_LOGE(TAG, "Calibration save failed: cannot read encoder");
        return false;
    }

    esp_err_t err = param_set(PARAM_CAL_RAW_CENTER, (float)current_raw);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set calibration parameter");
        return false;
    }

    err = param_save(PARAM_CAL_RAW_CENTER);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save calibration to NVS");
        return false;
    }

    ESP_LOGI(TAG, "Calibration saved: center_raw=%u, center_pos=%d",
             current_raw, (int)g_center_position);
    return true;
}

void rudder_set_commanded_angle(float angle) {
    g_commanded_angle = clampf(angle, g_cal_port, g_cal_stbd);
    g_last_command_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    ESP_LOGI(TAG, "Console set rudder command: %.1f deg", g_commanded_angle);
}

bool rudder_is_wifi_connected(void) {
    return network_manager_is_connected();
}

void rudder_get_ip_address(char *buf, size_t buf_size) {
    network_manager_get_ip(buf, buf_size);
}

const char* rudder_get_version(void) {
    return FIRMWARE_VERSION;
}

/*============================================================================
 * Motor Control
 *============================================================================*/

static void motor_init(void) {
    // Configure PWM (LEDC)
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t channel_conf = {
        .gpio_num = CONFIG_TESTAPEN_MOTOR_PWM_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel_conf);

    // Configure direction GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << CONFIG_TESTAPEN_MOTOR_DIR_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Set initial state
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    gpio_set_level(CONFIG_TESTAPEN_MOTOR_DIR_GPIO, 0);

    ESP_LOGI(TAG, "Motor initialized: PWM=%d, DIR=%d",
             CONFIG_TESTAPEN_MOTOR_PWM_GPIO, CONFIG_TESTAPEN_MOTOR_DIR_GPIO);
}

static void motor_stop(void) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    g_motor_running = false;
    g_motor_direction = 0;
}

static void motor_drive(int speed_percent, int direction) {
    if (!state_machine_motor_allowed(&g_state_machine)) {
        motor_stop();
        return;
    }

    // Clamp speed using runtime parameters
    int min_speed = (int)param_get(PARAM_MIN_MOTOR_SPEED);
    int max_speed = (int)param_get(PARAM_MAX_MOTOR_SPEED);
    if (speed_percent < min_speed) speed_percent = 0;
    if (speed_percent > max_speed) speed_percent = max_speed;

    // Convert to duty cycle (8-bit)
    uint32_t duty = (speed_percent * 255) / 100;

    // Set direction (DIR LOW = port, DIR HIGH = stbd)
    gpio_set_level(CONFIG_TESTAPEN_MOTOR_DIR_GPIO, direction > 0 ? 1 : 0);

    // Set PWM duty
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    g_motor_running = (speed_percent > 0);
    g_motor_direction = direction;
}

/*============================================================================
 * ESP-NOW Receive Callback
 *============================================================================*/

static void espnow_message_handler(uint8_t msg_type, const uint8_t *data,
                                    size_t len, const uint8_t *src_mac) {
    char mac_str[18];
    espnow_mac_to_str(src_mac, mac_str);

    switch (msg_type) {
        case MSG_RUDDER_COMMAND: {
            if (len < sizeof(espnow_rudder_command_t)) break;
            const espnow_rudder_command_t *cmd = (const espnow_rudder_command_t *)data;
            g_commanded_angle = be_to_int16((const uint8_t*)&cmd->cmd_angle_x10) / 10.0f;
            g_last_command_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            ESP_LOGD(TAG, "Rudder command: %.1f", g_commanded_angle);
            break;
        }

        case MSG_MASTER_HEARTBEAT:
            // Track master heartbeat for timeout detection
            g_last_master_heartbeat_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            g_master_heartbeat_received = true;
            break;

        case MSG_SYSTEM_COMMAND: {
            if (len < 1) break;
            uint8_t sys_cmd = data[0];
            switch (sys_cmd) {
                case SYS_CMD_ENGAGE: {
                    engage_preconditions_t pre = {
                        .heading_valid = true,
                        .rudder_feedback_valid = as5600_is_valid(),
                        .calibration_valid = g_calibration_valid,
                        .no_active_faults = (g_state_machine.fault_code == 0)
                    };
                    // Reset command timestamp to prevent immediate timeout fault
                    g_last_command_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    state_machine_process(&g_state_machine, EVENT_ENGAGE, &pre);
                    ESP_LOGI(TAG, "ENGAGE command received via ESP-NOW");
                    break;
                }
                case SYS_CMD_DISENGAGE:
                    state_machine_process(&g_state_machine, EVENT_DISENGAGE, NULL);
                    motor_stop();
                    break;
                case SYS_CMD_CAL_ENTER:
                    state_machine_process(&g_state_machine, EVENT_CAL_ENTER, NULL);
                    ESP_LOGI(TAG, "Entering calibration mode");
                    break;
                case SYS_CMD_CAL_EXIT:
                    state_machine_process(&g_state_machine, EVENT_CAL_EXIT, NULL);
                    ESP_LOGI(TAG, "Exiting calibration mode");
                    break;
                case SYS_CMD_FAULT_CLEAR:
                    g_state_machine.fault_code = 0;
                    state_machine_process(&g_state_machine, EVENT_FAULT_CLEAR, NULL);
                    break;
            }
            break;
        }

        case MSG_CALIBRATION_CMD: {
            if (state_machine_get_state(&g_state_machine) != STATE_CALIBRATION) {
                ESP_LOGW(TAG, "Calibration command ignored - not in calibration mode");
                break;
            }
            if (len < 1) break;
            uint8_t cal_cmd = data[0];
            switch (cal_cmd) {
                case CAL_CMD_CENTER:
                    calibration_recenter();
                    break;
                case CAL_CMD_PORT:
                    ESP_LOGI(TAG, "Port limit: %.1f deg (fixed)", g_cal_port);
                    break;
                case CAL_CMD_STARBOARD:
                    ESP_LOGI(TAG, "Starboard limit: %.1f deg (fixed)", g_cal_stbd);
                    break;
                case CAL_CMD_SAVE:
                    if (rudder_calibrate_save()) {
                        ESP_LOGI(TAG, "Calibration saved: center=%d", (int)g_center_position);
                    } else {
                        ESP_LOGE(TAG, "Calibration save FAILED");
                    }
                    break;
            }
            break;
        }

        case MSG_E_STOP:
            ESP_LOGW(TAG, "E_STOP received from %s!", mac_str);
            motor_stop();
            state_machine_set_fault(&g_state_machine, ERR_UNKNOWN);
            break;

        case MSG_PARAM_CONFIG: {
            if (len < sizeof(espnow_param_config_t)) break;
            const espnow_param_config_t *cfg = (const espnow_param_config_t *)data;
            param_id_t param_id = (param_id_t)cfg->param_id;

            // Only accept Rudder parameters (not Master params)
            if (param_is_local(param_id)) {
                esp_err_t err = param_set(param_id, cfg->value);
                if (err == ESP_OK) {
                    const param_meta_t *meta = param_get_meta(param_id);
                    ESP_LOGI(TAG, "ESP-NOW param update: %s = %.3f",
                             meta ? meta->name : "?", cfg->value);
                    if (cfg->flags & PARAM_FLAG_SAVE_NVS) {
                        param_save(param_id);
                    }
                } else {
                    ESP_LOGW(TAG, "ESP-NOW param update failed: id=%d, value=%.3f",
                             param_id, cfg->value);
                }
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
        // Send heartbeat at 50 Hz (20ms interval)
        if ((xTaskGetTickCount() - last_heartbeat) >= pdMS_TO_TICKS(RUDDER_HEARTBEAT_INTERVAL_MS)) {
            uint8_t motor_status = 0;
            if (g_motor_enabled) motor_status |= MOTOR_FLAG_ENABLED;
            if (g_motor_running) motor_status |= MOTOR_FLAG_RUNNING;
            if (g_motor_direction > 0) motor_status |= MOTOR_FLAG_DIRECTION;
            if (g_in_deadband) motor_status |= MOTOR_FLAG_IN_DEADBAND;

            espnow_rudder_heartbeat_t hb = {
                .state = (uint8_t)state_machine_get_state(&g_state_machine),
                .fault_code = g_state_machine.fault_code,
                .motor_status = motor_status,
                .sequence = g_heartbeat_seq++,
                .reserved = 0
            };

            int16_to_be((int16_t)(g_actual_angle * 10), (uint8_t*)&hb.angle_x10);

            // Broadcast heartbeat to all nodes
            espnow_send(MSG_RUDDER_HEARTBEAT, (uint8_t*)&hb, sizeof(hb), NULL);
            last_heartbeat = xTaskGetTickCount();
        }

        // ESP-NOW receive is handled via callback, just yield here
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/*============================================================================
 * Task: Rudder Servo Control
 *============================================================================*/

static void task_rudder(void *pvParameters) {
    ESP_LOGI(TAG, "Task_Rudder started");
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / RATE_RUDDER_HZ);
    const float dt = 1.0f / RATE_RUDDER_HZ;

    static float last_speed = 0.0f;

    while (1) {
        // Only read encoder if AS5600 was initialized successfully
        if (g_as5600_available) {
            as5600_data_t encoder_data;
            esp_err_t enc_err = as5600_update(&encoder_data);
            if (enc_err == ESP_OK && encoder_data.valid) {
                update_multi_turn_position(encoder_data.raw_angle);
                g_actual_angle = position_to_rudder_angle();
            } else if (enc_err != ESP_OK) {
                ESP_LOGE(TAG, "AS5600 read failed: %s", esp_err_to_name(enc_err));
                if (!encoder_data.status.magnet_detected) {
                    state_machine_set_fault(&g_state_machine, ERR_SENSOR_FAULT);
                    motor_stop();
                }
            }
        }

        // Servo control
        if (state_machine_get_state(&g_state_machine) == STATE_ENGAGED) {
            float error = g_commanded_angle - g_actual_angle;

            // Get runtime servo parameters
            float db_enter = param_get(PARAM_DEADBAND_ENTER);
            float db_exit = param_get(PARAM_DEADBAND_EXIT);
            float kp_servo = param_get(PARAM_KP_SERVO);
            float slew_rate = param_get(PARAM_RUDDER_SLEW_RATE);

            // Hysteresis deadband
            if (g_in_deadband && fabsf(error) > db_exit) {
                g_in_deadband = false;
            }
            if (!g_in_deadband && fabsf(error) < db_enter) {
                g_in_deadband = true;
                motor_stop();
            }

            if (!g_in_deadband) {
                // Slew-limited proportional
                float speed = kp_servo * error;
                float max_delta = slew_rate * dt;
                speed = clamp_delta(speed, last_speed, max_delta);
                last_speed = speed;

                int speed_percent = (int)fabsf(speed);
                int direction = signf(error);
                motor_drive(speed_percent, direction);
            }
        } else {
            motor_stop();
            last_speed = 0.0f;
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

        ssd1306_clear();

        // Line 0: State
        if (state == STATE_FAULTED) {
            ssd1306_printf(0, "FAULT: %d", g_state_machine.fault_code);
        } else {
            ssd1306_printf(0, "%-16s", state_to_string(state));
        }

        // Line 1: Actual rudder angle
        ssd1306_printf(1, "RUD: %+6.1f", g_actual_angle);

        // Line 2: Commanded angle (if engaged)
        if (state == STATE_ENGAGED) {
            float error = g_commanded_angle - g_actual_angle;
            ssd1306_printf(2, "CMD: %+5.1f E%+.0f", g_commanded_angle, error);
        } else {
            ssd1306_printf(2, "CMD: ---.-");
        }

        // Line 3: Motor status or IP
        if (g_motor_running) {
            if (g_motor_direction > 0) {
                ssd1306_printf(3, "STBD >>>");
            } else if (g_motor_direction < 0) {
                ssd1306_printf(3, "<<< PORT");
            } else {
                ssd1306_printf(3, "HOLDING");
            }
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
 * Rudder Node Initialization
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

void rudder_node_init(void) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  TestAPEN Rudder Node");
    ESP_LOGI(TAG, "  Version: %s", FIRMWARE_VERSION);
    ESP_LOGI(TAG, "========================================");

    // ========== PHASE 1: I2C and Display (FIRST) ==========
    g_i2c_mutex = xSemaphoreCreateMutex();
    if (g_i2c_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create I2C mutex!");
        return;
    }

    // Initialize I2C
    if (i2c_init() != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed!");
        return;
    }

    // Initialize display FIRST so we can show boot progress
    esp_err_t disp_err = ssd1306_init(I2C_NUM_0, g_i2c_mutex, 32);
    if (disp_err == ESP_OK) {
        g_display_available = true;
        ssd1306_clear();
        ssd1306_printf(0, "TestAPEN Rudder");
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
    }

    // ========== PHASE 3: Network (takes longest) ==========
    boot_status(2, "WiFi...");
    esp_err_t net_err = network_manager_init(
        CONFIG_TESTAPEN_WIFI_SSID,
        CONFIG_TESTAPEN_WIFI_PASSWORD,
        "testapen-rudder",
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

    // Initialize console (critical for debug access)
    console_init();
    network_manager_set_command_callback(console_process_command);
    ESP_LOGI(TAG, "Console initialized - telnet debug available");

    // ========== PHASE 4: ESP-NOW (before sensors - always available) ==========
    boot_status(3, "ESP-NOW...");
    if (espnow_init() != ESP_OK) {
        boot_status(3, "ESP-NOW: FAIL");
        ESP_LOGE(TAG, "ESP-NOW initialization failed!");
    } else {
        boot_status(3, "ESP-NOW: OK");
        ESP_LOGI(TAG, "ESP-NOW initialized");
        espnow_register_recv_callback(espnow_message_handler);
    }

    // ========== PHASE 5: Motor Driver (before sensor - can init without feedback) ==========
    boot_status(3, "Motor...");
    motor_init();
    // Note: g_motor_enabled set after AS5600 check - motor won't run without valid sensor
    boot_status(3, "Motor: OK");

    // ========== PHASE 6: AS5600 Encoder (last hardware - failures don't block OTA/debug) ==========
    boot_status(3, "AS5600...");
    esp_err_t as5600_err = as5600_init(I2C_NUM_0, g_i2c_mutex);
    if (as5600_err != ESP_OK) {
        boot_status(3, "AS5600: FAIL");
        ESP_LOGE(TAG, "AS5600 initialization failed: %s", esp_err_to_name(as5600_err));
        state_machine_set_fault(&g_state_machine, ERR_SENSOR_INIT);
        g_as5600_available = false;
        // Continue - OTA and debug still available
    } else {
        boot_status(3, "AS5600: OK");
        ESP_LOGI(TAG, "AS5600 encoder initialized");
        g_as5600_available = true;

        // Initialize multi-turn tracking (only if AS5600 available)
        uint16_t initial_raw = 0;
        if (as5600_read_raw_angle(&initial_raw) == ESP_OK) {
            init_multi_turn_tracking(initial_raw);
            ESP_LOGI(TAG, "Auto-calibration: boot position=%u assumed center", initial_raw);
            g_motor_enabled = true;  // Only enable motor if sensor is working
        } else {
            ESP_LOGE(TAG, "Failed to read initial encoder position!");
            state_machine_set_fault(&g_state_machine, ERR_SENSOR_FAULT);
            g_as5600_available = false;
        }
    }

    // ========== PHASE 8: Summary Screen ==========
    if (g_display_available) {
        ssd1306_clear();
        ssd1306_printf(0, "TestAPEN Rudder");
        if (g_state_machine.fault_code != 0) {
            ssd1306_printf(1, "FAULT: %d", g_state_machine.fault_code);
        } else {
            ssd1306_printf(1, "All systems OK");
        }
        char ip[16];
        network_manager_get_ip(ip, sizeof(ip));
        ssd1306_printf(2, "%s", ip);
        ssd1306_printf(3, "OTA ready");
        ssd1306_update();
        vTaskDelay(pdMS_TO_TICKS(1500));
    }

    // ========== PHASE 9: Create Tasks (always, even if faulted) ==========
    xTaskCreate(task_espnow, "Task_ESPNOW", TASK_CAN_STACK_SIZE,
                NULL, TASK_CAN_PRIORITY, &h_task_espnow);

    xTaskCreate(task_rudder, "Task_Rudder", TASK_RUDDER_STACK_SIZE,
                NULL, TASK_RUDDER_PRIORITY, &h_task_rudder);

    xTaskCreate(task_network, "Task_Network", 4096,
                NULL, 1, &h_task_network);

    xTaskCreate(task_display, "Task_Display", TASK_DISPLAY_STACK_SIZE,
                NULL, TASK_DISPLAY_PRIORITY, &h_task_display);

    // Transition from BOOT to IDLE
    state_machine_process(&g_state_machine, EVENT_POST_PASS, NULL);

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  Rudder Node initialized");
    ESP_LOGI(TAG, "  State: %s", state_to_string(state_machine_get_state(&g_state_machine)));
    ESP_LOGI(TAG, "========================================");
}

#endif // CONFIG_TESTAPEN_NODE_RUDDER
