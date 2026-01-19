/**
 * @file console.c
 * @brief Debug Console Command Handler Implementation
 *
 * Supports both Master and Rudder nodes with node-specific commands.
 */

#include "cmd_console.h"
#include "state_machine.h"
#include "param_store.h"
#include "espnow_protocol.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "sdkconfig.h"

#ifdef CONFIG_TESTAP2_NODE_MASTER
// Forward declarations for master interface functions (master_node.c)
extern system_state_t master_get_state(void);
extern uint8_t master_get_fault_code(void);
extern bool master_engage(void);
extern bool master_disengage(void);
extern bool master_clear_fault(void);
extern float master_get_heading_filtered(void);
extern float master_get_heading_raw(void);
extern float master_get_target_heading(void);
extern float master_get_heading_error(void);
extern void master_set_target_heading(float heading);
extern void master_adjust_target_heading(float delta);
extern float master_get_yaw_rate(void);
extern float master_get_roll(void);
extern float master_get_pitch(void);
extern void master_set_heading_simulation(bool enable);
extern bool master_is_heading_simulation(void);
extern void master_set_simulated_heading(float heading);
extern void master_set_mag_cal(float x, float y, float z);
extern void master_get_mag_cal(float *x, float *y, float *z);
// IMU calibration functions
extern esp_err_t master_magcal_start(void);
extern esp_err_t master_magcal_stop(void);
extern bool master_magcal_is_running(void);
extern size_t master_magcal_get_status(char *buf, size_t size);
extern esp_err_t master_magcal_save(void);
extern esp_err_t master_accel_cal_level(void);
extern void master_get_accel_cal(float *x, float *y, float *z);
extern esp_err_t master_gyro_cal(void);
extern void master_get_gyro_bias(float *x, float *y, float *z);
// Sensor fusion functions
extern void master_set_fusion_enabled(bool enable);
extern bool master_is_fusion_enabled(void);
extern void master_set_fusion_beta(float beta);
extern float master_get_fusion_beta(void);
extern bool master_is_fusion_converged(void);
// GPS-aided calibration
extern float master_get_heading_offset(void);
extern uint32_t master_get_gps_sample_count(void);
extern void master_reset_gps_offset(void);
extern esp_err_t master_apply_gps_offset(void);
extern bool master_is_wifi_connected(void);
extern void master_get_ip_address(char *buf, size_t buf_size);
extern const char* master_get_version(void);
// GNSS functions
extern bool master_gnss_available(void);
extern bool master_gnss_has_fix(void);
extern bool master_gnss_cog_valid(void);
extern float master_gnss_get_cog(void);
extern float master_gnss_get_speed(void);
extern int master_gnss_get_position(float *lat, float *lon);
extern float master_get_blended_heading(void);
#endif

#ifdef CONFIG_TESTAP2_NODE_RUDDER
// Forward declarations for rudder interface functions (rudder_node.c)
extern system_state_t rudder_get_state(void);
extern uint8_t rudder_get_fault_code(void);
extern float rudder_get_angle(void);
extern float rudder_get_commanded_angle(void);
extern uint16_t rudder_get_raw_angle(void);
extern bool rudder_get_motor_enabled(void);
extern bool rudder_get_motor_running(void);
extern int rudder_get_motor_direction(void);
extern bool rudder_get_calibration_valid(void);
extern bool rudder_engage(void);
extern bool rudder_disengage(void);
extern bool rudder_clear_fault(void);
extern bool rudder_enter_calibration(void);
extern bool rudder_exit_calibration(void);
extern void rudder_calibrate_center(void);
extern void rudder_calibrate_port(void);
extern void rudder_calibrate_starboard(void);
extern bool rudder_calibrate_save(void);
extern void rudder_set_commanded_angle(float angle);
extern bool rudder_is_wifi_connected(void);
extern void rudder_get_ip_address(char *buf, size_t buf_size);
extern const char* rudder_get_version(void);
#endif

#ifdef CONFIG_TESTAP2_NODE_UI
// Forward declarations for UI node interface functions (ui_node.c)
extern uint8_t ui_get_state(void);
extern uint8_t ui_get_fault_code(void);
extern float ui_get_heading(void);
extern float ui_get_target_heading(void);
extern float ui_get_rudder_angle(void);
extern bool ui_get_master_connected(void);
extern bool ui_get_rudder_connected(void);
extern void ui_set_page(int page);
extern bool ui_is_wifi_connected(void);
extern void ui_get_ip_address(char *buf, size_t buf_size);
extern const char* ui_get_version(void);
extern bool ui_is_epaper_initialized(void);
extern void ui_force_display_refresh(void);
extern size_t ui_get_button_states(char *buf, size_t size);
// e-Paper functions
extern bool epaper_is_busy(void);
#endif

static const char *TAG = "CONSOLE";

static bool g_reboot_pending = false;
static TickType_t g_reboot_time = 0;

/*============================================================================
 * Master Node Command Handlers
 *============================================================================*/

#ifdef CONFIG_TESTAP2_NODE_MASTER

static size_t cmd_help(char *response, size_t size) {
    return snprintf(response, size,
        "TestAPEN Master Console Commands:\r\n"
        "  status        - Full system status\r\n"
        "  state         - State machine state\r\n"
        "  heading       - Heading information\r\n"
        "  imu           - IMU sensor data\r\n"
        "  engage        - Engage autopilot\r\n"
        "  disengage     - Disengage autopilot\r\n"
        "  set heading N - Set target heading to N degrees\r\n"
        "  adjust N      - Adjust target heading by N degrees\r\n"
        "  heading sim N - Simulate heading N degrees\r\n"
        "  heading real  - Use real IMU heading\r\n"
        "  magcal get    - Get magnetometer calibration\r\n"
        "  magcal set X Y Z - Set magnetometer offsets\r\n"
        "  magcal start  - Start guided mag calibration\r\n"
        "  magcal stop   - Stop and compute calibration\r\n"
        "  magcal status - Show calibration status\r\n"
        "  magcal save   - Save mag cal to NVS\r\n"
        "  accelcal      - Calibrate accelerometer (level)\r\n"
        "  gyrocal       - Calibrate gyroscope (stationary)\r\n"
        "  fusion        - Show sensor fusion status\r\n"
        "  fusion on/off - Enable/disable sensor fusion\r\n"
        "  fusion beta N - Set filter beta (0.01-1.0)\r\n"
        "  gpsoffset     - Show GPS heading offset\r\n"
        "  gpsoffset apply - Apply GPS-derived correction\r\n"
        "  gpsoffset reset - Reset GPS offset tracking\r\n"
        "  espnow        - Show ESP-NOW status\r\n"
        "  pid           - Show PID parameters\r\n"
        "  pid Kp Ki Kd  - Set PID parameters\r\n"
        "  param list    - List all parameters\r\n"
        "  param set N V - Set parameter N to value V\r\n"
        "  param save    - Save parameters to NVS\r\n"
        "  param reset   - Reset to defaults\r\n"
        "  gnss          - GNSS status and position\r\n"
        "  fault clear   - Clear fault state\r\n"
        "  version       - Show version info\r\n"
        "  reboot        - Reboot device\r\n"
        "  help          - Show this help\r\n"
    );
}

static size_t cmd_status(char *response, size_t size) {
    char ip[32];
    master_get_ip_address(ip, sizeof(ip));

    return snprintf(response, size,
        "=== TestAPEN Master Status ===\r\n"
        "State: %s\r\n"
        "Fault: %d\r\n"
        "Heading: %.1f deg (raw: %.1f)\r\n"
        "Target: %.1f deg\r\n"
        "Error: %.1f deg\r\n"
        "Yaw Rate: %.1f deg/s\r\n"
        "Roll: %.1f, Pitch: %.1f\r\n"
        "Simulation: %s\r\n"
        "WiFi: %s (%s)\r\n",
        state_to_string(master_get_state()),
        master_get_fault_code(),
        master_get_heading_filtered(),
        master_get_heading_raw(),
        master_get_target_heading(),
        master_get_heading_error(),
        master_get_yaw_rate(),
        master_get_roll(),
        master_get_pitch(),
        master_is_heading_simulation() ? "ON" : "OFF",
        master_is_wifi_connected() ? "Connected" : "Disconnected",
        ip
    );
}

static size_t cmd_state(char *response, size_t size) {
    return snprintf(response, size,
        "State: %s\r\n"
        "Fault Code: %d\r\n",
        state_to_string(master_get_state()),
        master_get_fault_code()
    );
}

static size_t cmd_heading(char *response, size_t size) {
    return snprintf(response, size,
        "Heading (filtered): %.1f deg\r\n"
        "Heading (raw): %.1f deg\r\n"
        "Target: %.1f deg\r\n"
        "Error: %.1f deg\r\n"
        "Simulation: %s\r\n",
        master_get_heading_filtered(),
        master_get_heading_raw(),
        master_get_target_heading(),
        master_get_heading_error(),
        master_is_heading_simulation() ? "ON" : "OFF"
    );
}

static size_t cmd_imu(char *response, size_t size) {
    float mx, my, mz;
    master_get_mag_cal(&mx, &my, &mz);

    return snprintf(response, size,
        "Heading: %.1f deg\r\n"
        "Yaw Rate: %.1f deg/s\r\n"
        "Roll: %.1f deg\r\n"
        "Pitch: %.1f deg\r\n"
        "Mag Cal: X=%.1f Y=%.1f Z=%.1f\r\n",
        master_get_heading_filtered(),
        master_get_yaw_rate(),
        master_get_roll(),
        master_get_pitch(),
        mx, my, mz
    );
}

#include "gnss_driver.h"

static size_t cmd_gnss(char *response, size_t size) {
    if (!master_gnss_available()) {
        return snprintf(response, size, "GNSS: Not available\r\n");
    }

    float lat = 0, lon = 0;
    bool has_pos = (master_gnss_get_position(&lat, &lon) == 0);

    return snprintf(response, size,
        "=== GNSS Status ===\r\n"
        "Available: Yes\r\n"
        "Fix: %s\r\n"
        "Position: %s%.6f, %.6f\r\n"
        "Speed: %.1f kts\r\n"
        "COG: %.1f deg (%s)\r\n"
        "Blended Heading: %.1f deg\r\n"
        "--- Debug ---\r\n"
        "Bytes RX: %lu\r\n"
        "Messages: %lu\r\n"
        "Errors: %lu\r\n",
        master_gnss_has_fix() ? "Yes" : "No",
        has_pos ? "" : "N/A ",
        lat, lon,
        master_gnss_get_speed(),
        master_gnss_get_cog(),
        master_gnss_cog_valid() ? "valid" : "invalid",
        master_get_blended_heading(),
        gnss_get_bytes_received(),
        gnss_get_message_count(),
        gnss_get_error_count()
    );
}

static size_t cmd_engage(char *response, size_t size) {
    if (master_engage()) {
        return snprintf(response, size,
            "Autopilot ENGAGED\r\n"
            "Target heading: %.1f deg\r\n",
            master_get_target_heading()
        );
    } else {
        return snprintf(response, size,
            "Engage FAILED (state=%s)\r\n",
            state_to_string(master_get_state())
        );
    }
}

static size_t cmd_disengage(char *response, size_t size) {
    if (master_disengage()) {
        return snprintf(response, size, "Autopilot DISENGAGED\r\n");
    } else {
        return snprintf(response, size,
            "Disengage FAILED (state=%s)\r\n",
            state_to_string(master_get_state())
        );
    }
}

static size_t cmd_set_heading(const char *args, char *response, size_t size) {
    float heading = atof(args);
    if (heading < 0 || heading >= 360) {
        return snprintf(response, size, "Invalid heading (must be 0-359.9)\r\n");
    }
    master_set_target_heading(heading);
    return snprintf(response, size, "Target heading set to %.1f deg\r\n", heading);
}

static size_t cmd_adjust(const char *args, char *response, size_t size) {
    float delta = atof(args);
    master_adjust_target_heading(delta);
    return snprintf(response, size,
        "Target adjusted by %.1f deg\r\n"
        "New target: %.1f deg\r\n",
        delta, master_get_target_heading()
    );
}

static size_t cmd_heading_sim(const char *args, char *response, size_t size) {
    float heading = atof(args);
    master_set_simulated_heading(heading);
    master_set_heading_simulation(true);
    return snprintf(response, size,
        "Simulation mode ENABLED\r\n"
        "Simulated heading: %.1f deg\r\n",
        heading
    );
}

static size_t cmd_heading_real(char *response, size_t size) {
    master_set_heading_simulation(false);
    return snprintf(response, size, "Simulation mode DISABLED - using real IMU\r\n");
}

static size_t cmd_magcal_get(char *response, size_t size) {
    float x, y, z;
    master_get_mag_cal(&x, &y, &z);
    return snprintf(response, size,
        "Magnetometer Calibration:\r\n"
        "  X offset: %.2f uT\r\n"
        "  Y offset: %.2f uT\r\n"
        "  Z offset: %.2f uT\r\n",
        x, y, z
    );
}

static size_t cmd_magcal_set(const char *args, char *response, size_t size) {
    float x, y, z;
    if (sscanf(args, "%f %f %f", &x, &y, &z) != 3) {
        return snprintf(response, size, "Usage: magcal set X Y Z\r\n");
    }
    master_set_mag_cal(x, y, z);
    return snprintf(response, size,
        "Magnetometer calibration set:\r\n"
        "  X=%.2f Y=%.2f Z=%.2f\r\n",
        x, y, z
    );
}

static size_t cmd_magcal_start(char *response, size_t size) {
    esp_err_t err = master_magcal_start();
    if (err == ESP_OK) {
        return snprintf(response, size,
            "Magnetometer calibration STARTED\r\n"
            "Rotate device in figure-8 pattern for 30-60 seconds\r\n"
            "Use 'magcal status' to monitor, 'magcal stop' when done\r\n");
    } else {
        return snprintf(response, size, "Failed to start calibration\r\n");
    }
}

static size_t cmd_magcal_stop(char *response, size_t size) {
    esp_err_t err = master_magcal_stop();
    if (err == ESP_OK) {
        return master_magcal_get_status(response, size);
    } else {
        return snprintf(response, size,
            "Calibration FAILED\r\n"
            "Need more samples or rotation coverage\r\n"
            "Try rotating device more during calibration\r\n");
    }
}

static size_t cmd_magcal_status(char *response, size_t size) {
    return master_magcal_get_status(response, size);
}

static size_t cmd_magcal_save(char *response, size_t size) {
    esp_err_t err = master_magcal_save();
    if (err == ESP_OK) {
        return snprintf(response, size, "Magnetometer calibration saved to NVS\r\n");
    } else {
        return snprintf(response, size, "Failed to save calibration\r\n");
    }
}

static size_t cmd_accelcal(char *response, size_t size) {
    size_t len = snprintf(response, size,
        "Starting accelerometer calibration...\r\n"
        "Keep device STATIONARY and LEVEL!\r\n");

    esp_err_t err = master_accel_cal_level();
    if (err == ESP_OK) {
        float x, y, z;
        master_get_accel_cal(&x, &y, &z);
        len += snprintf(response + len, size - len,
            "Calibration COMPLETE\r\n"
            "Offset: [%.4f, %.4f, %.4f] g\r\n"
            "Use 'param save' to persist\r\n",
            x, y, z);
    } else {
        len += snprintf(response + len, size - len, "Calibration FAILED\r\n");
    }
    return len;
}

static size_t cmd_gyrocal(char *response, size_t size) {
    size_t len = snprintf(response, size,
        "Starting gyroscope calibration...\r\n"
        "Keep device STATIONARY!\r\n");

    esp_err_t err = master_gyro_cal();
    if (err == ESP_OK) {
        float x, y, z;
        master_get_gyro_bias(&x, &y, &z);
        len += snprintf(response + len, size - len,
            "Calibration COMPLETE\r\n"
            "Bias: [%.3f, %.3f, %.3f] deg/s\r\n",
            x, y, z);
    } else {
        len += snprintf(response + len, size - len, "Calibration FAILED\r\n");
    }
    return len;
}

static size_t cmd_fusion(const char *args, char *response, size_t size) {
    // No args - show status
    if (args == NULL || *args == '\0') {
        return snprintf(response, size,
            "Sensor Fusion (Madgwick AHRS):\r\n"
            "  Enabled: %s\r\n"
            "  Beta: %.3f\r\n"
            "  Converged: %s\r\n",
            master_is_fusion_enabled() ? "YES" : "NO",
            master_get_fusion_beta(),
            master_is_fusion_converged() ? "YES" : "NO");
    }

    // Parse subcommand
    if (strcmp(args, "on") == 0) {
        master_set_fusion_enabled(true);
        return snprintf(response, size, "Sensor fusion ENABLED\r\n");
    }
    else if (strcmp(args, "off") == 0) {
        master_set_fusion_enabled(false);
        return snprintf(response, size, "Sensor fusion DISABLED\r\n");
    }
    else if (strncmp(args, "beta ", 5) == 0) {
        float beta = atof(args + 5);
        if (beta >= 0.01f && beta <= 1.0f) {
            master_set_fusion_beta(beta);
            return snprintf(response, size, "Filter beta set to %.3f\r\n", beta);
        } else {
            return snprintf(response, size, "Invalid beta (must be 0.01-1.0)\r\n");
        }
    }

    return snprintf(response, size, "Usage: fusion [on|off|beta N]\r\n");
}

static size_t cmd_gpsoffset(const char *args, char *response, size_t size) {
    // No args - show status
    if (args == NULL || *args == '\0') {
        float offset = master_get_heading_offset();
        uint32_t count = master_get_gps_sample_count();

        if (count < 10) {
            return snprintf(response, size,
                "GPS Heading Offset Monitor:\r\n"
                "  Samples: %lu (need 10+ for estimate)\r\n"
                "  Status: Collecting data...\r\n"
                "Move at >3 knots for GPS COG samples\r\n",
                count);
        } else {
            return snprintf(response, size,
                "GPS Heading Offset Monitor:\r\n"
                "  Samples: %lu\r\n"
                "  Avg Offset: %.1f deg (mag - gps)\r\n"
                "  Status: %s\r\n"
                "Use 'gpsoffset apply' to correct calibration\r\n",
                count, offset,
                count >= 100 ? "Ready to apply" : "Collecting...");
        }
    }

    // Parse subcommand
    if (strcmp(args, "apply") == 0) {
        esp_err_t err = master_apply_gps_offset();
        if (err == ESP_OK) {
            return snprintf(response, size,
                "GPS-derived offset APPLIED\r\n"
                "Use 'magcal save' to persist to NVS\r\n");
        } else {
            return snprintf(response, size,
                "Apply FAILED\r\n"
                "Need 100+ samples at >3 knots\r\n");
        }
    }
    else if (strcmp(args, "reset") == 0) {
        master_reset_gps_offset();
        return snprintf(response, size, "GPS offset tracking RESET\r\n");
    }

    return snprintf(response, size, "Usage: gpsoffset [apply|reset]\r\n");
}

static size_t cmd_fault_clear(char *response, size_t size) {
    if (master_clear_fault()) {
        return snprintf(response, size, "Fault cleared\r\n");
    } else {
        return snprintf(response, size,
            "Clear fault FAILED (state=%s)\r\n",
            state_to_string(master_get_state())
        );
    }
}

static size_t cmd_version(char *response, size_t size) {
    return snprintf(response, size,
        "TestAPEN Master Node\r\n"
        "Version: %s\r\n"
        "FSD: TestAP2.FSD.v1.0.0\r\n",
        master_get_version()
    );
}

#endif // CONFIG_TESTAP2_NODE_MASTER

/*============================================================================
 * Rudder Node Command Handlers
 *============================================================================*/

#ifdef CONFIG_TESTAP2_NODE_RUDDER

static size_t cmd_help(char *response, size_t size) {
    return snprintf(response, size,
        "TestAPEN Rudder Console Commands:\r\n"
        "  status        - Full system status\r\n"
        "  state         - State machine state\r\n"
        "  rudder        - Rudder angle and motor status\r\n"
        "  motor         - Motor status details\r\n"
        "  servo         - Show servo parameters\r\n"
        "  servo Kp E X  - Set servo Kp, deadband enter/exit\r\n"
        "  motor params  - Show motor parameters\r\n"
        "  motor params M X S - Set min/max speed, slew rate\r\n"
        "  engage        - Engage servo control\r\n"
        "  disengage     - Disengage servo control\r\n"
        "  set rudder N  - Set commanded angle to N degrees\r\n"
        "  cal enter     - Enter calibration mode\r\n"
        "  cal exit      - Exit calibration mode\r\n"
        "  cal center    - Re-center at current position\r\n"
        "  param list    - List all parameters\r\n"
        "  param set N V - Set parameter N to value V\r\n"
        "  param save    - Save parameters to NVS\r\n"
        "  param reset   - Reset to defaults\r\n"
        "  espnow        - Show ESP-NOW status\r\n"
        "  fault clear   - Clear fault state\r\n"
        "  version       - Show version info\r\n"
        "  reboot        - Reboot device\r\n"
        "  help          - Show this help\r\n"
    );
}

static size_t cmd_status(char *response, size_t size) {
    char ip[32];
    rudder_get_ip_address(ip, sizeof(ip));

    const char *dir_str = "STOP";
    int dir = rudder_get_motor_direction();
    if (dir < 0) dir_str = "PORT";
    else if (dir > 0) dir_str = "STBD";

    return snprintf(response, size,
        "=== TestAPEN Rudder Status ===\r\n"
        "State: %s\r\n"
        "Fault: %d\r\n"
        "Rudder: %.1f deg (cmd: %.1f)\r\n"
        "Raw Angle: %u\r\n"
        "Motor: %s %s (%s)\r\n"
        "Calibration: %s\r\n"
        "WiFi: %s (%s)\r\n",
        state_to_string(rudder_get_state()),
        rudder_get_fault_code(),
        rudder_get_angle(),
        rudder_get_commanded_angle(),
        rudder_get_raw_angle(),
        rudder_get_motor_enabled() ? "ENABLED" : "DISABLED",
        rudder_get_motor_running() ? "RUNNING" : "STOPPED",
        dir_str,
        rudder_get_calibration_valid() ? "VALID" : "INVALID",
        rudder_is_wifi_connected() ? "Connected" : "Disconnected",
        ip
    );
}

static size_t cmd_state(char *response, size_t size) {
    return snprintf(response, size,
        "State: %s\r\n"
        "Fault Code: %d\r\n",
        state_to_string(rudder_get_state()),
        rudder_get_fault_code()
    );
}

static size_t cmd_rudder(char *response, size_t size) {
    return snprintf(response, size,
        "Actual Angle: %.1f deg\r\n"
        "Commanded: %.1f deg\r\n"
        "Raw Encoder: %u\r\n"
        "Calibration: %s\r\n",
        rudder_get_angle(),
        rudder_get_commanded_angle(),
        rudder_get_raw_angle(),
        rudder_get_calibration_valid() ? "VALID" : "INVALID"
    );
}

static size_t cmd_motor(char *response, size_t size) {
    const char *dir_str = "STOP";
    int dir = rudder_get_motor_direction();
    if (dir < 0) dir_str = "PORT";
    else if (dir > 0) dir_str = "STBD";

    return snprintf(response, size,
        "Motor Enabled: %s\r\n"
        "Motor Running: %s\r\n"
        "Direction: %s\r\n",
        rudder_get_motor_enabled() ? "YES" : "NO",
        rudder_get_motor_running() ? "YES" : "NO",
        dir_str
    );
}

static size_t cmd_engage(char *response, size_t size) {
    if (rudder_engage()) {
        return snprintf(response, size, "Rudder servo ENGAGED\r\n");
    } else {
        return snprintf(response, size,
            "Engage FAILED (state=%s, cal=%s)\r\n",
            state_to_string(rudder_get_state()),
            rudder_get_calibration_valid() ? "valid" : "invalid"
        );
    }
}

static size_t cmd_disengage(char *response, size_t size) {
    if (rudder_disengage()) {
        return snprintf(response, size, "Rudder servo DISENGAGED\r\n");
    } else {
        return snprintf(response, size,
            "Disengage FAILED (state=%s)\r\n",
            state_to_string(rudder_get_state())
        );
    }
}

static size_t cmd_set_rudder(const char *args, char *response, size_t size) {
    float angle = atof(args);
    if (angle < -35 || angle > 35) {
        return snprintf(response, size, "Invalid angle (must be -35 to +35)\r\n");
    }
    rudder_set_commanded_angle(angle);
    return snprintf(response, size, "Commanded angle set to %.1f deg\r\n", angle);
}

static size_t cmd_cal_enter(char *response, size_t size) {
    if (rudder_enter_calibration()) {
        return snprintf(response, size, "Entered CALIBRATION mode\r\n");
    } else {
        return snprintf(response, size,
            "Enter calibration FAILED (state=%s)\r\n",
            state_to_string(rudder_get_state())
        );
    }
}

static size_t cmd_cal_exit(char *response, size_t size) {
    if (rudder_exit_calibration()) {
        return snprintf(response, size, "Exited CALIBRATION mode\r\n");
    } else {
        return snprintf(response, size,
            "Exit calibration FAILED (state=%s)\r\n",
            state_to_string(rudder_get_state())
        );
    }
}

static size_t cmd_cal_center(char *response, size_t size) {
    rudder_calibrate_center();
    return snprintf(response, size,
        "Center position captured (raw=%u)\r\n",
        rudder_get_raw_angle()
    );
}

static size_t cmd_cal_port(char *response, size_t size) {
    rudder_calibrate_port();
    return snprintf(response, size,
        "Port limit captured (raw=%u)\r\n",
        rudder_get_raw_angle()
    );
}

static size_t cmd_cal_stbd(char *response, size_t size) {
    rudder_calibrate_starboard();
    return snprintf(response, size,
        "Starboard limit captured (raw=%u)\r\n",
        rudder_get_raw_angle()
    );
}

static size_t cmd_cal_save(char *response, size_t size) {
    if (rudder_calibrate_save()) {
        return snprintf(response, size, "Calibration SAVED and validated\r\n");
    } else {
        return snprintf(response, size,
            "Calibration save FAILED (range too small or not in cal mode)\r\n"
        );
    }
}

static size_t cmd_fault_clear(char *response, size_t size) {
    if (rudder_clear_fault()) {
        return snprintf(response, size, "Fault cleared\r\n");
    } else {
        return snprintf(response, size,
            "Clear fault FAILED (state=%s)\r\n",
            state_to_string(rudder_get_state())
        );
    }
}

static size_t cmd_version(char *response, size_t size) {
    return snprintf(response, size,
        "TestAPEN Rudder Node\r\n"
        "Version: %s\r\n"
        "FSD: TestAP2.FSD.v1.0.0\r\n",
        rudder_get_version()
    );
}

#endif // CONFIG_TESTAP2_NODE_RUDDER

/*============================================================================
 * UI Node Command Handlers
 *============================================================================*/

#ifdef CONFIG_TESTAP2_NODE_UI

static size_t cmd_help(char *response, size_t size) {
    return snprintf(response, size,
        "TestAPEN UI Console Commands:\r\n"
        "  status        - Full system status\r\n"
        "  state         - State machine state\r\n"
        "  display       - Display status\r\n"
        "  page N        - Switch to page N (0-2)\r\n"
        "  buttons       - Show button states\r\n"
        "  nodes         - Show node connectivity\r\n"
        "  param list    - List all parameters\r\n"
        "  param set N V - Set parameter N to value V\r\n"
        "  param save    - Save parameters to NVS\r\n"
        "  param reset   - Reset to defaults\r\n"
        "  espnow        - Show ESP-NOW status\r\n"
        "  btn N         - Simulate button press (0-5)\r\n"
        "  btn N long    - Simulate long button press\r\n"
        "  btn list      - Show button ID mapping\r\n"
        "  version       - Show version info\r\n"
        "  reboot        - Reboot device\r\n"
        "  help          - Show this help\r\n"
    );
}

static size_t cmd_status(char *response, size_t size) {
    char ip[32];
    ui_get_ip_address(ip, sizeof(ip));

    return snprintf(response, size,
        "=== TestAPEN UI Status ===\r\n"
        "Master: %s (state=%s)\r\n"
        "Rudder: %s (angle=%.1f)\r\n"
        "Heading: %.1f deg\r\n"
        "Target: %.1f deg\r\n"
        "Rudder: %.1f deg\r\n"
        "WiFi: %s (%s)\r\n",
        ui_get_master_connected() ? "Connected" : "Disconnected",
        state_to_string((system_state_t)ui_get_state()),
        ui_get_rudder_connected() ? "Connected" : "Disconnected",
        ui_get_rudder_angle(),
        ui_get_heading(),
        ui_get_target_heading(),
        ui_get_rudder_angle(),
        ui_is_wifi_connected() ? "Connected" : "Disconnected",
        ip
    );
}

static size_t cmd_state(char *response, size_t size) {
    return snprintf(response, size,
        "Master State: %s\r\n"
        "Master Fault: %d\r\n",
        state_to_string((system_state_t)ui_get_state()),
        ui_get_fault_code()
    );
}

static size_t cmd_engage(char *response, size_t size) {
    return snprintf(response, size,
        "UI Node cannot directly engage.\r\n"
        "Use physical buttons or send ESP-NOW command.\r\n"
    );
}

static size_t cmd_disengage(char *response, size_t size) {
    return snprintf(response, size,
        "UI Node cannot directly disengage.\r\n"
        "Use physical buttons or send ESP-NOW command.\r\n"
    );
}

static size_t cmd_fault_clear(char *response, size_t size) {
    return snprintf(response, size,
        "UI Node cannot directly clear faults.\r\n"
        "Use Master node console or BLE.\r\n"
    );
}

// External function from ui_node.c
extern int ui_node_simulate_button(int btn_id, bool long_press);

static size_t cmd_btn(const char *arg, char *response, size_t size) {
    // btn list - show button mapping
    if (arg && strcmp(arg, "list") == 0) {
        return snprintf(response, size,
            "Button ID Mapping:\r\n"
            "  0 = DEC10   (-10 degrees)\r\n"
            "  1 = DEC1    (-1 degree)\r\n"
            "  2 = ENGAGE  (toggle engage/disengage)\r\n"
            "  3 = MODE    (cycle display page)\r\n"
            "  4 = INC1    (+1 degree)\r\n"
            "  5 = INC10   (+10 degrees)\r\n"
        );
    }

    if (!arg || strlen(arg) == 0) {
        return snprintf(response, size,
            "Usage: btn <id> [long]\r\n"
            "  btn 2       - Press engage button\r\n"
            "  btn 5 long  - Long press +10 button\r\n"
            "  btn list    - Show button mapping\r\n"
        );
    }

    // Parse button ID
    int btn_id = -1;
    bool long_press = false;

    // Check for "long" modifier
    char arg_copy[32];
    strncpy(arg_copy, arg, sizeof(arg_copy) - 1);
    arg_copy[sizeof(arg_copy) - 1] = '\0';

    char *space = strchr(arg_copy, ' ');
    if (space) {
        *space = '\0';
        if (strcmp(space + 1, "long") == 0) {
            long_press = true;
        }
    }

    btn_id = atoi(arg_copy);

    if (btn_id < 0 || btn_id > 5) {
        return snprintf(response, size,
            "Invalid button ID: %d\r\n"
            "Valid IDs: 0-5 (use 'btn list' for mapping)\r\n",
            btn_id
        );
    }

    int result = ui_node_simulate_button(btn_id, long_press);
    if (result == 0) {
        const char *btn_names[] = {"DEC10", "DEC1", "ENGAGE", "MODE", "INC1", "INC10"};
        return snprintf(response, size,
            "Button %d (%s) %s press simulated\r\n",
            btn_id, btn_names[btn_id], long_press ? "LONG" : "SHORT"
        );
    } else {
        return snprintf(response, size, "Button simulation failed\r\n");
    }
}

static size_t cmd_version(char *response, size_t size) {
    return snprintf(response, size,
        "TestAPEN UI Node\r\n"
        "Version: %s\r\n"
        "FSD: TestAP2.FSD.v1.0.0\r\n",
        ui_get_version()
    );
}

static size_t cmd_display(char *response, size_t size) {
    bool initialized = ui_is_epaper_initialized();
    bool busy = false;
    if (initialized) {
        busy = epaper_is_busy();
    }

    return snprintf(response, size,
        "Display Status:\r\n"
        "  Type: 2.9\" e-Paper (296x128)\r\n"
        "  Driver: SSD1680\r\n"
        "  Initialized: %s\r\n"
        "  Busy: %s\r\n"
        "Use 'display refresh' to force update\r\n",
        initialized ? "YES" : "NO",
        busy ? "YES" : "NO"
    );
}

static size_t cmd_display_refresh(char *response, size_t size) {
    if (!ui_is_epaper_initialized()) {
        return snprintf(response, size, "e-Paper not initialized!\r\n");
    }
    ui_force_display_refresh();
    return snprintf(response, size, "Display refresh triggered\r\n");
}

static size_t cmd_page(const char *args, char *response, size_t size) {
    if (args == NULL || *args == '\0') {
        return snprintf(response, size,
            "Usage: page <N>\r\n"
            "  0 = Autopilot status\r\n"
            "  1 = Navigation\r\n"
            "  2 = System status\r\n"
        );
    }
    int page = atoi(args);
    if (page < 0 || page > 2) {
        return snprintf(response, size, "Invalid page. Use 0-2.\r\n");
    }
    ui_set_page(page);
    return snprintf(response, size, "Switched to page %d\r\n", page);
}

static size_t cmd_nodes(char *response, size_t size) {
    return snprintf(response, size,
        "Node Connectivity:\r\n"
        "  Master: %s\r\n"
        "  Rudder: %s\r\n",
        ui_get_master_connected() ? "CONNECTED" : "DISCONNECTED",
        ui_get_rudder_connected() ? "CONNECTED" : "DISCONNECTED"
    );
}

static size_t cmd_buttons(char *response, size_t size) {
    return ui_get_button_states(response, size);
}

#endif // CONFIG_TESTAP2_NODE_UI

/*============================================================================
 * Common Command Handlers
 *============================================================================*/

static size_t cmd_reboot(char *response, size_t size) {
    g_reboot_pending = true;
    g_reboot_time = xTaskGetTickCount() + pdMS_TO_TICKS(1000);
    return snprintf(response, size, "Rebooting in 1 second...\r\n");
}

static size_t cmd_espnow(char *response, size_t size) {
    espnow_status_t status;
    espnow_get_status(&status);

    char own_mac_str[18];
    espnow_mac_to_str(status.own_mac, own_mac_str);

    char master_mac_str[18];
    char rudder_mac_str[18];
    char ui_mac_str[18];
    espnow_mac_to_str(espnow_get_master_mac(), master_mac_str);
    espnow_mac_to_str(espnow_get_rudder_mac(), rudder_mac_str);
    espnow_mac_to_str(espnow_get_ui_mac(), ui_mac_str);

    return snprintf(response, size,
        "ESP-NOW Status:\r\n"
        "  Initialized: %s\r\n"
        "  Own MAC: %s\r\n"
        "  Channel: %d\r\n"
        "  Peers: %d\r\n"
        "  TX success: %lu\r\n"
        "  TX failed: %lu\r\n"
        "  RX count: %lu\r\n"
        "  RX errors: %lu\r\n"
        "Configured Peers:\r\n"
        "  Master: %s\r\n"
        "  Rudder: %s\r\n"
        "  UI: %s\r\n",
        status.initialized ? "yes" : "no",
        own_mac_str,
        status.channel,
        status.peer_count,
        status.tx_success_count,
        status.tx_fail_count,
        status.rx_count,
        status.rx_error_count,
        master_mac_str,
        rudder_mac_str,
        ui_mac_str);
}

/*============================================================================
 * Parameter Commands (common to both nodes)
 *============================================================================*/

static size_t cmd_param_list(char *response, size_t size) {
    size_t len = 0;
    len += snprintf(response + len, size - len, "Parameters:\r\n");

    for (int i = 0; i < PARAM_COUNT; i++) {
        const param_meta_t *meta = param_get_meta((param_id_t)i);
        if (meta) {
            bool local = param_is_local((param_id_t)i);
            float value = param_get((param_id_t)i);
            len += snprintf(response + len, size - len,
                "  %s = %.3f [%.1f-%.1f] %s\r\n",
                meta->name, value, meta->min_value, meta->max_value,
                local ? "(local)" : "(remote)");
        }
    }
    return len;
}

static size_t cmd_param_get(const char *args, char *response, size_t size) {
    param_id_t id = param_find_by_name(args);
    if (id == PARAM_COUNT) {
        return snprintf(response, size, "Unknown parameter: %s\r\n", args);
    }

    const param_meta_t *meta = param_get_meta(id);
    float value = param_get(id);
    return snprintf(response, size, "%s = %.3f [%.1f-%.1f]\r\n",
                    meta->name, value, meta->min_value, meta->max_value);
}

static size_t cmd_param_set(const char *args, char *response, size_t size) {
    // Parse "name value"
    char name[16];
    float value;
    if (sscanf(args, "%15s %f", name, &value) != 2) {
        return snprintf(response, size, "Usage: param set <name> <value>\r\n");
    }

    param_id_t id = param_find_by_name(name);
    if (id == PARAM_COUNT) {
        return snprintf(response, size, "Unknown parameter: %s\r\n", name);
    }

    esp_err_t err = param_set(id, value);
    if (err == ESP_OK) {
        return snprintf(response, size, "Set %s = %.3f\r\n", name, value);
    } else {
        const param_meta_t *meta = param_get_meta(id);
        return snprintf(response, size, "Invalid value. Range: [%.1f-%.1f]\r\n",
                        meta->min_value, meta->max_value);
    }
}

static size_t cmd_param_save(char *response, size_t size) {
    esp_err_t err = param_save_all();
    if (err == ESP_OK) {
        return snprintf(response, size, "All parameters saved to NVS\r\n");
    } else {
        return snprintf(response, size, "Failed to save parameters\r\n");
    }
}

static size_t cmd_param_reset(char *response, size_t size) {
    esp_err_t err = param_reset_defaults();
    if (err == ESP_OK) {
        return snprintf(response, size, "All parameters reset to defaults\r\n");
    } else {
        return snprintf(response, size, "Failed to reset parameters\r\n");
    }
}

#ifdef CONFIG_TESTAP2_NODE_MASTER
static size_t cmd_pid(const char *args, char *response, size_t size) {
    // If no args, show current PID values
    if (args == NULL || *args == '\0') {
        float kp = param_get(PARAM_KP_HEADING);
        float ki = param_get(PARAM_KI_HEADING);
        float kd = param_get(PARAM_KD_HEADING);
        return snprintf(response, size, "PID: Kp=%.3f Ki=%.3f Kd=%.3f\r\n", kp, ki, kd);
    }

    // Parse "kp ki kd"
    float kp, ki, kd;
    if (sscanf(args, "%f %f %f", &kp, &ki, &kd) != 3) {
        return snprintf(response, size, "Usage: pid <kp> <ki> <kd>\r\n");
    }

    esp_err_t err = param_set(PARAM_KP_HEADING, kp);
    if (err != ESP_OK) {
        return snprintf(response, size, "Invalid Kp value\r\n");
    }
    err = param_set(PARAM_KI_HEADING, ki);
    if (err != ESP_OK) {
        return snprintf(response, size, "Invalid Ki value\r\n");
    }
    err = param_set(PARAM_KD_HEADING, kd);
    if (err != ESP_OK) {
        return snprintf(response, size, "Invalid Kd value\r\n");
    }

    return snprintf(response, size, "PID set: Kp=%.3f Ki=%.3f Kd=%.3f\r\n", kp, ki, kd);
}
#endif

#ifdef CONFIG_TESTAP2_NODE_RUDDER
static size_t cmd_servo(const char *args, char *response, size_t size) {
    // If no args, show current servo values
    if (args == NULL || *args == '\0') {
        float kp = param_get(PARAM_KP_SERVO);
        float db_ent = param_get(PARAM_DEADBAND_ENTER);
        float db_ext = param_get(PARAM_DEADBAND_EXIT);
        return snprintf(response, size, "Servo: Kp=%.1f DB_enter=%.2f DB_exit=%.2f\r\n",
                        kp, db_ent, db_ext);
    }

    // Parse "kp db_enter db_exit"
    float kp, db_ent, db_ext;
    if (sscanf(args, "%f %f %f", &kp, &db_ent, &db_ext) != 3) {
        return snprintf(response, size, "Usage: servo <kp> <db_enter> <db_exit>\r\n");
    }

    esp_err_t err = param_set(PARAM_KP_SERVO, kp);
    if (err != ESP_OK) {
        return snprintf(response, size, "Invalid Kp value\r\n");
    }
    err = param_set(PARAM_DEADBAND_ENTER, db_ent);
    if (err != ESP_OK) {
        return snprintf(response, size, "Invalid deadband enter value\r\n");
    }
    err = param_set(PARAM_DEADBAND_EXIT, db_ext);
    if (err != ESP_OK) {
        return snprintf(response, size, "Invalid deadband exit value\r\n");
    }

    return snprintf(response, size, "Servo set: Kp=%.1f DB_enter=%.2f DB_exit=%.2f\r\n",
                    kp, db_ent, db_ext);
}

static size_t cmd_motor_params(const char *args, char *response, size_t size) {
    // If no args, show current motor values
    if (args == NULL || *args == '\0') {
        float min_spd = param_get(PARAM_MIN_MOTOR_SPEED);
        float max_spd = param_get(PARAM_MAX_MOTOR_SPEED);
        float slew = param_get(PARAM_RUDDER_SLEW_RATE);
        return snprintf(response, size, "Motor: min=%d%% max=%d%% slew=%.1f deg/s\r\n",
                        (int)min_spd, (int)max_spd, slew);
    }

    // Parse "min max slew"
    float min_spd, max_spd, slew;
    if (sscanf(args, "%f %f %f", &min_spd, &max_spd, &slew) != 3) {
        return snprintf(response, size, "Usage: motor params <min> <max> <slew>\r\n");
    }

    esp_err_t err = param_set(PARAM_MIN_MOTOR_SPEED, min_spd);
    if (err != ESP_OK) {
        return snprintf(response, size, "Invalid min speed value\r\n");
    }
    err = param_set(PARAM_MAX_MOTOR_SPEED, max_spd);
    if (err != ESP_OK) {
        return snprintf(response, size, "Invalid max speed value\r\n");
    }
    err = param_set(PARAM_RUDDER_SLEW_RATE, slew);
    if (err != ESP_OK) {
        return snprintf(response, size, "Invalid slew rate value\r\n");
    }

    return snprintf(response, size, "Motor set: min=%d%% max=%d%% slew=%.1f deg/s\r\n",
                    (int)min_spd, (int)max_spd, slew);
}
#endif

/*============================================================================
 * Public API
 *============================================================================*/

void console_init(void) {
    ESP_LOGI(TAG, "Console initialized");
}

size_t console_process_command(const char *cmd, char *response, size_t response_size) {
    // Skip leading whitespace
    while (*cmd == ' ' || *cmd == '\t') cmd++;

    // Empty command
    if (*cmd == '\0') {
        return 0;
    }

    ESP_LOGI(TAG, "Command: %s", cmd);

    // Common commands
    if (strcmp(cmd, "help") == 0) {
        return cmd_help(response, response_size);
    }
    else if (strcmp(cmd, "status") == 0) {
        return cmd_status(response, response_size);
    }
    else if (strcmp(cmd, "state") == 0) {
        return cmd_state(response, response_size);
    }
    else if (strcmp(cmd, "engage") == 0) {
        return cmd_engage(response, response_size);
    }
    else if (strcmp(cmd, "disengage") == 0) {
        return cmd_disengage(response, response_size);
    }
    else if (strcmp(cmd, "fault clear") == 0) {
        return cmd_fault_clear(response, response_size);
    }
    else if (strcmp(cmd, "version") == 0) {
        return cmd_version(response, response_size);
    }
    else if (strcmp(cmd, "reboot") == 0) {
        return cmd_reboot(response, response_size);
    }
    // Parameter commands (common)
    else if (strcmp(cmd, "param list") == 0) {
        return cmd_param_list(response, response_size);
    }
    else if (strncmp(cmd, "param get ", 10) == 0) {
        return cmd_param_get(cmd + 10, response, response_size);
    }
    else if (strncmp(cmd, "param set ", 10) == 0) {
        return cmd_param_set(cmd + 10, response, response_size);
    }
    else if (strcmp(cmd, "param save") == 0) {
        return cmd_param_save(response, response_size);
    }
    else if (strcmp(cmd, "param reset") == 0) {
        return cmd_param_reset(response, response_size);
    }
    else if (strcmp(cmd, "espnow") == 0) {
        return cmd_espnow(response, response_size);
    }

#ifdef CONFIG_TESTAP2_NODE_MASTER
    // Master-specific commands
    else if (strcmp(cmd, "pid") == 0) {
        return cmd_pid(NULL, response, response_size);
    }
    else if (strncmp(cmd, "pid ", 4) == 0) {
        return cmd_pid(cmd + 4, response, response_size);
    }
    else if (strcmp(cmd, "heading") == 0) {
        return cmd_heading(response, response_size);
    }
    else if (strcmp(cmd, "imu") == 0) {
        return cmd_imu(response, response_size);
    }
    else if (strncmp(cmd, "set heading ", 12) == 0) {
        return cmd_set_heading(cmd + 12, response, response_size);
    }
    else if (strncmp(cmd, "adjust ", 7) == 0) {
        return cmd_adjust(cmd + 7, response, response_size);
    }
    else if (strncmp(cmd, "heading sim ", 12) == 0) {
        return cmd_heading_sim(cmd + 12, response, response_size);
    }
    else if (strcmp(cmd, "heading real") == 0) {
        return cmd_heading_real(response, response_size);
    }
    else if (strcmp(cmd, "magcal get") == 0) {
        return cmd_magcal_get(response, response_size);
    }
    else if (strncmp(cmd, "magcal set ", 11) == 0) {
        return cmd_magcal_set(cmd + 11, response, response_size);
    }
    else if (strcmp(cmd, "magcal start") == 0) {
        return cmd_magcal_start(response, response_size);
    }
    else if (strcmp(cmd, "magcal stop") == 0) {
        return cmd_magcal_stop(response, response_size);
    }
    else if (strcmp(cmd, "magcal status") == 0) {
        return cmd_magcal_status(response, response_size);
    }
    else if (strcmp(cmd, "magcal save") == 0) {
        return cmd_magcal_save(response, response_size);
    }
    else if (strcmp(cmd, "accelcal") == 0) {
        return cmd_accelcal(response, response_size);
    }
    else if (strcmp(cmd, "gyrocal") == 0) {
        return cmd_gyrocal(response, response_size);
    }
    else if (strcmp(cmd, "fusion") == 0) {
        return cmd_fusion(NULL, response, response_size);
    }
    else if (strncmp(cmd, "fusion ", 7) == 0) {
        return cmd_fusion(cmd + 7, response, response_size);
    }
    else if (strcmp(cmd, "gpsoffset") == 0) {
        return cmd_gpsoffset(NULL, response, response_size);
    }
    else if (strncmp(cmd, "gpsoffset ", 10) == 0) {
        return cmd_gpsoffset(cmd + 10, response, response_size);
    }
    else if (strcmp(cmd, "gnss") == 0) {
        return cmd_gnss(response, response_size);
    }
#endif

#ifdef CONFIG_TESTAP2_NODE_RUDDER
    // Rudder-specific commands
    else if (strcmp(cmd, "rudder") == 0) {
        return cmd_rudder(response, response_size);
    }
    else if (strcmp(cmd, "motor") == 0) {
        return cmd_motor(response, response_size);
    }
    else if (strcmp(cmd, "servo") == 0) {
        return cmd_servo(NULL, response, response_size);
    }
    else if (strncmp(cmd, "servo ", 6) == 0) {
        return cmd_servo(cmd + 6, response, response_size);
    }
    else if (strcmp(cmd, "motor params") == 0) {
        return cmd_motor_params(NULL, response, response_size);
    }
    else if (strncmp(cmd, "motor params ", 13) == 0) {
        return cmd_motor_params(cmd + 13, response, response_size);
    }
    else if (strncmp(cmd, "set rudder ", 11) == 0) {
        return cmd_set_rudder(cmd + 11, response, response_size);
    }
    else if (strcmp(cmd, "cal enter") == 0) {
        return cmd_cal_enter(response, response_size);
    }
    else if (strcmp(cmd, "cal exit") == 0) {
        return cmd_cal_exit(response, response_size);
    }
    else if (strcmp(cmd, "cal center") == 0) {
        return cmd_cal_center(response, response_size);
    }
    else if (strcmp(cmd, "cal port") == 0) {
        return cmd_cal_port(response, response_size);
    }
    else if (strcmp(cmd, "cal stbd") == 0) {
        return cmd_cal_stbd(response, response_size);
    }
    else if (strcmp(cmd, "cal save") == 0) {
        return cmd_cal_save(response, response_size);
    }
#endif

#ifdef CONFIG_TESTAP2_NODE_UI
    // UI-specific commands
    else if (strcmp(cmd, "display") == 0) {
        return cmd_display(response, response_size);
    }
    else if (strcmp(cmd, "display refresh") == 0) {
        return cmd_display_refresh(response, response_size);
    }
    else if (strcmp(cmd, "page") == 0) {
        return cmd_page(NULL, response, response_size);
    }
    else if (strncmp(cmd, "page ", 5) == 0) {
        return cmd_page(cmd + 5, response, response_size);
    }
    else if (strcmp(cmd, "nodes") == 0) {
        return cmd_nodes(response, response_size);
    }
    else if (strcmp(cmd, "buttons") == 0) {
        return cmd_buttons(response, response_size);
    }
    else if (strcmp(cmd, "btn") == 0) {
        return cmd_btn(NULL, response, response_size);
    }
    else if (strncmp(cmd, "btn ", 4) == 0) {
        return cmd_btn(cmd + 4, response, response_size);
    }
#endif

    else {
        return snprintf(response, response_size, "Unknown command: %s\r\nType 'help' for commands\r\n", cmd);
    }
}

void console_handle(void) {
    if (g_reboot_pending && xTaskGetTickCount() >= g_reboot_time) {
        ESP_LOGI(TAG, "Rebooting...");
        esp_restart();
    }
}
