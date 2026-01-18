/**
 * @file autopilot_common.h
 * @brief Common definitions for TestAP2 Autopilot
 *
 * FSD Reference: TestAP2.FSD.v1.0.0.md
 */

#ifndef AUTOPILOT_COMMON_H
#define AUTOPILOT_COMMON_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * Version Information
 *============================================================================*/

#define TESTAP2_FSD_VERSION     "1.0.0"
#define TESTAP2_FW_VERSION      "1.1.0"

/*============================================================================
 * Node IDs (FSD Section 5.3)
 *============================================================================*/

#define NODE_ID_BROADCAST       0
#define NODE_ID_MASTER          1
#define NODE_ID_RUDDER          2

/*============================================================================
 * Control Parameters (FSD Section 7.3)
 * Note: These are DEFAULT values. Runtime values come from param_store.
 *============================================================================*/

// PID Gains (defaults - use param_get() for runtime values)
#define KP_HEADING_DEFAULT      0.8f    // °/°
#define KI_HEADING_DEFAULT      0.05f   // °/(°·s)
#define KD_HEADING_DEFAULT      0.5f    // °/(°/s)

// Limits
#define INTEGRAL_LIMIT          5.0f    // °
#define INTEGRAL_RESET_THRESH   20.0f   // °
#define MAX_YAW_RATE_CMD        10.0f   // °/s
#define RUDDER_CMD_MAX          35.0f   // °

/*============================================================================
 * Adaptive Filter Alpha Values (FSD Section 7.4)
 *============================================================================*/

#define ALPHA_CALM              0.15f
#define ALPHA_NORMAL            0.08f
#define ALPHA_ROUGH             0.05f
#define ALPHA_STORM             0.03f

#define VARIANCE_CALM           4.0f
#define VARIANCE_NORMAL         16.0f
#define VARIANCE_ROUGH          36.0f

/*============================================================================
 * Servo Parameters (FSD Section 7.6)
 * Note: These are DEFAULT values. Runtime values come from param_store.
 *============================================================================*/

#define KP_SERVO_DEFAULT        10.0f   // %/°
#define DEADBAND_ENTER_DEFAULT  1.0f    // °
#define DEADBAND_EXIT_DEFAULT   1.5f    // °
#define RUDDER_SLEW_RATE_DEFAULT 15.0f  // °/s
#define MIN_MOTOR_SPEED_DEFAULT 20      // %
#define MAX_MOTOR_SPEED_DEFAULT 100     // %

/*============================================================================
 * PWM Configuration (FSD Section 4.8)
 *============================================================================*/

#define PWM_FREQUENCY_HZ        20000   // 20 kHz
#define PWM_RESOLUTION_BITS     8       // 0-255

/*============================================================================
 * Safety Thresholds (FSD Section 9.2)
 *============================================================================*/

#define HEARTBEAT_TIMEOUT_MS    500
#define COMMAND_TIMEOUT_MS      200
#define COMMAND_LOSS_FAULT_MS   500
#define STALL_TIMEOUT_MS        500
#define STALL_MIN_DELTA_DEG     0.5f
#define DRIVE_TIMEOUT_MS        5000
#define COURSE_DEVIATION_WARN   15.0f   // °
#define COURSE_DEVIATION_ALARM  30.0f   // °
#define MIN_CAL_RANGE           5.0f    // °

/*============================================================================
 * Voltage Thresholds (FSD Section 4.6)
 *============================================================================*/

#define VOLTAGE_LOW_WARN        22.0f   // V
#define VOLTAGE_LOW_FAULT       20.0f   // V
#define VOLTAGE_HIGH_WARN       29.0f   // V

/*============================================================================
 * Task Stack Sizes (FSD Section 6.2, 6.3)
 *============================================================================*/

#define TASK_CAN_STACK_SIZE     4096
#define TASK_IMU_STACK_SIZE     4096
#define TASK_GNSS_STACK_SIZE    4096
#define TASK_AUTOPILOT_STACK_SIZE 2048
#define TASK_DISPLAY_STACK_SIZE 4096
#define TASK_BLE_STACK_SIZE     4096
#define TASK_RUDDER_STACK_SIZE  4096

/*============================================================================
 * Task Priorities
 *============================================================================*/

#define TASK_CAN_PRIORITY       5
#define TASK_IMU_PRIORITY       4
#define TASK_GNSS_PRIORITY      4
#define TASK_RUDDER_PRIORITY    4
#define TASK_AUTOPILOT_PRIORITY 3
#define TASK_DISPLAY_PRIORITY   2
#define TASK_BLE_PRIORITY       1

/*============================================================================
 * Task Rates (Hz)
 *============================================================================*/

#define RATE_IMU_HZ             50
#define RATE_RUDDER_HZ          50
#define RATE_AUTOPILOT_HZ       10
#define RATE_DISPLAY_HZ         5

/*============================================================================
 * Sea State Enumeration
 *============================================================================*/

typedef enum {
    SEA_STATE_AUTO = 0,
    SEA_STATE_CALM = 1,
    SEA_STATE_NORMAL = 2,
    SEA_STATE_ROUGH = 3,
    SEA_STATE_STORM = 4
} sea_state_t;

/*============================================================================
 * Utility Functions
 *============================================================================*/

/**
 * @brief Wrap angle to [-180, 180) range
 */
float wrap180(float angle);

/**
 * @brief Wrap angle to [0, 360) range
 */
float wrap360(float angle);

/**
 * @brief Clamp value to range
 */
float clampf(float value, float min, float max);

/**
 * @brief Clamp delta (rate limit)
 */
float clamp_delta(float value, float last, float max_delta);

/**
 * @brief Sign function
 */
int signf(float value);

#ifdef __cplusplus
}
#endif

#endif // AUTOPILOT_COMMON_H
