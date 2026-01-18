/**
 * @file master_interface.h
 * @brief Interface for Master Node state access
 *
 * Provides functions for console and other modules to interact with
 * the Master Node state.
 */

#ifndef MASTER_INTERFACE_H
#define MASTER_INTERFACE_H

#include "state_machine.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// State machine access
state_t master_get_state(void);
uint8_t master_get_fault_code(void);
bool master_engage(void);
bool master_disengage(void);
bool master_clear_fault(void);
bool master_enter_calibration(void);
bool master_exit_calibration(void);

// Heading access
float master_get_heading_filtered(void);
float master_get_heading_raw(void);
float master_get_target_heading(void);
float master_get_heading_error(void);
void master_set_target_heading(float heading);
void master_adjust_target_heading(float delta);

// IMU access
float master_get_yaw_rate(void);
float master_get_roll(void);
float master_get_pitch(void);
bool master_imu_is_valid(void);

// Simulation mode
void master_set_heading_simulation(bool enable);
bool master_is_heading_simulation(void);
void master_set_simulated_heading(float heading);

// Magnetometer calibration
void master_set_mag_cal(float x, float y, float z);
void master_get_mag_cal(float *x, float *y, float *z);

// Network info
bool master_is_wifi_connected(void);
void master_get_ip_address(char *buf, size_t buf_size);

// System
void master_request_reboot(void);
const char* master_get_version(void);

#ifdef __cplusplus
}
#endif

#endif // MASTER_INTERFACE_H
