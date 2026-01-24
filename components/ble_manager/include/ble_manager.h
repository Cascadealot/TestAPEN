/**
 * @file ble_manager.h
 * @brief BLE Manager for TestAPEN Autopilot
 *
 * Implements BLE GATT service per FSD Section 11.
 * Provides control interface for autopilot via BLE.
 *
 * FSD Reference: TestAPEN.FSD.v1.0.0.md Section 11
 */

#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * BLE Service UUIDs (FSD Section 11.1)
 *============================================================================*/

// Service UUID: 12345678-1234-1234-1234-123456789abc
#define BLE_SERVICE_UUID_BASE { \
    0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12, \
    0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12 \
}

/*============================================================================
 * BLE Command Codes (FSD Section 11.2)
 *============================================================================*/

#define BLE_CMD_ENGAGE          0x01
#define BLE_CMD_DISENGAGE       0x02
#define BLE_CMD_SET_HEADING     0x03
#define BLE_CMD_ADJUST_HEADING  0x04

#define BLE_CMD_CAL_ENTER       0x10
#define BLE_CMD_CAL_EXIT        0x11
#define BLE_CMD_CAL_CENTER      0x12
#define BLE_CMD_CAL_PORT        0x13
#define BLE_CMD_CAL_STBD        0x14
#define BLE_CMD_CAL_SAVE        0x15

#define BLE_CMD_FAULT_CLEAR     0x20
#define BLE_CMD_STATUS_REQ      0x30

#define BLE_CMD_SEA_STATE_MODE  0x40
#define BLE_CMD_LAZY_HELM       0x41
#define BLE_CMD_STEER_MODE      0x42
#define BLE_CMD_GNSS_STATUS     0x43

// Parameter commands (extended)
#define BLE_CMD_PARAM_GET       0x50
#define BLE_CMD_PARAM_SET       0x51
#define BLE_CMD_PARAM_SAVE      0x52
#define BLE_CMD_PARAM_RESET     0x53

// Magnetometer calibration commands
#define BLE_CMD_MAGCAL_GET      0x60    // Read current mag calibration
#define BLE_CMD_MAGCAL_SET      0x61    // Write mag calibration (hard + soft iron)
#define BLE_CMD_MAGCAL_SAVE     0x62    // Save mag calibration to NVS
#define BLE_CMD_MAGCAL_RESET    0x63    // Reset mag calibration to defaults

/*============================================================================
 * API Functions
 *============================================================================*/

/**
 * @brief Initialize BLE manager
 *
 * Initializes NimBLE stack, creates GATT service with characteristics,
 * and starts advertising as "TestAPEN".
 *
 * @return ESP_OK on success
 */
esp_err_t ble_manager_init(void);

/**
 * @brief Check if a BLE client is connected
 *
 * @return true if connected
 */
bool ble_manager_is_connected(void);

/**
 * @brief Update status characteristic
 *
 * Call this when system state changes to notify connected clients.
 *
 * @param state System state
 * @param fault Fault code
 * @param flags Status flags
 */
void ble_manager_update_status(uint8_t state, uint8_t fault, uint8_t flags);

/**
 * @brief Update heading characteristic
 *
 * Call this periodically with current heading information.
 *
 * @param current Current heading (degrees)
 * @param target Target heading (degrees)
 */
void ble_manager_update_heading(float current, float target);

/**
 * @brief Update rudder characteristic
 *
 * Call this periodically with rudder position information.
 *
 * @param angle Current rudder angle (degrees)
 * @param motor_status Motor status flags
 */
void ble_manager_update_rudder(float angle, uint8_t motor_status);

/**
 * @brief Notify parameter change
 *
 * Send parameter update notification to connected client.
 *
 * @param param_id Parameter ID
 * @param value New value
 */
void ble_manager_notify_param(uint8_t param_id, float value);

#ifdef __cplusplus
}
#endif

#endif // BLE_MANAGER_H
