/**
 * @file as5600.h
 * @brief AS5600 Magnetic Rotary Encoder Driver for ESP-IDF
 *
 * FSD Reference: TestAP2.FSD.v1.0.0.md Section 4.2, 9.3
 *
 * Features:
 * - 12-bit angle measurement (0-4095 counts = 0-360 degrees)
 * - Magnet status monitoring (detected, too weak, too strong)
 * - I2C mutex support for multi-task safety
 */

#ifndef AS5600_H
#define AS5600_H

#include "esp_err.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// I2C Address (fixed, not configurable on AS5600)
#define AS5600_I2C_ADDR         0x36

// Register Addresses
#define AS5600_REG_ZMCO         0x00    // Zero position programming count
#define AS5600_REG_ZPOS_H       0x01    // Zero position high byte
#define AS5600_REG_ZPOS_L       0x02    // Zero position low byte
#define AS5600_REG_MPOS_H       0x03    // Maximum position high byte
#define AS5600_REG_MPOS_L       0x04    // Maximum position low byte
#define AS5600_REG_MANG_H       0x05    // Maximum angle high byte
#define AS5600_REG_MANG_L       0x06    // Maximum angle low byte
#define AS5600_REG_CONF_H       0x07    // Configuration high byte
#define AS5600_REG_CONF_L       0x08    // Configuration low byte
#define AS5600_REG_RAW_ANGLE_H  0x0C    // Raw angle high byte (read-only)
#define AS5600_REG_RAW_ANGLE_L  0x0D    // Raw angle low byte (read-only)
#define AS5600_REG_ANGLE_H      0x0E    // Filtered angle high byte (read-only)
#define AS5600_REG_ANGLE_L      0x0F    // Filtered angle low byte (read-only)
#define AS5600_REG_STATUS       0x0B    // Status register (read-only)
#define AS5600_REG_AGC          0x1A    // Automatic gain control (read-only)
#define AS5600_REG_MAGNITUDE_H  0x1B    // Magnitude high byte (read-only)
#define AS5600_REG_MAGNITUDE_L  0x1C    // Magnitude low byte (read-only)

// Status register bit masks (FSD Section 9.3)
#define AS5600_STATUS_MH        (1 << 3)    // Magnet too strong
#define AS5600_STATUS_ML        (1 << 4)    // Magnet too weak
#define AS5600_STATUS_MD        (1 << 5)    // Magnet detected

// Angle conversion
#define AS5600_RAW_TO_DEGREES(raw)  ((float)(raw) * 360.0f / 4096.0f)
#define AS5600_DEGREES_TO_RAW(deg)  ((uint16_t)((deg) * 4096.0f / 360.0f))

// Status structure
typedef struct {
    bool magnet_detected;   // MD bit - magnet present
    bool magnet_too_weak;   // ML bit - magnet too far or weak
    bool magnet_too_strong; // MH bit - magnet too close or strong
    uint8_t agc;            // Automatic gain control value (0-255)
    uint16_t magnitude;     // Magnetic field magnitude
} as5600_status_t;

// Data structure
typedef struct {
    uint16_t raw_angle;     // Raw 12-bit angle (0-4095)
    float angle_degrees;    // Angle in degrees (0-360)
    as5600_status_t status; // Magnet status
    bool valid;             // Data validity flag
    uint32_t last_update_ms;// Timestamp of last successful read
} as5600_data_t;

/**
 * @brief Initialize AS5600 encoder
 *
 * Verifies device presence at I2C address 0x36 and checks magnet status.
 *
 * @param i2c_num I2C port number (I2C_NUM_0 or I2C_NUM_1)
 * @param i2c_mutex Mutex for I2C access (required for multi-task safety)
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if device not responding,
 *         ESP_ERR_INVALID_STATE if magnet not detected
 */
esp_err_t as5600_init(i2c_port_t i2c_num, SemaphoreHandle_t i2c_mutex);

/**
 * @brief Read raw 12-bit angle value
 *
 * @param raw_angle Pointer to store raw angle (0-4095)
 * @return ESP_OK on success
 */
esp_err_t as5600_read_raw_angle(uint16_t *raw_angle);

/**
 * @brief Read angle in degrees
 *
 * @param degrees Pointer to store angle (0.0 - 360.0)
 * @return ESP_OK on success
 */
esp_err_t as5600_read_angle_degrees(float *degrees);

/**
 * @brief Read magnet status
 *
 * @param status Pointer to status structure
 * @return ESP_OK on success
 */
esp_err_t as5600_read_status(as5600_status_t *status);

/**
 * @brief Check if magnet is detected
 *
 * @return true if magnet detected, false otherwise
 */
bool as5600_is_magnet_detected(void);

/**
 * @brief Get full encoder data (angle + status)
 *
 * This is the main function for Task_Rudder to call at 50Hz.
 * Updates internal state and returns all data.
 *
 * @param data Pointer to data structure
 * @return ESP_OK on success
 */
esp_err_t as5600_update(as5600_data_t *data);

/**
 * @brief Get last known angle in degrees
 *
 * Returns cached value without I2C read.
 *
 * @return Last known angle in degrees
 */
float as5600_get_angle(void);

/**
 * @brief Check if encoder data is valid
 *
 * @return true if data is valid and recent
 */
bool as5600_is_valid(void);

/**
 * @brief Set zero position offset
 *
 * Used during calibration to set the center position.
 *
 * @param raw_offset Raw angle value to use as zero
 * @return ESP_OK on success
 */
esp_err_t as5600_set_zero_position(uint16_t raw_offset);

/**
 * @brief Get current zero position offset
 *
 * @return Current zero position raw value
 */
uint16_t as5600_get_zero_position(void);

#ifdef __cplusplus
}
#endif

#endif // AS5600_H
