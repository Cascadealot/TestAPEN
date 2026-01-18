/**
 * @file icm20948.h
 * @brief ICM-20948 9-axis IMU Driver for ESP-IDF
 */

#ifndef ICM20948_H
#define ICM20948_H

#include "esp_err.h"
#include "driver/i2c.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// I2C Addresses
#define ICM20948_I2C_ADDR       0x68
#define AK09916_I2C_ADDR        0x0C

// IMU Data Structure
typedef struct {
    // Accelerometer (in g)
    float accel_x;
    float accel_y;
    float accel_z;

    // Gyroscope (in deg/s)
    float gyro_x;
    float gyro_y;
    float gyro_z;

    // Magnetometer (in uT)
    float mag_x;
    float mag_y;
    float mag_z;

    // Computed values
    float heading_raw;      // Tilt-compensated heading [0, 360)
    float heading_filtered; // Filtered heading
    float roll;             // Roll angle (degrees)
    float pitch;            // Pitch angle (degrees)
    float yaw_rate;         // Yaw rate from gyro (deg/s)

    // Status
    bool valid;
    uint32_t last_update_ms;
} icm20948_data_t;

/**
 * @brief Initialize ICM-20948
 * @param i2c_num I2C port number
 * @param i2c_mutex Mutex for I2C access (can be NULL)
 * @return ESP_OK on success
 */
esp_err_t icm20948_init(i2c_port_t i2c_num, SemaphoreHandle_t i2c_mutex);

/**
 * @brief Read sensor data and update heading
 * @return ESP_OK on success
 */
esp_err_t icm20948_update(void);

/**
 * @brief Get current IMU data
 * @param data Pointer to data structure
 */
void icm20948_get_data(icm20948_data_t *data);

/**
 * @brief Get filtered heading
 * @return Heading in degrees [0, 360)
 */
float icm20948_get_heading(void);

/**
 * @brief Get raw (unfiltered) heading
 * @return Heading in degrees [0, 360)
 */
float icm20948_get_heading_raw(void);

/**
 * @brief Get yaw rate from gyroscope
 * @return Yaw rate in degrees/second
 */
float icm20948_get_yaw_rate(void);

/**
 * @brief Set hard-iron magnetometer calibration
 * @param x X offset in uT
 * @param y Y offset in uT
 * @param z Z offset in uT
 */
void icm20948_set_mag_cal(float x, float y, float z);

/**
 * @brief Get hard-iron magnetometer calibration
 */
void icm20948_get_mag_cal(float *x, float *y, float *z);

/**
 * @brief Set soft-iron magnetometer calibration matrix
 * @param matrix 3x3 correction matrix in row-major order (9 floats)
 */
void icm20948_set_mag_soft_iron(const float matrix[9]);

/**
 * @brief Get soft-iron magnetometer calibration matrix
 * @param matrix Output buffer for 3x3 matrix in row-major order (9 floats)
 */
void icm20948_get_mag_soft_iron(float matrix[9]);

/**
 * @brief Load magnetometer calibration from NVS (param_store)
 * Loads both hard-iron offsets and soft-iron matrix
 */
void icm20948_load_mag_cal_from_nvs(void);

/**
 * @brief Save magnetometer calibration to NVS (param_store)
 * Saves both hard-iron offsets and soft-iron matrix
 * @return ESP_OK on success
 */
esp_err_t icm20948_save_mag_cal_to_nvs(void);

/**
 * @brief Reset magnetometer calibration to defaults
 * Hard-iron: all zeros, Soft-iron: identity matrix
 * Does NOT save to NVS - call save function separately
 */
void icm20948_reset_mag_cal(void);

/**
 * @brief Enable simulation mode (use synthetic heading)
 * @param enable true to enable simulation
 */
void icm20948_set_simulation(bool enable);

/**
 * @brief Check if simulation mode is enabled
 */
bool icm20948_is_simulation(void);

/**
 * @brief Set simulated heading (only used in simulation mode)
 * @param heading Heading in degrees
 */
void icm20948_set_simulated_heading(float heading);

/**
 * @brief Check if IMU data is valid
 */
bool icm20948_is_valid(void);

/*============================================================================
 * Guided Magnetometer Calibration (Interactive)
 *============================================================================*/

/**
 * @brief Start magnetometer calibration mode
 * Begins collecting min/max magnetometer values.
 * User should rotate device in figure-8 pattern.
 * @return ESP_OK on success
 */
esp_err_t icm20948_magcal_start(void);

/**
 * @brief Stop calibration and compute results
 * Computes hard-iron offsets and soft-iron scale factors.
 * Call icm20948_save_mag_cal_to_nvs() to persist results.
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not enough samples
 */
esp_err_t icm20948_magcal_stop(void);

/**
 * @brief Check if calibration mode is active
 * @return true if calibration in progress
 */
bool icm20948_magcal_is_running(void);

/**
 * @brief Get calibration quality metric
 * Higher is better. 1.0 = perfect sphere.
 * @return Quality 0.0-1.0, or -1.0 if calibration not complete
 */
float icm20948_magcal_get_quality(void);

/**
 * @brief Get number of samples collected during calibration
 * @return Sample count
 */
uint32_t icm20948_magcal_get_sample_count(void);

/**
 * @brief Get calibration status string
 * @param buf Output buffer
 * @param size Buffer size
 * @return Length written
 */
size_t icm20948_magcal_get_status(char *buf, size_t size);

/*============================================================================
 * Accelerometer Calibration
 *============================================================================*/

/**
 * @brief Calibrate accelerometer assuming device is level
 * Measures gravity and computes offset correction.
 * Device must be stationary and level during calibration.
 * @return ESP_OK on success
 */
esp_err_t icm20948_accel_cal_level(void);

/**
 * @brief Get accelerometer calibration offsets
 * @param x X offset in g (output)
 * @param y Y offset in g (output)
 * @param z Z offset in g (output)
 */
void icm20948_get_accel_cal(float *x, float *y, float *z);

/**
 * @brief Set accelerometer calibration offsets
 * @param x X offset in g
 * @param y Y offset in g
 * @param z Z offset in g
 */
void icm20948_set_accel_cal(float x, float y, float z);

/*============================================================================
 * Gyroscope Calibration
 *============================================================================*/

/**
 * @brief Calibrate gyroscope bias (device must be stationary)
 * Collects samples and computes average bias.
 * @return ESP_OK on success
 */
esp_err_t icm20948_calibrate_gyro(void);

/**
 * @brief Get gyroscope bias values
 * @param x X bias in deg/s (output)
 * @param y Y bias in deg/s (output)
 * @param z Z bias in deg/s (output)
 */
void icm20948_get_gyro_bias(float *x, float *y, float *z);

/*============================================================================
 * Sensor Fusion (Madgwick Filter)
 *============================================================================*/

/**
 * @brief Enable or disable sensor fusion
 * When enabled, heading uses Madgwick 9-axis fusion.
 * When disabled, uses simple tilt-compensated magnetometer.
 * @param enable true to enable fusion (default)
 */
void icm20948_set_fusion_enabled(bool enable);

/**
 * @brief Check if sensor fusion is enabled
 * @return true if fusion is enabled
 */
bool icm20948_is_fusion_enabled(void);

/**
 * @brief Set Madgwick filter gain (beta)
 * Higher = faster convergence, more noise
 * Lower = smoother, slower response
 * Range: 0.01 - 1.0, default 0.1
 * @param beta Filter gain
 */
void icm20948_set_fusion_beta(float beta);

/**
 * @brief Get current Madgwick filter gain
 * @return Current beta value
 */
float icm20948_get_fusion_beta(void);

/**
 * @brief Check if sensor fusion has converged
 * @return true if filter has processed enough samples
 */
bool icm20948_is_fusion_converged(void);

/**
 * @brief Reset sensor fusion filter
 * Call after significant orientation change.
 */
void icm20948_reset_fusion(void);

/*============================================================================
 * GPS-Aided Calibration Monitoring
 *============================================================================*/

/**
 * @brief Update GPS reference heading for calibration monitoring
 * Call this when GPS COG is valid (speed > 3 knots typically)
 * @param gps_heading GPS Course Over Ground in degrees [0, 360)
 * @param speed_knots Current speed in knots (used to validate COG)
 */
void icm20948_update_gps_reference(float gps_heading, float speed_knots);

/**
 * @brief Get estimated heading offset from GPS
 * Positive value means magnetometer reads higher than GPS.
 * @return Average heading offset in degrees, or NAN if no data
 */
float icm20948_get_heading_offset(void);

/**
 * @brief Get number of GPS heading samples collected
 * @return Sample count
 */
uint32_t icm20948_get_gps_sample_count(void);

/**
 * @brief Reset GPS heading offset estimation
 */
void icm20948_reset_gps_offset(void);

/**
 * @brief Apply GPS-derived heading offset to calibration
 * Adjusts hard-iron X/Y offsets to reduce heading error.
 * Call after collecting sufficient GPS samples.
 * @return ESP_OK if offset applied, ESP_ERR_INVALID_STATE if insufficient data
 */
esp_err_t icm20948_apply_gps_offset(void);

#ifdef __cplusplus
}
#endif

#endif // ICM20948_H
