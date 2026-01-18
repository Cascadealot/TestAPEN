/**
 * @file madgwick.h
 * @brief Madgwick AHRS (Attitude and Heading Reference System) Filter
 *
 * Implementation of Sebastian Madgwick's sensor fusion algorithm for
 * combining accelerometer, gyroscope, and magnetometer data into a
 * stable orientation estimate.
 *
 * Reference: "An efficient orientation filter for inertial and
 * inertial/magnetic sensor arrays" - S. Madgwick, 2010
 */

#ifndef MADGWICK_H
#define MADGWICK_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the Madgwick filter
 * @param sample_freq Expected update frequency in Hz (e.g., 100)
 */
void madgwick_init(float sample_freq);

/**
 * @brief Set the filter gain (beta)
 * Higher values = faster convergence but more noise
 * Lower values = smoother but slower response
 * Typical: 0.01 - 0.5, default 0.1
 * @param beta Filter gain
 */
void madgwick_set_beta(float beta);

/**
 * @brief Get current filter gain
 * @return Current beta value
 */
float madgwick_get_beta(void);

/**
 * @brief Update filter with 9-axis sensor data (full MARG)
 *
 * @param gx Gyroscope X in radians/sec
 * @param gy Gyroscope Y in radians/sec
 * @param gz Gyroscope Z in radians/sec
 * @param ax Accelerometer X (normalized or g)
 * @param ay Accelerometer Y
 * @param az Accelerometer Z
 * @param mx Magnetometer X (any units, will be normalized)
 * @param my Magnetometer Y
 * @param mz Magnetometer Z
 * @param dt Delta time in seconds since last update
 */
void madgwick_update_9dof(float gx, float gy, float gz,
                          float ax, float ay, float az,
                          float mx, float my, float mz,
                          float dt);

/**
 * @brief Update filter with 6-axis sensor data (gyro + accel only)
 *
 * Use this when magnetometer is unavailable or unreliable.
 * Note: Heading will drift over time without magnetometer.
 *
 * @param gx Gyroscope X in radians/sec
 * @param gy Gyroscope Y in radians/sec
 * @param gz Gyroscope Z in radians/sec
 * @param ax Accelerometer X (normalized or g)
 * @param ay Accelerometer Y
 * @param az Accelerometer Z
 * @param dt Delta time in seconds since last update
 */
void madgwick_update_6dof(float gx, float gy, float gz,
                          float ax, float ay, float az,
                          float dt);

/**
 * @brief Get current quaternion orientation
 * @param q0 Scalar component (output)
 * @param q1 X component (output)
 * @param q2 Y component (output)
 * @param q3 Z component (output)
 */
void madgwick_get_quaternion(float *q0, float *q1, float *q2, float *q3);

/**
 * @brief Get yaw angle (heading) from quaternion
 * @return Yaw angle in degrees [0, 360)
 */
float madgwick_get_yaw(void);

/**
 * @brief Get pitch angle from quaternion
 * @return Pitch angle in degrees [-90, 90]
 */
float madgwick_get_pitch(void);

/**
 * @brief Get roll angle from quaternion
 * @return Roll angle in degrees [-180, 180]
 */
float madgwick_get_roll(void);

/**
 * @brief Check if filter has converged (has valid output)
 * @return true if filter has processed enough samples
 */
bool madgwick_is_converged(void);

/**
 * @brief Reset filter to initial state
 */
void madgwick_reset(void);

#ifdef __cplusplus
}
#endif

#endif // MADGWICK_H
