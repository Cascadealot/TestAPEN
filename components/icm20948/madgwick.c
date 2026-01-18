/**
 * @file madgwick.c
 * @brief Madgwick AHRS Filter Implementation
 *
 * Based on Sebastian Madgwick's IMU sensor fusion algorithm.
 * Optimized for embedded systems (no dynamic allocation).
 */

#include "madgwick.h"
#include <math.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Filter state
static float g_q0 = 1.0f, g_q1 = 0.0f, g_q2 = 0.0f, g_q3 = 0.0f;  // Quaternion
static float g_beta = 0.1f;  // Filter gain
static float g_sample_freq = 100.0f;
static int g_sample_count = 0;

// Fast inverse square root using union for proper type punning
static float inv_sqrt(float x) {
    union {
        float f;
        uint32_t i;
    } conv;

    float halfx = 0.5f * x;
    conv.f = x;
    conv.i = 0x5f3759df - (conv.i >> 1);
    conv.f = conv.f * (1.5f - (halfx * conv.f * conv.f));  // Newton step
    conv.f = conv.f * (1.5f - (halfx * conv.f * conv.f));  // Second step
    return conv.f;
}

void madgwick_init(float sample_freq) {
    g_sample_freq = sample_freq;
    g_q0 = 1.0f;
    g_q1 = 0.0f;
    g_q2 = 0.0f;
    g_q3 = 0.0f;
    g_sample_count = 0;
}

void madgwick_set_beta(float beta) {
    if (beta > 0.0f && beta < 10.0f) {
        g_beta = beta;
    }
}

float madgwick_get_beta(void) {
    return g_beta;
}

void madgwick_update_9dof(float gx, float gy, float gz,
                          float ax, float ay, float az,
                          float mx, float my, float mz,
                          float dt) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx;
    float _2bx, _2bz, _4bx, _4bz;
    float _2q0, _2q1, _2q2, _2q3;
    float _2q0q2, _2q2q3;
    float q0q0, q0q1, q0q2, q0q3;
    float q1q1, q1q2, q1q3;
    float q2q2, q2q3;
    float q3q3;

    // Use default sample frequency if dt not provided
    if (dt <= 0.0f) {
        dt = 1.0f / g_sample_freq;
    }

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-g_q1 * gx - g_q2 * gy - g_q3 * gz);
    qDot2 = 0.5f * (g_q0 * gx + g_q2 * gz - g_q3 * gy);
    qDot3 = 0.5f * (g_q0 * gy - g_q1 * gz + g_q3 * gx);
    qDot4 = 0.5f * (g_q0 * gz + g_q1 * gy - g_q2 * gx);

    // Compute feedback only if accelerometer measurement valid
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalise accelerometer measurement
        recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = inv_sqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * g_q0 * mx;
        _2q0my = 2.0f * g_q0 * my;
        _2q0mz = 2.0f * g_q0 * mz;
        _2q1mx = 2.0f * g_q1 * mx;
        _2q0 = 2.0f * g_q0;
        _2q1 = 2.0f * g_q1;
        _2q2 = 2.0f * g_q2;
        _2q3 = 2.0f * g_q3;
        _2q0q2 = 2.0f * g_q0 * g_q2;
        _2q2q3 = 2.0f * g_q2 * g_q3;
        q0q0 = g_q0 * g_q0;
        q0q1 = g_q0 * g_q1;
        q0q2 = g_q0 * g_q2;
        q0q3 = g_q0 * g_q3;
        q1q1 = g_q1 * g_q1;
        q1q2 = g_q1 * g_q2;
        q1q3 = g_q1 * g_q3;
        q2q2 = g_q2 * g_q2;
        q2q3 = g_q2 * g_q3;
        q3q3 = g_q3 * g_q3;

        // Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * g_q3 + _2q0mz * g_q2 + mx * q1q1 +
             _2q1 * my * g_q2 + _2q1 * mz * g_q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * g_q3 + my * q0q0 - _2q0mz * g_q1 + _2q1mx * g_q2 -
             my * q1q1 + my * q2q2 + _2q2 * mz * g_q3 - my * q3q3;
        _2bx = sqrtf(hx * hx + hy * hy);
        _2bz = -_2q0mx * g_q2 + _2q0my * g_q1 + mz * q0q0 + _2q1mx * g_q3 -
               mz * q1q1 + _2q2 * my * g_q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient descent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) +
             _2q1 * (2.0f * q0q1 + _2q2q3 - ay) -
             _2bz * g_q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
             (-_2bx * g_q3 + _2bz * g_q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
             _2bx * g_q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) +
             _2q0 * (2.0f * q0q1 + _2q2q3 - ay) -
             4.0f * g_q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) +
             _2bz * g_q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
             (_2bx * g_q2 + _2bz * g_q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
             (_2bx * g_q3 - _4bz * g_q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) +
             _2q3 * (2.0f * q0q1 + _2q2q3 - ay) -
             4.0f * g_q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) +
             (-_4bx * g_q2 - _2bz * g_q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
             (_2bx * g_q1 + _2bz * g_q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
             (_2bx * g_q0 - _4bz * g_q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) +
             _2q2 * (2.0f * q0q1 + _2q2q3 - ay) +
             (-_4bx * g_q3 + _2bz * g_q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
             (-_2bx * g_q0 + _2bz * g_q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
             _2bx * g_q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

        // Normalise step magnitude
        recipNorm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= g_beta * s0;
        qDot2 -= g_beta * s1;
        qDot3 -= g_beta * s2;
        qDot4 -= g_beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    g_q0 += qDot1 * dt;
    g_q1 += qDot2 * dt;
    g_q2 += qDot3 * dt;
    g_q3 += qDot4 * dt;

    // Normalise quaternion
    recipNorm = inv_sqrt(g_q0 * g_q0 + g_q1 * g_q1 + g_q2 * g_q2 + g_q3 * g_q3);
    g_q0 *= recipNorm;
    g_q1 *= recipNorm;
    g_q2 *= recipNorm;
    g_q3 *= recipNorm;

    g_sample_count++;
}

void madgwick_update_6dof(float gx, float gy, float gz,
                          float ax, float ay, float az,
                          float dt) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3;
    float _4q0, _4q1, _4q2;
    float _8q1, _8q2;
    float q0q0, q1q1, q2q2, q3q3;

    // Use default sample frequency if dt not provided
    if (dt <= 0.0f) {
        dt = 1.0f / g_sample_freq;
    }

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-g_q1 * gx - g_q2 * gy - g_q3 * gz);
    qDot2 = 0.5f * (g_q0 * gx + g_q2 * gz - g_q3 * gy);
    qDot3 = 0.5f * (g_q0 * gy - g_q1 * gz + g_q3 * gx);
    qDot4 = 0.5f * (g_q0 * gz + g_q1 * gy - g_q2 * gx);

    // Compute feedback only if accelerometer measurement valid
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalise accelerometer measurement
        recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * g_q0;
        _2q1 = 2.0f * g_q1;
        _2q2 = 2.0f * g_q2;
        _2q3 = 2.0f * g_q3;
        _4q0 = 4.0f * g_q0;
        _4q1 = 4.0f * g_q1;
        _4q2 = 4.0f * g_q2;
        _8q1 = 8.0f * g_q1;
        _8q2 = 8.0f * g_q2;
        q0q0 = g_q0 * g_q0;
        q1q1 = g_q1 * g_q1;
        q2q2 = g_q2 * g_q2;
        q3q3 = g_q3 * g_q3;

        // Gradient descent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * g_q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * g_q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * g_q3 - _2q1 * ax + 4.0f * q2q2 * g_q3 - _2q2 * ay;

        // Normalise step magnitude
        recipNorm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= g_beta * s0;
        qDot2 -= g_beta * s1;
        qDot3 -= g_beta * s2;
        qDot4 -= g_beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    g_q0 += qDot1 * dt;
    g_q1 += qDot2 * dt;
    g_q2 += qDot3 * dt;
    g_q3 += qDot4 * dt;

    // Normalise quaternion
    recipNorm = inv_sqrt(g_q0 * g_q0 + g_q1 * g_q1 + g_q2 * g_q2 + g_q3 * g_q3);
    g_q0 *= recipNorm;
    g_q1 *= recipNorm;
    g_q2 *= recipNorm;
    g_q3 *= recipNorm;

    g_sample_count++;
}

void madgwick_get_quaternion(float *q0, float *q1, float *q2, float *q3) {
    if (q0) *q0 = g_q0;
    if (q1) *q1 = g_q1;
    if (q2) *q2 = g_q2;
    if (q3) *q3 = g_q3;
}

float madgwick_get_yaw(void) {
    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (g_q0 * g_q3 + g_q1 * g_q2);
    float cosy_cosp = 1.0f - 2.0f * (g_q2 * g_q2 + g_q3 * g_q3);
    float yaw = atan2f(siny_cosp, cosy_cosp) * 180.0f / M_PI;

    // Convert to 0-360 range
    if (yaw < 0) yaw += 360.0f;
    return yaw;
}

float madgwick_get_pitch(void) {
    // Pitch (y-axis rotation)
    float sinp = 2.0f * (g_q0 * g_q2 - g_q3 * g_q1);
    float pitch;
    if (fabsf(sinp) >= 1.0f) {
        pitch = copysignf(M_PI / 2.0f, sinp);  // Use 90 degrees if out of range
    } else {
        pitch = asinf(sinp);
    }
    return pitch * 180.0f / M_PI;
}

float madgwick_get_roll(void) {
    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (g_q0 * g_q1 + g_q2 * g_q3);
    float cosr_cosp = 1.0f - 2.0f * (g_q1 * g_q1 + g_q2 * g_q2);
    float roll = atan2f(sinr_cosp, cosr_cosp);
    return roll * 180.0f / M_PI;
}

bool madgwick_is_converged(void) {
    // Consider converged after ~1 second at 100Hz
    return g_sample_count >= 100;
}

void madgwick_reset(void) {
    g_q0 = 1.0f;
    g_q1 = 0.0f;
    g_q2 = 0.0f;
    g_q3 = 0.0f;
    g_sample_count = 0;
}
