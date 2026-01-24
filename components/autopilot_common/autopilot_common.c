/**
 * @file autopilot_common.c
 * @brief Common utility functions for TestAPEN Autopilot
 */

#include "autopilot_common.h"
#include <math.h>

float wrap180(float angle) {
    while (angle >= 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

float wrap360(float angle) {
    while (angle >= 360.0f) angle -= 360.0f;
    while (angle < 0.0f) angle += 360.0f;
    return angle;
}

float clampf(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

float clamp_delta(float value, float last, float max_delta) {
    float delta = value - last;
    if (delta > max_delta) return last + max_delta;
    if (delta < -max_delta) return last - max_delta;
    return value;
}

int signf(float value) {
    if (value > 0.0f) return 1;
    if (value < 0.0f) return -1;
    return 0;
}
