/**
 * @file param_store.h
 * @brief Centralized parameter storage with NVS persistence
 *
 * Provides runtime-configurable autopilot parameters accessible via
 * BLE, Console, and CAN. Parameters persist in NVS flash.
 *
 * FSD Reference: TestAPEN.FSD.v1.0.0.md Section 13.3
 */

#ifndef PARAM_STORE_H
#define PARAM_STORE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * Parameter IDs
 *============================================================================*/

typedef enum {
    // Master PID parameters (stored on Master NVS)
    PARAM_KP_HEADING = 0,
    PARAM_KI_HEADING,
    PARAM_KD_HEADING,

    // Rudder servo parameters (stored on Rudder NVS)
    PARAM_KP_SERVO,
    PARAM_DEADBAND_ENTER,
    PARAM_DEADBAND_EXIT,

    // Motor parameters (stored on Rudder NVS)
    PARAM_MIN_MOTOR_SPEED,
    PARAM_MAX_MOTOR_SPEED,
    PARAM_RUDDER_SLEW_RATE,

    // Rudder calibration (stored on Rudder NVS)
    PARAM_CAL_RAW_CENTER,       // Raw encoder value (0-4095) when rudder is centered

    // Magnetometer calibration (stored on Master NVS)
    // Hard-iron offsets (uT) - subtracted from raw readings
    PARAM_MAG_HARD_X,
    PARAM_MAG_HARD_Y,
    PARAM_MAG_HARD_Z,
    // Soft-iron 3x3 correction matrix (row-major order)
    // Applied as: corrected = matrix * (raw - hard_iron)
    PARAM_MAG_SOFT_00,  // Row 0
    PARAM_MAG_SOFT_01,
    PARAM_MAG_SOFT_02,
    PARAM_MAG_SOFT_10,  // Row 1
    PARAM_MAG_SOFT_11,
    PARAM_MAG_SOFT_12,
    PARAM_MAG_SOFT_20,  // Row 2
    PARAM_MAG_SOFT_21,
    PARAM_MAG_SOFT_22,

    PARAM_COUNT
} param_id_t;

/*============================================================================
 * Parameter Metadata
 *============================================================================*/

typedef struct {
    param_id_t id;
    const char *name;           // Human-readable name for console
    const char *nvs_key;        // NVS key (max 15 chars)
    float default_value;
    float min_value;
    float max_value;
    bool is_master_param;       // true = Master node, false = Rudder node
} param_meta_t;

/*============================================================================
 * API Functions
 *============================================================================*/

/**
 * @brief Initialize parameter store
 *
 * Loads parameters from NVS. If not found, uses defaults.
 * Must be called before any other param_store functions.
 *
 * @return ESP_OK on success
 */
esp_err_t param_store_init(void);

/**
 * @brief Get parameter value
 *
 * Thread-safe. Returns current runtime value.
 *
 * @param id Parameter ID
 * @return Current value (or default if invalid ID)
 */
float param_get(param_id_t id);

/**
 * @brief Set parameter value
 *
 * Validates against min/max bounds. Does NOT persist to NVS.
 * Call param_save() or param_save_all() to persist.
 *
 * @param id Parameter ID
 * @param value New value
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if out of bounds
 */
esp_err_t param_set(param_id_t id, float value);

/**
 * @brief Save single parameter to NVS
 *
 * @param id Parameter ID
 * @return ESP_OK on success
 */
esp_err_t param_save(param_id_t id);

/**
 * @brief Save all parameters to NVS
 *
 * @return ESP_OK on success
 */
esp_err_t param_save_all(void);

/**
 * @brief Reset all parameters to FSD defaults
 *
 * Also clears NVS storage.
 *
 * @return ESP_OK on success
 */
esp_err_t param_reset_defaults(void);

/**
 * @brief Get parameter metadata
 *
 * @param id Parameter ID
 * @return Pointer to metadata (NULL if invalid ID)
 */
const param_meta_t *param_get_meta(param_id_t id);

/**
 * @brief Find parameter by name
 *
 * @param name Parameter name (e.g., "kp_hdg")
 * @return Parameter ID, or PARAM_COUNT if not found
 */
param_id_t param_find_by_name(const char *name);

/**
 * @brief Set parameter change callback
 *
 * Called whenever a parameter value changes.
 *
 * @param cb Callback function (NULL to disable)
 */
void param_set_change_callback(void (*cb)(param_id_t id, float value));

/**
 * @brief Check if parameter belongs to this node
 *
 * Compares parameter's is_master_param flag against node type.
 *
 * @param id Parameter ID
 * @return true if this node should store/process this parameter
 */
bool param_is_local(param_id_t id);

#ifdef __cplusplus
}
#endif

#endif // PARAM_STORE_H
