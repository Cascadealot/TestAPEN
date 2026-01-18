/**
 * @file param_store.c
 * @brief Centralized parameter storage with NVS persistence
 */

#include "param_store.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include <string.h>
#include <math.h>

static const char *TAG = "param_store";

/*============================================================================
 * Parameter Metadata Table
 *============================================================================*/

static const param_meta_t s_param_meta[PARAM_COUNT] = {
    // Master PID parameters
    [PARAM_KP_HEADING] = {
        .id = PARAM_KP_HEADING,
        .name = "kp_hdg",
        .nvs_key = "kp_hdg",
        .default_value = 0.8f,
        .min_value = 0.1f,
        .max_value = 5.0f,
        .is_master_param = true
    },
    [PARAM_KI_HEADING] = {
        .id = PARAM_KI_HEADING,
        .name = "ki_hdg",
        .nvs_key = "ki_hdg",
        .default_value = 0.05f,
        .min_value = 0.0f,
        .max_value = 1.0f,
        .is_master_param = true
    },
    [PARAM_KD_HEADING] = {
        .id = PARAM_KD_HEADING,
        .name = "kd_hdg",
        .nvs_key = "kd_hdg",
        .default_value = 0.5f,
        .min_value = 0.0f,
        .max_value = 5.0f,
        .is_master_param = true
    },

    // Rudder servo parameters
    [PARAM_KP_SERVO] = {
        .id = PARAM_KP_SERVO,
        .name = "kp_srv",
        .nvs_key = "kp_srv",
        .default_value = 10.0f,
        .min_value = 1.0f,
        .max_value = 50.0f,
        .is_master_param = false
    },
    [PARAM_DEADBAND_ENTER] = {
        .id = PARAM_DEADBAND_ENTER,
        .name = "db_ent",
        .nvs_key = "db_ent",
        .default_value = 1.0f,
        .min_value = 0.1f,
        .max_value = 5.0f,
        .is_master_param = false
    },
    [PARAM_DEADBAND_EXIT] = {
        .id = PARAM_DEADBAND_EXIT,
        .name = "db_ext",
        .nvs_key = "db_ext",
        .default_value = 1.5f,
        .min_value = 0.2f,
        .max_value = 6.0f,
        .is_master_param = false
    },

    // Motor parameters
    [PARAM_MIN_MOTOR_SPEED] = {
        .id = PARAM_MIN_MOTOR_SPEED,
        .name = "mot_min",
        .nvs_key = "mot_min",
        .default_value = 20.0f,
        .min_value = 0.0f,
        .max_value = 50.0f,
        .is_master_param = false
    },
    [PARAM_MAX_MOTOR_SPEED] = {
        .id = PARAM_MAX_MOTOR_SPEED,
        .name = "mot_max",
        .nvs_key = "mot_max",
        .default_value = 100.0f,
        .min_value = 50.0f,
        .max_value = 100.0f,
        .is_master_param = false
    },
    [PARAM_RUDDER_SLEW_RATE] = {
        .id = PARAM_RUDDER_SLEW_RATE,
        .name = "slew",
        .nvs_key = "slew",
        .default_value = 15.0f,
        .min_value = 1.0f,
        .max_value = 60.0f,
        .is_master_param = false
    },

    // Calibration - raw encoder value at center
    // Default -1.0 means "not calibrated, assume boot position is center"
    [PARAM_CAL_RAW_CENTER] = {
        .id = PARAM_CAL_RAW_CENTER,
        .name = "cal_ctr",
        .nvs_key = "cal_ctr",
        .default_value = -1.0f,
        .min_value = -1.0f,
        .max_value = 4095.0f,
        .is_master_param = false
    },

    // Magnetometer hard-iron offsets (uT)
    [PARAM_MAG_HARD_X] = {
        .id = PARAM_MAG_HARD_X,
        .name = "mag_hx",
        .nvs_key = "mag_hx",
        .default_value = 0.0f,
        .min_value = -200.0f,
        .max_value = 200.0f,
        .is_master_param = true
    },
    [PARAM_MAG_HARD_Y] = {
        .id = PARAM_MAG_HARD_Y,
        .name = "mag_hy",
        .nvs_key = "mag_hy",
        .default_value = 0.0f,
        .min_value = -200.0f,
        .max_value = 200.0f,
        .is_master_param = true
    },
    [PARAM_MAG_HARD_Z] = {
        .id = PARAM_MAG_HARD_Z,
        .name = "mag_hz",
        .nvs_key = "mag_hz",
        .default_value = 0.0f,
        .min_value = -200.0f,
        .max_value = 200.0f,
        .is_master_param = true
    },

    // Magnetometer soft-iron matrix (identity matrix default)
    // Row 0
    [PARAM_MAG_SOFT_00] = {
        .id = PARAM_MAG_SOFT_00,
        .name = "mag_s00",
        .nvs_key = "mag_s00",
        .default_value = 1.0f,
        .min_value = -2.0f,
        .max_value = 2.0f,
        .is_master_param = true
    },
    [PARAM_MAG_SOFT_01] = {
        .id = PARAM_MAG_SOFT_01,
        .name = "mag_s01",
        .nvs_key = "mag_s01",
        .default_value = 0.0f,
        .min_value = -2.0f,
        .max_value = 2.0f,
        .is_master_param = true
    },
    [PARAM_MAG_SOFT_02] = {
        .id = PARAM_MAG_SOFT_02,
        .name = "mag_s02",
        .nvs_key = "mag_s02",
        .default_value = 0.0f,
        .min_value = -2.0f,
        .max_value = 2.0f,
        .is_master_param = true
    },
    // Row 1
    [PARAM_MAG_SOFT_10] = {
        .id = PARAM_MAG_SOFT_10,
        .name = "mag_s10",
        .nvs_key = "mag_s10",
        .default_value = 0.0f,
        .min_value = -2.0f,
        .max_value = 2.0f,
        .is_master_param = true
    },
    [PARAM_MAG_SOFT_11] = {
        .id = PARAM_MAG_SOFT_11,
        .name = "mag_s11",
        .nvs_key = "mag_s11",
        .default_value = 1.0f,
        .min_value = -2.0f,
        .max_value = 2.0f,
        .is_master_param = true
    },
    [PARAM_MAG_SOFT_12] = {
        .id = PARAM_MAG_SOFT_12,
        .name = "mag_s12",
        .nvs_key = "mag_s12",
        .default_value = 0.0f,
        .min_value = -2.0f,
        .max_value = 2.0f,
        .is_master_param = true
    },
    // Row 2
    [PARAM_MAG_SOFT_20] = {
        .id = PARAM_MAG_SOFT_20,
        .name = "mag_s20",
        .nvs_key = "mag_s20",
        .default_value = 0.0f,
        .min_value = -2.0f,
        .max_value = 2.0f,
        .is_master_param = true
    },
    [PARAM_MAG_SOFT_21] = {
        .id = PARAM_MAG_SOFT_21,
        .name = "mag_s21",
        .nvs_key = "mag_s21",
        .default_value = 0.0f,
        .min_value = -2.0f,
        .max_value = 2.0f,
        .is_master_param = true
    },
    [PARAM_MAG_SOFT_22] = {
        .id = PARAM_MAG_SOFT_22,
        .name = "mag_s22",
        .nvs_key = "mag_s22",
        .default_value = 1.0f,
        .min_value = -2.0f,
        .max_value = 2.0f,
        .is_master_param = true
    },
};

/*============================================================================
 * Runtime Storage
 *============================================================================*/

static float s_param_values[PARAM_COUNT];
static bool s_initialized = false;
static void (*s_change_callback)(param_id_t id, float value) = NULL;

#define NVS_NAMESPACE "params"

/*============================================================================
 * Implementation
 *============================================================================*/

esp_err_t param_store_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    // Initialize NVS if not already done
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition erased, reinitializing");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(err));
        return err;
    }

    // Load defaults first
    for (int i = 0; i < PARAM_COUNT; i++) {
        s_param_values[i] = s_param_meta[i].default_value;
    }

    // Try to load from NVS
    nvs_handle_t nvs;
    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (err == ESP_OK) {
        for (int i = 0; i < PARAM_COUNT; i++) {
            // Only load parameters that belong to this node
            if (param_is_local(i)) {
                uint32_t raw;
                if (nvs_get_u32(nvs, s_param_meta[i].nvs_key, &raw) == ESP_OK) {
                    float value;
                    memcpy(&value, &raw, sizeof(float));
                    // Validate loaded value
                    if (value >= s_param_meta[i].min_value &&
                        value <= s_param_meta[i].max_value) {
                        s_param_values[i] = value;
                        ESP_LOGI(TAG, "Loaded %s = %.3f from NVS",
                                 s_param_meta[i].name, value);
                    } else {
                        ESP_LOGW(TAG, "Invalid NVS value for %s, using default",
                                 s_param_meta[i].name);
                    }
                }
            }
        }
        nvs_close(nvs);
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No saved parameters, using defaults");
    } else {
        ESP_LOGW(TAG, "NVS open failed: %s, using defaults", esp_err_to_name(err));
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Parameter store initialized");
    return ESP_OK;
}

float param_get(param_id_t id)
{
    if (id >= PARAM_COUNT) {
        ESP_LOGW(TAG, "Invalid param ID %d", id);
        return 0.0f;
    }

    if (!s_initialized) {
        ESP_LOGW(TAG, "param_store not initialized, returning default");
        return s_param_meta[id].default_value;
    }

    return s_param_values[id];
}

esp_err_t param_set(param_id_t id, float value)
{
    if (id >= PARAM_COUNT) {
        ESP_LOGE(TAG, "Invalid param ID %d", id);
        return ESP_ERR_INVALID_ARG;
    }

    const param_meta_t *meta = &s_param_meta[id];

    // Validate bounds
    if (value < meta->min_value || value > meta->max_value) {
        ESP_LOGE(TAG, "Value %.3f out of range [%.3f, %.3f] for %s",
                 value, meta->min_value, meta->max_value, meta->name);
        return ESP_ERR_INVALID_ARG;
    }

    float old_value = s_param_values[id];
    s_param_values[id] = value;

    ESP_LOGI(TAG, "Set %s: %.3f -> %.3f", meta->name, old_value, value);

    // Notify callback if value changed
    if (s_change_callback && fabsf(old_value - value) > 0.0001f) {
        s_change_callback(id, value);
    }

    return ESP_OK;
}

esp_err_t param_save(param_id_t id)
{
    if (id >= PARAM_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    // Only save parameters that belong to this node
    if (!param_is_local(id)) {
        ESP_LOGW(TAG, "Cannot save %s - not a local parameter",
                 s_param_meta[id].name);
        return ESP_ERR_INVALID_STATE;
    }

    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(err));
        return err;
    }

    uint32_t raw;
    memcpy(&raw, &s_param_values[id], sizeof(float));
    err = nvs_set_u32(nvs, s_param_meta[id].nvs_key, raw);
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }

    nvs_close(nvs);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Saved %s = %.3f to NVS",
                 s_param_meta[id].name, s_param_values[id]);
    } else {
        ESP_LOGE(TAG, "Failed to save %s: %s",
                 s_param_meta[id].name, esp_err_to_name(err));
    }

    return err;
}

esp_err_t param_save_all(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(err));
        return err;
    }

    int saved_count = 0;
    for (int i = 0; i < PARAM_COUNT; i++) {
        // Only save parameters that belong to this node
        if (param_is_local(i)) {
            uint32_t raw;
            memcpy(&raw, &s_param_values[i], sizeof(float));
            if (nvs_set_u32(nvs, s_param_meta[i].nvs_key, raw) == ESP_OK) {
                saved_count++;
            }
        }
    }

    err = nvs_commit(nvs);
    nvs_close(nvs);

    ESP_LOGI(TAG, "Saved %d parameters to NVS", saved_count);
    return err;
}

esp_err_t param_reset_defaults(void)
{
    // Reset runtime values
    for (int i = 0; i < PARAM_COUNT; i++) {
        float old_value = s_param_values[i];
        s_param_values[i] = s_param_meta[i].default_value;

        // Notify callback for local params that changed
        if (param_is_local(i) && s_change_callback &&
            fabsf(old_value - s_param_values[i]) > 0.0001f) {
            s_change_callback(i, s_param_values[i]);
        }
    }

    // Clear NVS
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        nvs_erase_all(nvs);
        nvs_commit(nvs);
        nvs_close(nvs);
    }

    ESP_LOGI(TAG, "All parameters reset to defaults");
    return ESP_OK;
}

const param_meta_t *param_get_meta(param_id_t id)
{
    if (id >= PARAM_COUNT) {
        return NULL;
    }
    return &s_param_meta[id];
}

param_id_t param_find_by_name(const char *name)
{
    if (name == NULL) {
        return PARAM_COUNT;
    }

    for (int i = 0; i < PARAM_COUNT; i++) {
        if (strcmp(s_param_meta[i].name, name) == 0) {
            return (param_id_t)i;
        }
    }

    return PARAM_COUNT;
}

void param_set_change_callback(void (*cb)(param_id_t id, float value))
{
    s_change_callback = cb;
}

bool param_is_local(param_id_t id)
{
    if (id >= PARAM_COUNT) {
        return false;
    }

#ifdef CONFIG_TESTAP2_NODE_MASTER
    return s_param_meta[id].is_master_param;
#else
    return !s_param_meta[id].is_master_param;
#endif
}
