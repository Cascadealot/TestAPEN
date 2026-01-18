/**
 * @file as5600.c
 * @brief AS5600 Magnetic Rotary Encoder Driver Implementation
 *
 * FSD Reference: TestAP2.FSD.v1.0.0.md Section 4.2, 9.3
 */

#include "as5600.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "AS5600";

// Module state
static struct {
    i2c_port_t i2c_num;
    SemaphoreHandle_t i2c_mutex;
    bool initialized;
    as5600_data_t data;
    uint16_t zero_position;
} g_as5600 = {
    .initialized = false,
    .zero_position = 0
};

// I2C timeout
#define I2C_TIMEOUT_MS      50
#define I2C_TIMEOUT_TICKS   pdMS_TO_TICKS(I2C_TIMEOUT_MS)

/*============================================================================
 * Internal I2C Functions
 *============================================================================*/

/**
 * @brief Read bytes from AS5600 register
 */
static esp_err_t as5600_read_reg(uint8_t reg, uint8_t *data, size_t len) {
    if (!g_as5600.initialized && reg != AS5600_REG_STATUS) {
        return ESP_ERR_INVALID_STATE;
    }

    // Acquire mutex if available
    if (g_as5600.i2c_mutex != NULL) {
        if (xSemaphoreTake(g_as5600.i2c_mutex, I2C_TIMEOUT_TICKS) != pdTRUE) {
            ESP_LOGW(TAG, "I2C mutex timeout");
            return ESP_ERR_TIMEOUT;
        }
    }

    // Build I2C command: write register address, then read data
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS5600_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);  // Repeated start
    i2c_master_write_byte(cmd, (AS5600_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(g_as5600.i2c_num, cmd, I2C_TIMEOUT_TICKS);
    i2c_cmd_link_delete(cmd);

    // Release mutex
    if (g_as5600.i2c_mutex != NULL) {
        xSemaphoreGive(g_as5600.i2c_mutex);
    }

    if (err != ESP_OK) {
        ESP_LOGD(TAG, "I2C read failed: reg=0x%02X, err=%s", reg, esp_err_to_name(err));
    }

    return err;
}

/**
 * @brief Write bytes to AS5600 register
 */
static esp_err_t as5600_write_reg(uint8_t reg, const uint8_t *data, size_t len) {
    if (!g_as5600.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Acquire mutex if available
    if (g_as5600.i2c_mutex != NULL) {
        if (xSemaphoreTake(g_as5600.i2c_mutex, I2C_TIMEOUT_TICKS) != pdTRUE) {
            ESP_LOGW(TAG, "I2C mutex timeout");
            return ESP_ERR_TIMEOUT;
        }
    }

    // Build I2C command
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS5600_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(g_as5600.i2c_num, cmd, I2C_TIMEOUT_TICKS);
    i2c_cmd_link_delete(cmd);

    // Release mutex
    if (g_as5600.i2c_mutex != NULL) {
        xSemaphoreGive(g_as5600.i2c_mutex);
    }

    if (err != ESP_OK) {
        ESP_LOGD(TAG, "I2C write failed: reg=0x%02X, err=%s", reg, esp_err_to_name(err));
    }

    return err;
}

/*============================================================================
 * Public API
 *============================================================================*/

esp_err_t as5600_init(i2c_port_t i2c_num, SemaphoreHandle_t i2c_mutex) {
    ESP_LOGI(TAG, "Initializing AS5600 encoder");

    g_as5600.i2c_num = i2c_num;
    g_as5600.i2c_mutex = i2c_mutex;
    g_as5600.initialized = true;  // Temporarily set for read to work
    g_as5600.zero_position = 0;

    memset(&g_as5600.data, 0, sizeof(g_as5600.data));

    // Verify device presence by reading status register
    uint8_t status;
    esp_err_t err = as5600_read_reg(AS5600_REG_STATUS, &status, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to communicate with AS5600 at address 0x%02X", AS5600_I2C_ADDR);
        g_as5600.initialized = false;
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "AS5600 status register: 0x%02X", status);

    // Check magnet status (FSD Section 9.3)
    bool magnet_detected = (status & AS5600_STATUS_MD) != 0;
    bool magnet_too_weak = (status & AS5600_STATUS_ML) != 0;
    bool magnet_too_strong = (status & AS5600_STATUS_MH) != 0;

    if (!magnet_detected) {
        ESP_LOGE(TAG, "Magnet not detected (MD=0) - check magnet placement");
        g_as5600.initialized = false;
        return ESP_ERR_INVALID_STATE;
    }

    if (magnet_too_weak) {
        ESP_LOGW(TAG, "Magnet too weak (ML=1) - consider moving magnet closer");
    }

    if (magnet_too_strong) {
        ESP_LOGW(TAG, "Magnet too strong (MH=1) - consider moving magnet farther");
    }

    // Read initial angle
    uint16_t raw_angle;
    err = as5600_read_raw_angle(&raw_angle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read initial angle");
        g_as5600.initialized = false;
        return err;
    }

    g_as5600.data.raw_angle = raw_angle;
    g_as5600.data.angle_degrees = AS5600_RAW_TO_DEGREES(raw_angle);
    g_as5600.data.valid = true;
    g_as5600.data.last_update_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    ESP_LOGI(TAG, "AS5600 initialized - initial angle: %.1f deg (raw: %u)",
             g_as5600.data.angle_degrees, raw_angle);

    return ESP_OK;
}

esp_err_t as5600_read_raw_angle(uint16_t *raw_angle) {
    if (raw_angle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buf[2];
    esp_err_t err = as5600_read_reg(AS5600_REG_RAW_ANGLE_H, buf, 2);
    if (err != ESP_OK) {
        return err;
    }

    // 12-bit value: high byte [11:8], low byte [7:0]
    *raw_angle = ((uint16_t)(buf[0] & 0x0F) << 8) | buf[1];
    return ESP_OK;
}

esp_err_t as5600_read_angle_degrees(float *degrees) {
    if (degrees == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t raw;
    esp_err_t err = as5600_read_raw_angle(&raw);
    if (err != ESP_OK) {
        return err;
    }

    // Apply zero position offset
    int32_t adjusted = (int32_t)raw - (int32_t)g_as5600.zero_position;
    if (adjusted < 0) {
        adjusted += 4096;
    }

    *degrees = AS5600_RAW_TO_DEGREES(adjusted);
    return ESP_OK;
}

esp_err_t as5600_read_status(as5600_status_t *status) {
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Read status register
    uint8_t status_reg;
    esp_err_t err = as5600_read_reg(AS5600_REG_STATUS, &status_reg, 1);
    if (err != ESP_OK) {
        return err;
    }

    status->magnet_detected = (status_reg & AS5600_STATUS_MD) != 0;
    status->magnet_too_weak = (status_reg & AS5600_STATUS_ML) != 0;
    status->magnet_too_strong = (status_reg & AS5600_STATUS_MH) != 0;

    // Read AGC
    err = as5600_read_reg(AS5600_REG_AGC, &status->agc, 1);
    if (err != ESP_OK) {
        return err;
    }

    // Read magnitude
    uint8_t mag_buf[2];
    err = as5600_read_reg(AS5600_REG_MAGNITUDE_H, mag_buf, 2);
    if (err != ESP_OK) {
        return err;
    }
    status->magnitude = ((uint16_t)(mag_buf[0] & 0x0F) << 8) | mag_buf[1];

    return ESP_OK;
}

bool as5600_is_magnet_detected(void) {
    if (!g_as5600.initialized) {
        return false;
    }

    uint8_t status;
    if (as5600_read_reg(AS5600_REG_STATUS, &status, 1) != ESP_OK) {
        return false;
    }

    return (status & AS5600_STATUS_MD) != 0;
}

esp_err_t as5600_update(as5600_data_t *data) {
    if (!g_as5600.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Read raw angle
    uint16_t raw;
    esp_err_t err = as5600_read_raw_angle(&raw);
    if (err != ESP_OK) {
        g_as5600.data.valid = false;
        if (data != NULL) {
            *data = g_as5600.data;
        }
        return err;
    }

    // Apply zero position offset and convert to degrees
    int32_t adjusted = (int32_t)raw - (int32_t)g_as5600.zero_position;
    if (adjusted < 0) {
        adjusted += 4096;
    }

    g_as5600.data.raw_angle = raw;
    g_as5600.data.angle_degrees = AS5600_RAW_TO_DEGREES(adjusted);
    g_as5600.data.valid = true;
    g_as5600.data.last_update_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // Read status periodically (not every cycle to reduce I2C traffic)
    static uint8_t status_counter = 0;
    if (++status_counter >= 25) {  // Every 25 calls (~500ms at 50Hz)
        status_counter = 0;
        as5600_read_status(&g_as5600.data.status);

        // Check for magnet issues (FSD Section 9.3)
        if (!g_as5600.data.status.magnet_detected) {
            ESP_LOGE(TAG, "Magnet not detected!");
            g_as5600.data.valid = false;
        }
    }

    if (data != NULL) {
        *data = g_as5600.data;
    }

    return ESP_OK;
}

float as5600_get_angle(void) {
    return g_as5600.data.angle_degrees;
}

bool as5600_is_valid(void) {
    if (!g_as5600.initialized || !g_as5600.data.valid) {
        return false;
    }

    // Check if data is stale (>100ms old)
    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t age_ms = now_ms - g_as5600.data.last_update_ms;
    return age_ms < 100;
}

esp_err_t as5600_set_zero_position(uint16_t raw_offset) {
    g_as5600.zero_position = raw_offset & 0x0FFF;  // Mask to 12 bits
    ESP_LOGI(TAG, "Zero position set to %u (%.1f deg)",
             g_as5600.zero_position,
             AS5600_RAW_TO_DEGREES(g_as5600.zero_position));
    return ESP_OK;
}

uint16_t as5600_get_zero_position(void) {
    return g_as5600.zero_position;
}
