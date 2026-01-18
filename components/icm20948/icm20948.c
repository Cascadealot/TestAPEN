/**
 * @file icm20948.c
 * @brief ICM-20948 9-axis IMU Driver Implementation
 */

#include "icm20948.h"
#include "madgwick.h"
#include "param_store.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

static const char *TAG = "ICM20948";

// ICM-20948 Registers (Bank 0)
#define REG_WHO_AM_I            0x00
#define REG_USER_CTRL           0x03
#define REG_LP_CONFIG           0x05
#define REG_PWR_MGMT_1          0x06
#define REG_PWR_MGMT_2          0x07
#define REG_INT_PIN_CFG         0x0F
#define REG_INT_ENABLE          0x10
#define REG_ACCEL_XOUT_H        0x2D
#define REG_GYRO_XOUT_H         0x33
#define REG_EXT_SLV_SENS_DATA_00 0x3B
#define REG_BANK_SEL            0x7F

// ICM-20948 Registers (Bank 2)
#define REG_GYRO_CONFIG_1       0x01
#define REG_ACCEL_CONFIG        0x14

// AK09916 Magnetometer Registers
#define AK_WHO_AM_I             0x01
#define AK_ST1                  0x10
#define AK_HXL                  0x11
#define AK_ST2                  0x18
#define AK_CNTL2                0x31
#define AK_CNTL3                0x32

// Expected WHO_AM_I values
#define ICM20948_WHO_AM_I_VAL   0xEA
#define AK09916_WHO_AM_I_VAL    0x09

// Scale factors
#define ACCEL_SCALE_2G          (1.0f / 16384.0f)   // ±2g
#define GYRO_SCALE_250DPS       (1.0f / 131.0f)     // ±250°/s
#define MAG_SCALE_UT            0.15f               // 0.15 µT/LSB

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// State
static i2c_port_t g_i2c_num = I2C_NUM_0;
static SemaphoreHandle_t g_i2c_mutex = NULL;
static icm20948_data_t g_data;
static SemaphoreHandle_t g_data_mutex = NULL;

// Magnetometer calibration
// Hard-iron offsets (subtracted from raw uT values)
static float g_mag_offset_x = 0.0f;
static float g_mag_offset_y = 0.0f;
static float g_mag_offset_z = 0.0f;

// Soft-iron 3x3 correction matrix (identity by default)
// Applied as: corrected = matrix * (raw - hard_iron)
static float g_mag_soft[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
};

// Heading filter state (EMA on unit circle)
static float g_filter_sin = 0.0f;
static float g_filter_cos = 1.0f;
static bool g_filter_init = false;
static uint32_t g_last_update = 0;
static float g_filter_tau_ms = 100.0f;  // Filter time constant

// Simulation mode
static bool g_simulation = false;
static float g_sim_heading = 0.0f;

// Accelerometer calibration (offsets in g)
static float g_accel_offset_x = 0.0f;
static float g_accel_offset_y = 0.0f;
static float g_accel_offset_z = 0.0f;

// Gyroscope bias (in deg/s)
static float g_gyro_bias_x = 0.0f;
static float g_gyro_bias_y = 0.0f;
static float g_gyro_bias_z = 0.0f;

// Guided magnetometer calibration state
static bool g_magcal_running = false;
static uint32_t g_magcal_sample_count = 0;
static float g_magcal_min_x, g_magcal_max_x;
static float g_magcal_min_y, g_magcal_max_y;
static float g_magcal_min_z, g_magcal_max_z;
static float g_magcal_quality = -1.0f;

// Sensor fusion mode
static bool g_fusion_enabled = true;  // Enable Madgwick fusion by default
static float g_fusion_beta = 0.1f;    // Default filter gain

// GPS-aided calibration monitoring
static float g_gps_offset_sum_sin = 0.0f;  // Circular averaging
static float g_gps_offset_sum_cos = 0.0f;
static uint32_t g_gps_offset_count = 0;
static float g_last_mag_heading = 0.0f;
#define GPS_OFFSET_MIN_SPEED_KNOTS  3.0f   // Minimum speed for valid COG

/*============================================================================
 * I2C Helpers
 *============================================================================*/

static bool i2c_acquire(uint32_t timeout_ms) {
    if (g_i2c_mutex == NULL) return true;
    return xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

static void i2c_release(void) {
    if (g_i2c_mutex != NULL) {
        xSemaphoreGive(g_i2c_mutex);
    }
}

static esp_err_t i2c_write_byte(uint8_t addr, uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    return i2c_master_write_to_device(g_i2c_num, addr, buf, 2, pdMS_TO_TICKS(100));
}

static esp_err_t i2c_read_bytes(uint8_t addr, uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(g_i2c_num, addr, &reg, 1, data, len, pdMS_TO_TICKS(100));
}

static esp_err_t icm_write_reg(uint8_t reg, uint8_t data) {
    return i2c_write_byte(ICM20948_I2C_ADDR, reg, data);
}

static esp_err_t icm_read_reg(uint8_t reg, uint8_t *data) {
    return i2c_read_bytes(ICM20948_I2C_ADDR, reg, data, 1);
}

static esp_err_t icm_read_bytes(uint8_t reg, uint8_t *data, size_t len) {
    return i2c_read_bytes(ICM20948_I2C_ADDR, reg, data, len);
}

static esp_err_t icm_set_bank(uint8_t bank) {
    return icm_write_reg(REG_BANK_SEL, (bank & 0x03) << 4);
}

static esp_err_t ak_write_reg(uint8_t reg, uint8_t data) {
    return i2c_write_byte(AK09916_I2C_ADDR, reg, data);
}

static esp_err_t ak_read_bytes(uint8_t reg, uint8_t *data, size_t len) {
    return i2c_read_bytes(AK09916_I2C_ADDR, reg, data, len);
}

/*============================================================================
 * Heading Computation
 *============================================================================*/

static float clamp360(float d) {
    while (d < 0.0f) d += 360.0f;
    while (d >= 360.0f) d -= 360.0f;
    return d;
}

static float compute_tilt_compensated_heading(
    float ax, float ay, float az,
    float mx, float my, float mz)
{
    // Normalize accelerometer
    float an = sqrtf(ax*ax + ay*ay + az*az);
    if (an > 0.01f) {
        ax /= an;
        ay /= an;
        az /= an;
    }

    // Compute roll and pitch from accelerometer
    float roll = atan2f(ay, az);
    float pitch = atanf(-ax / sqrtf(ay*ay + az*az + 1e-9f));

    // Tilt-compensate magnetometer
    float mx2 = mx * cosf(pitch) + mz * sinf(pitch);
    float my2 = mx * sinf(roll) * sinf(pitch) + my * cosf(roll) - mz * sinf(roll) * cosf(pitch);

    // Compute heading (0 = North, clockwise positive)
    float heading = atan2f(-my2, mx2) * 180.0f / M_PI;
    if (heading < 0) heading += 360.0f;

    return heading;
}

static float filter_heading(float raw, uint32_t now_ms) {
    float dt_ms = (float)(now_ms - g_last_update);
    if (dt_ms < 1.0f) dt_ms = 1.0f;
    if (dt_ms > 500.0f) dt_ms = 500.0f;
    g_last_update = now_ms;

    float r = raw * M_PI / 180.0f;
    float s = sinf(r);
    float c = cosf(r);

    if (!g_filter_init) {
        g_filter_sin = s;
        g_filter_cos = c;
        g_filter_init = true;
        return raw;
    }

    float tau = g_filter_tau_ms / 1000.0f;
    float alpha = 1.0f - expf(-dt_ms / 1000.0f / tau);

    g_filter_sin = (1.0f - alpha) * g_filter_sin + alpha * s;
    g_filter_cos = (1.0f - alpha) * g_filter_cos + alpha * c;

    float n = sqrtf(g_filter_sin * g_filter_sin + g_filter_cos * g_filter_cos);
    if (n > 1e-6f) {
        g_filter_sin /= n;
        g_filter_cos /= n;
    }

    float filtered = atan2f(g_filter_sin, g_filter_cos) * 180.0f / M_PI;
    return clamp360(filtered);
}

/*============================================================================
 * Public API
 *============================================================================*/

esp_err_t icm20948_init(i2c_port_t i2c_num, SemaphoreHandle_t i2c_mutex) {
    esp_err_t err;
    uint8_t who_am_i;

    g_i2c_num = i2c_num;
    g_i2c_mutex = i2c_mutex;

    // Create data mutex
    g_data_mutex = xSemaphoreCreateMutex();
    if (g_data_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    memset(&g_data, 0, sizeof(g_data));

    if (!i2c_acquire(500)) {
        ESP_LOGE(TAG, "Failed to acquire I2C");
        return ESP_ERR_TIMEOUT;
    }

    // Select bank 0
    err = icm_set_bank(0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set bank 0");
        i2c_release();
        return err;
    }

    // Read WHO_AM_I
    err = icm_read_reg(REG_WHO_AM_I, &who_am_i);
    if (err != ESP_OK || who_am_i != ICM20948_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "WHO_AM_I mismatch: got 0x%02X, expected 0x%02X", who_am_i, ICM20948_WHO_AM_I_VAL);
        i2c_release();
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "ICM-20948 found (WHO_AM_I=0x%02X)", who_am_i);

    // Reset device
    err = icm_write_reg(REG_PWR_MGMT_1, 0x80);
    if (err != ESP_OK) {
        i2c_release();
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    // Wake up, auto-select clock
    err = icm_write_reg(REG_PWR_MGMT_1, 0x01);
    if (err != ESP_OK) {
        i2c_release();
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    // Enable all sensors
    err = icm_write_reg(REG_PWR_MGMT_2, 0x00);
    if (err != ESP_OK) {
        i2c_release();
        return err;
    }

    // Configure accelerometer (Bank 2)
    err = icm_set_bank(2);
    if (err != ESP_OK) {
        i2c_release();
        return err;
    }

    // Accel: ±2g, DLPF enabled
    err = icm_write_reg(REG_ACCEL_CONFIG, 0x01);
    if (err != ESP_OK) {
        i2c_release();
        return err;
    }

    // Gyro: ±250°/s, DLPF enabled
    err = icm_write_reg(REG_GYRO_CONFIG_1, 0x01);
    if (err != ESP_OK) {
        i2c_release();
        return err;
    }

    // Back to bank 0
    err = icm_set_bank(0);
    if (err != ESP_OK) {
        i2c_release();
        return err;
    }

    // Enable I2C bypass to access magnetometer directly
    err = icm_write_reg(REG_INT_PIN_CFG, 0x02);
    if (err != ESP_OK) {
        i2c_release();
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // Now we can access AK09916 directly
    // Reset magnetometer
    err = ak_write_reg(AK_CNTL3, 0x01);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "AK09916 reset failed (may be OK)");
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    // Check magnetometer WHO_AM_I
    uint8_t ak_id;
    err = ak_read_bytes(AK_WHO_AM_I, &ak_id, 1);
    if (err == ESP_OK && ak_id == AK09916_WHO_AM_I_VAL) {
        ESP_LOGI(TAG, "AK09916 magnetometer found (ID=0x%02X)", ak_id);
    } else {
        ESP_LOGW(TAG, "AK09916 check: err=%d, id=0x%02X", err, ak_id);
    }

    // Set magnetometer to continuous mode 100Hz
    err = ak_write_reg(AK_CNTL2, 0x08);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "AK09916 mode set failed");
    }

    i2c_release();

    // Initialize Madgwick sensor fusion filter (100Hz sample rate)
    madgwick_init(100.0f);
    madgwick_set_beta(g_fusion_beta);
    ESP_LOGI(TAG, "Madgwick sensor fusion initialized (beta=%.2f)", g_fusion_beta);

    ESP_LOGI(TAG, "ICM-20948 initialization complete");
    return ESP_OK;
}

esp_err_t icm20948_update(void) {
    esp_err_t err;
    uint8_t buf[14];
    int16_t raw[7];

    if (!i2c_acquire(50)) {
        return ESP_ERR_TIMEOUT;
    }

    // Ensure bank 0
    icm_set_bank(0);

    // Read accelerometer and gyroscope (12 bytes from 0x2D)
    err = icm_read_bytes(REG_ACCEL_XOUT_H, buf, 12);
    if (err != ESP_OK) {
        i2c_release();
        return err;
    }

    // Parse accelerometer (big-endian)
    raw[0] = (int16_t)((buf[0] << 8) | buf[1]);   // Accel X
    raw[1] = (int16_t)((buf[2] << 8) | buf[3]);   // Accel Y
    raw[2] = (int16_t)((buf[4] << 8) | buf[5]);   // Accel Z

    // Parse gyroscope (big-endian)
    raw[3] = (int16_t)((buf[6] << 8) | buf[7]);   // Gyro X
    raw[4] = (int16_t)((buf[8] << 8) | buf[9]);   // Gyro Y
    raw[5] = (int16_t)((buf[10] << 8) | buf[11]); // Gyro Z

    // Read magnetometer (via I2C bypass)
    uint8_t st1;
    err = ak_read_bytes(AK_ST1, &st1, 1);
    if (err == ESP_OK && (st1 & 0x01)) {
        // Data ready
        uint8_t mag_buf[8];
        err = ak_read_bytes(AK_HXL, mag_buf, 8);  // Includes ST2 to clear
        if (err == ESP_OK) {
            // AK09916 is little-endian
            raw[6] = 0;  // placeholder
            int16_t mx = (int16_t)((mag_buf[1] << 8) | mag_buf[0]);
            int16_t my = (int16_t)((mag_buf[3] << 8) | mag_buf[2]);
            int16_t mz = (int16_t)((mag_buf[5] << 8) | mag_buf[4]);

            // Raw magnetometer values in uT (before calibration)
            float raw_mx = mx * MAG_SCALE_UT;
            float raw_my = my * MAG_SCALE_UT;
            float raw_mz = mz * MAG_SCALE_UT;

            // Collect samples for guided calibration if running
            if (g_magcal_running) {
                if (g_magcal_sample_count == 0) {
                    // First sample - initialize min/max
                    g_magcal_min_x = g_magcal_max_x = raw_mx;
                    g_magcal_min_y = g_magcal_max_y = raw_my;
                    g_magcal_min_z = g_magcal_max_z = raw_mz;
                } else {
                    // Update min/max
                    if (raw_mx < g_magcal_min_x) g_magcal_min_x = raw_mx;
                    if (raw_mx > g_magcal_max_x) g_magcal_max_x = raw_mx;
                    if (raw_my < g_magcal_min_y) g_magcal_min_y = raw_my;
                    if (raw_my > g_magcal_max_y) g_magcal_max_y = raw_my;
                    if (raw_mz < g_magcal_min_z) g_magcal_min_z = raw_mz;
                    if (raw_mz > g_magcal_max_z) g_magcal_max_z = raw_mz;
                }
                g_magcal_sample_count++;
            }

            // Apply hard-iron correction (subtract offsets)
            float hx = raw_mx - g_mag_offset_x;
            float hy = raw_my - g_mag_offset_y;
            float hz = raw_mz - g_mag_offset_z;

            // Apply soft-iron correction (matrix multiplication)
            float mag_x = g_mag_soft[0][0] * hx + g_mag_soft[0][1] * hy + g_mag_soft[0][2] * hz;
            float mag_y = g_mag_soft[1][0] * hx + g_mag_soft[1][1] * hy + g_mag_soft[1][2] * hz;
            float mag_z = g_mag_soft[2][0] * hx + g_mag_soft[2][1] * hy + g_mag_soft[2][2] * hz;

            // Store in temporary for later use
            if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_data.mag_x = mag_x;
                g_data.mag_y = mag_y;
                g_data.mag_z = mag_z;
                xSemaphoreGive(g_data_mutex);
            }
        }
    }

    i2c_release();

    // Convert to physical units and apply calibration
    float ax = raw[0] * ACCEL_SCALE_2G - g_accel_offset_x;
    float ay = raw[1] * ACCEL_SCALE_2G - g_accel_offset_y;
    float az = raw[2] * ACCEL_SCALE_2G - g_accel_offset_z;
    float gx = raw[3] * GYRO_SCALE_250DPS - g_gyro_bias_x;
    float gy = raw[4] * GYRO_SCALE_250DPS - g_gyro_bias_y;
    float gz = raw[5] * GYRO_SCALE_250DPS - g_gyro_bias_z;

    // Get current time
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // Compute delta time for Madgwick filter
    static uint32_t last_madgwick_update = 0;
    float dt = 0.01f;  // Default 10ms
    if (last_madgwick_update > 0 && now > last_madgwick_update) {
        dt = (float)(now - last_madgwick_update) / 1000.0f;
        if (dt > 0.1f) dt = 0.1f;  // Cap at 100ms to prevent large jumps
    }
    last_madgwick_update = now;

    // Update Madgwick sensor fusion filter (requires radians/sec for gyro)
    if (g_fusion_enabled) {
        float gx_rad = gx * M_PI / 180.0f;
        float gy_rad = gy * M_PI / 180.0f;
        float gz_rad = gz * M_PI / 180.0f;

        // Get current mag values (thread-safe)
        float mx_fused, my_fused, mz_fused;
        if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            mx_fused = g_data.mag_x;
            my_fused = g_data.mag_y;
            mz_fused = g_data.mag_z;
            xSemaphoreGive(g_data_mutex);
        } else {
            mx_fused = my_fused = mz_fused = 0.0f;
        }

        // Update Madgwick with 9-axis data
        madgwick_update_9dof(gx_rad, gy_rad, gz_rad,
                            ax, ay, az,
                            mx_fused, my_fused, mz_fused,
                            dt);
    }

    // Update data structure
    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_data.accel_x = ax;
        g_data.accel_y = ay;
        g_data.accel_z = az;
        g_data.gyro_x = gx;
        g_data.gyro_y = gy;
        g_data.gyro_z = gz;
        g_data.yaw_rate = gz;  // Z-axis is yaw

        // Compute roll and pitch from accelerometer (simple method)
        float accel_roll = atan2f(ay, az) * 180.0f / M_PI;
        float accel_pitch = atanf(-ax / sqrtf(ay*ay + az*az + 1e-9f)) * 180.0f / M_PI;

        // Compute tilt-compensated heading (raw, no fusion)
        g_data.heading_raw = compute_tilt_compensated_heading(
            ax, ay, az, g_data.mag_x, g_data.mag_y, g_data.mag_z);

        // Use Madgwick fusion for filtered values if enabled and converged
        if (g_fusion_enabled && madgwick_is_converged()) {
            g_data.heading_filtered = madgwick_get_yaw();
            g_data.roll = madgwick_get_roll();
            g_data.pitch = madgwick_get_pitch();
        } else {
            // Fall back to simple computation
            g_data.roll = accel_roll;
            g_data.pitch = accel_pitch;
            g_data.heading_filtered = filter_heading(g_data.heading_raw, now);
        }

        g_data.valid = true;
        g_data.last_update_ms = now;

        xSemaphoreGive(g_data_mutex);
    }

    return ESP_OK;
}

void icm20948_get_data(icm20948_data_t *data) {
    if (data == NULL) return;

    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        memcpy(data, &g_data, sizeof(icm20948_data_t));
        xSemaphoreGive(g_data_mutex);
    }
}

float icm20948_get_heading(void) {
    if (g_simulation) {
        return g_sim_heading;
    }

    float heading = 0.0f;
    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        heading = g_data.heading_filtered;
        xSemaphoreGive(g_data_mutex);
    }
    return heading;
}

float icm20948_get_heading_raw(void) {
    float heading = 0.0f;
    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        heading = g_data.heading_raw;
        xSemaphoreGive(g_data_mutex);
    }
    return heading;
}

float icm20948_get_yaw_rate(void) {
    float rate = 0.0f;
    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        rate = g_data.yaw_rate;
        xSemaphoreGive(g_data_mutex);
    }
    return rate;
}

void icm20948_set_mag_cal(float x, float y, float z) {
    g_mag_offset_x = x;
    g_mag_offset_y = y;
    g_mag_offset_z = z;
}

void icm20948_get_mag_cal(float *x, float *y, float *z) {
    if (x) *x = g_mag_offset_x;
    if (y) *y = g_mag_offset_y;
    if (z) *z = g_mag_offset_z;
}

void icm20948_set_simulation(bool enable) {
    g_simulation = enable;
    ESP_LOGI(TAG, "Simulation mode %s", enable ? "enabled" : "disabled");
}

bool icm20948_is_simulation(void) {
    return g_simulation;
}

void icm20948_set_simulated_heading(float heading) {
    g_sim_heading = clamp360(heading);
}

bool icm20948_is_valid(void) {
    bool valid = false;
    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        valid = g_data.valid;
        xSemaphoreGive(g_data_mutex);
    }
    return valid;
}

void icm20948_set_mag_soft_iron(const float matrix[9]) {
    if (matrix == NULL) return;

    // Copy to 3x3 matrix (row-major order)
    g_mag_soft[0][0] = matrix[0];
    g_mag_soft[0][1] = matrix[1];
    g_mag_soft[0][2] = matrix[2];
    g_mag_soft[1][0] = matrix[3];
    g_mag_soft[1][1] = matrix[4];
    g_mag_soft[1][2] = matrix[5];
    g_mag_soft[2][0] = matrix[6];
    g_mag_soft[2][1] = matrix[7];
    g_mag_soft[2][2] = matrix[8];

    // Minimal logging to reduce stack usage in BLE callback context
    ESP_LOGI(TAG, "Soft-iron matrix updated");
}

void icm20948_get_mag_soft_iron(float matrix[9]) {
    if (matrix == NULL) return;

    // Copy from 3x3 matrix (row-major order)
    matrix[0] = g_mag_soft[0][0];
    matrix[1] = g_mag_soft[0][1];
    matrix[2] = g_mag_soft[0][2];
    matrix[3] = g_mag_soft[1][0];
    matrix[4] = g_mag_soft[1][1];
    matrix[5] = g_mag_soft[1][2];
    matrix[6] = g_mag_soft[2][0];
    matrix[7] = g_mag_soft[2][1];
    matrix[8] = g_mag_soft[2][2];
}

void icm20948_load_mag_cal_from_nvs(void) {
    // Load hard-iron offsets
    g_mag_offset_x = param_get(PARAM_MAG_HARD_X);
    g_mag_offset_y = param_get(PARAM_MAG_HARD_Y);
    g_mag_offset_z = param_get(PARAM_MAG_HARD_Z);

    // Load soft-iron matrix
    g_mag_soft[0][0] = param_get(PARAM_MAG_SOFT_00);
    g_mag_soft[0][1] = param_get(PARAM_MAG_SOFT_01);
    g_mag_soft[0][2] = param_get(PARAM_MAG_SOFT_02);
    g_mag_soft[1][0] = param_get(PARAM_MAG_SOFT_10);
    g_mag_soft[1][1] = param_get(PARAM_MAG_SOFT_11);
    g_mag_soft[1][2] = param_get(PARAM_MAG_SOFT_12);
    g_mag_soft[2][0] = param_get(PARAM_MAG_SOFT_20);
    g_mag_soft[2][1] = param_get(PARAM_MAG_SOFT_21);
    g_mag_soft[2][2] = param_get(PARAM_MAG_SOFT_22);

    ESP_LOGI(TAG, "Loaded mag cal from NVS:");
    ESP_LOGI(TAG, "  Hard-iron: [%.2f, %.2f, %.2f] uT",
             g_mag_offset_x, g_mag_offset_y, g_mag_offset_z);
    ESP_LOGI(TAG, "  Soft-iron: [%.4f, %.4f, %.4f]", g_mag_soft[0][0], g_mag_soft[0][1], g_mag_soft[0][2]);
    ESP_LOGI(TAG, "             [%.4f, %.4f, %.4f]", g_mag_soft[1][0], g_mag_soft[1][1], g_mag_soft[1][2]);
    ESP_LOGI(TAG, "             [%.4f, %.4f, %.4f]", g_mag_soft[2][0], g_mag_soft[2][1], g_mag_soft[2][2]);
}

esp_err_t icm20948_save_mag_cal_to_nvs(void) {
    esp_err_t err;

    // Save hard-iron offsets
    err = param_set(PARAM_MAG_HARD_X, g_mag_offset_x);
    if (err != ESP_OK) return err;
    err = param_set(PARAM_MAG_HARD_Y, g_mag_offset_y);
    if (err != ESP_OK) return err;
    err = param_set(PARAM_MAG_HARD_Z, g_mag_offset_z);
    if (err != ESP_OK) return err;

    // Save soft-iron matrix
    err = param_set(PARAM_MAG_SOFT_00, g_mag_soft[0][0]);
    if (err != ESP_OK) return err;
    err = param_set(PARAM_MAG_SOFT_01, g_mag_soft[0][1]);
    if (err != ESP_OK) return err;
    err = param_set(PARAM_MAG_SOFT_02, g_mag_soft[0][2]);
    if (err != ESP_OK) return err;
    err = param_set(PARAM_MAG_SOFT_10, g_mag_soft[1][0]);
    if (err != ESP_OK) return err;
    err = param_set(PARAM_MAG_SOFT_11, g_mag_soft[1][1]);
    if (err != ESP_OK) return err;
    err = param_set(PARAM_MAG_SOFT_12, g_mag_soft[1][2]);
    if (err != ESP_OK) return err;
    err = param_set(PARAM_MAG_SOFT_20, g_mag_soft[2][0]);
    if (err != ESP_OK) return err;
    err = param_set(PARAM_MAG_SOFT_21, g_mag_soft[2][1]);
    if (err != ESP_OK) return err;
    err = param_set(PARAM_MAG_SOFT_22, g_mag_soft[2][2]);
    if (err != ESP_OK) return err;

    // Persist to NVS
    err = param_save_all();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save mag cal to NVS: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Mag calibration saved to NVS");
    return ESP_OK;
}

void icm20948_reset_mag_cal(void) {
    // Reset hard-iron to zeros
    g_mag_offset_x = 0.0f;
    g_mag_offset_y = 0.0f;
    g_mag_offset_z = 0.0f;

    // Reset soft-iron to identity matrix
    g_mag_soft[0][0] = 1.0f; g_mag_soft[0][1] = 0.0f; g_mag_soft[0][2] = 0.0f;
    g_mag_soft[1][0] = 0.0f; g_mag_soft[1][1] = 1.0f; g_mag_soft[1][2] = 0.0f;
    g_mag_soft[2][0] = 0.0f; g_mag_soft[2][1] = 0.0f; g_mag_soft[2][2] = 1.0f;

    ESP_LOGI(TAG, "Mag calibration reset to defaults");
}

/*============================================================================
 * Guided Magnetometer Calibration
 *============================================================================*/

esp_err_t icm20948_magcal_start(void) {
    if (g_magcal_running) {
        ESP_LOGW(TAG, "Mag calibration already running");
        return ESP_ERR_INVALID_STATE;
    }

    // Reset state
    g_magcal_sample_count = 0;
    g_magcal_min_x = g_magcal_max_x = 0.0f;
    g_magcal_min_y = g_magcal_max_y = 0.0f;
    g_magcal_min_z = g_magcal_max_z = 0.0f;
    g_magcal_quality = -1.0f;
    g_magcal_running = true;

    ESP_LOGI(TAG, "Mag calibration started - rotate device in figure-8 pattern");
    return ESP_OK;
}

esp_err_t icm20948_magcal_stop(void) {
    if (!g_magcal_running) {
        ESP_LOGW(TAG, "Mag calibration not running");
        return ESP_ERR_INVALID_STATE;
    }

    g_magcal_running = false;

    // Require minimum samples (at 100Hz, ~500 samples = 5 seconds)
    if (g_magcal_sample_count < 200) {
        ESP_LOGW(TAG, "Not enough samples (%lu), need at least 200", g_magcal_sample_count);
        return ESP_ERR_INVALID_STATE;
    }

    // Compute hard-iron offsets (center of ellipsoid)
    float offset_x = (g_magcal_max_x + g_magcal_min_x) / 2.0f;
    float offset_y = (g_magcal_max_y + g_magcal_min_y) / 2.0f;
    float offset_z = (g_magcal_max_z + g_magcal_min_z) / 2.0f;

    // Compute axis ranges
    float range_x = g_magcal_max_x - g_magcal_min_x;
    float range_y = g_magcal_max_y - g_magcal_min_y;
    float range_z = g_magcal_max_z - g_magcal_min_z;

    // Check for valid ranges
    if (range_x < 5.0f || range_y < 5.0f || range_z < 5.0f) {
        ESP_LOGW(TAG, "Range too small: X=%.1f Y=%.1f Z=%.1f uT", range_x, range_y, range_z);
        ESP_LOGW(TAG, "Rotate device more during calibration");
        return ESP_ERR_INVALID_STATE;
    }

    // Compute average radius (ideally equal for a sphere)
    float avg_radius = (range_x + range_y + range_z) / 6.0f;  // /2 for radius, /3 for average

    // Compute soft-iron scale factors (diagonal only for simplicity)
    // Scale each axis to make the ellipsoid into a sphere
    float scale_x = avg_radius / (range_x / 2.0f);
    float scale_y = avg_radius / (range_y / 2.0f);
    float scale_z = avg_radius / (range_z / 2.0f);

    // Compute quality metric (how spherical the data is)
    // Perfect sphere = 1.0, elongated ellipsoid < 1.0
    float min_range = fminf(fminf(range_x, range_y), range_z);
    float max_range = fmaxf(fmaxf(range_x, range_y), range_z);
    g_magcal_quality = min_range / max_range;

    // Apply computed calibration
    g_mag_offset_x = offset_x;
    g_mag_offset_y = offset_y;
    g_mag_offset_z = offset_z;

    // Set diagonal soft-iron matrix
    g_mag_soft[0][0] = scale_x; g_mag_soft[0][1] = 0.0f;    g_mag_soft[0][2] = 0.0f;
    g_mag_soft[1][0] = 0.0f;    g_mag_soft[1][1] = scale_y; g_mag_soft[1][2] = 0.0f;
    g_mag_soft[2][0] = 0.0f;    g_mag_soft[2][1] = 0.0f;    g_mag_soft[2][2] = scale_z;

    ESP_LOGI(TAG, "Mag calibration complete (%lu samples)", g_magcal_sample_count);
    ESP_LOGI(TAG, "  Hard-iron: [%.2f, %.2f, %.2f] uT", offset_x, offset_y, offset_z);
    ESP_LOGI(TAG, "  Scale: [%.3f, %.3f, %.3f]", scale_x, scale_y, scale_z);
    ESP_LOGI(TAG, "  Quality: %.1f%%", g_magcal_quality * 100.0f);

    return ESP_OK;
}

bool icm20948_magcal_is_running(void) {
    return g_magcal_running;
}

float icm20948_magcal_get_quality(void) {
    return g_magcal_quality;
}

uint32_t icm20948_magcal_get_sample_count(void) {
    return g_magcal_sample_count;
}

size_t icm20948_magcal_get_status(char *buf, size_t size) {
    if (g_magcal_running) {
        float range_x = g_magcal_max_x - g_magcal_min_x;
        float range_y = g_magcal_max_y - g_magcal_min_y;
        float range_z = g_magcal_max_z - g_magcal_min_z;
        return snprintf(buf, size,
            "Mag Calibration IN PROGRESS\r\n"
            "Samples: %lu\r\n"
            "Range X: %.1f uT (%.1f to %.1f)\r\n"
            "Range Y: %.1f uT (%.1f to %.1f)\r\n"
            "Range Z: %.1f uT (%.1f to %.1f)\r\n"
            "Rotate device in figure-8 pattern...\r\n",
            g_magcal_sample_count,
            range_x, g_magcal_min_x, g_magcal_max_x,
            range_y, g_magcal_min_y, g_magcal_max_y,
            range_z, g_magcal_min_z, g_magcal_max_z);
    } else if (g_magcal_quality >= 0) {
        return snprintf(buf, size,
            "Mag Calibration COMPLETE\r\n"
            "Samples: %lu\r\n"
            "Hard-iron: [%.2f, %.2f, %.2f] uT\r\n"
            "Soft-iron: [%.3f, %.3f, %.3f]\r\n"
            "Quality: %.1f%%\r\n"
            "Use 'magcal save' to persist to NVS\r\n",
            g_magcal_sample_count,
            g_mag_offset_x, g_mag_offset_y, g_mag_offset_z,
            g_mag_soft[0][0], g_mag_soft[1][1], g_mag_soft[2][2],
            g_magcal_quality * 100.0f);
    } else {
        return snprintf(buf, size,
            "Mag Calibration NOT STARTED\r\n"
            "Current hard-iron: [%.2f, %.2f, %.2f] uT\r\n"
            "Use 'magcal start' to begin calibration\r\n",
            g_mag_offset_x, g_mag_offset_y, g_mag_offset_z);
    }
}

/*============================================================================
 * Accelerometer Calibration
 *============================================================================*/

esp_err_t icm20948_accel_cal_level(void) {
    ESP_LOGI(TAG, "Starting accelerometer level calibration...");
    ESP_LOGI(TAG, "Keep device stationary and level!");

    // Collect samples for 2 seconds
    float sum_ax = 0, sum_ay = 0, sum_az = 0;
    int samples = 0;
    const int target_samples = 100;

    for (int i = 0; i < target_samples; i++) {
        esp_err_t err = icm20948_update();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Update failed during accel cal");
            continue;
        }

        icm20948_data_t data;
        icm20948_get_data(&data);

        // Note: need raw uncalibrated values, so add back current offset
        sum_ax += data.accel_x + g_accel_offset_x;
        sum_ay += data.accel_y + g_accel_offset_y;
        sum_az += data.accel_z + g_accel_offset_z;
        samples++;

        vTaskDelay(pdMS_TO_TICKS(20));  // ~50 Hz
    }

    if (samples < target_samples / 2) {
        ESP_LOGE(TAG, "Not enough samples for accel calibration");
        return ESP_ERR_INVALID_STATE;
    }

    // Compute average (should be [0, 0, 1g] when level)
    float avg_ax = sum_ax / samples;
    float avg_ay = sum_ay / samples;
    float avg_az = sum_az / samples;

    // Offset = measured - expected
    // Expected: X=0, Y=0, Z=1.0g (assuming Z is up)
    g_accel_offset_x = avg_ax - 0.0f;
    g_accel_offset_y = avg_ay - 0.0f;
    g_accel_offset_z = avg_az - 1.0f;

    ESP_LOGI(TAG, "Accel calibration complete (%d samples)", samples);
    ESP_LOGI(TAG, "  Measured: [%.4f, %.4f, %.4f] g", avg_ax, avg_ay, avg_az);
    ESP_LOGI(TAG, "  Offset: [%.4f, %.4f, %.4f] g",
             g_accel_offset_x, g_accel_offset_y, g_accel_offset_z);

    return ESP_OK;
}

void icm20948_get_accel_cal(float *x, float *y, float *z) {
    if (x) *x = g_accel_offset_x;
    if (y) *y = g_accel_offset_y;
    if (z) *z = g_accel_offset_z;
}

void icm20948_set_accel_cal(float x, float y, float z) {
    g_accel_offset_x = x;
    g_accel_offset_y = y;
    g_accel_offset_z = z;
    ESP_LOGI(TAG, "Accel offset set: [%.4f, %.4f, %.4f] g", x, y, z);
}

/*============================================================================
 * Gyroscope Calibration
 *============================================================================*/

esp_err_t icm20948_calibrate_gyro(void) {
    ESP_LOGI(TAG, "Starting gyroscope bias calibration...");
    ESP_LOGI(TAG, "Keep device STATIONARY!");

    // Temporarily disable gyro bias so we read raw values
    float old_bias_x = g_gyro_bias_x;
    float old_bias_y = g_gyro_bias_y;
    float old_bias_z = g_gyro_bias_z;
    g_gyro_bias_x = g_gyro_bias_y = g_gyro_bias_z = 0.0f;

    // Collect samples for 1 second
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    int samples = 0;
    const int target_samples = 100;

    for (int i = 0; i < target_samples; i++) {
        esp_err_t err = icm20948_update();
        if (err != ESP_OK) {
            continue;
        }

        icm20948_data_t data;
        icm20948_get_data(&data);

        sum_gx += data.gyro_x;
        sum_gy += data.gyro_y;
        sum_gz += data.gyro_z;
        samples++;

        vTaskDelay(pdMS_TO_TICKS(10));  // ~100 Hz
    }

    if (samples < target_samples / 2) {
        // Restore old bias on failure
        g_gyro_bias_x = old_bias_x;
        g_gyro_bias_y = old_bias_y;
        g_gyro_bias_z = old_bias_z;
        ESP_LOGE(TAG, "Not enough samples for gyro calibration");
        return ESP_ERR_INVALID_STATE;
    }

    // Compute bias (average when stationary should be zero)
    g_gyro_bias_x = sum_gx / samples;
    g_gyro_bias_y = sum_gy / samples;
    g_gyro_bias_z = sum_gz / samples;

    ESP_LOGI(TAG, "Gyro calibration complete (%d samples)", samples);
    ESP_LOGI(TAG, "  Bias: [%.3f, %.3f, %.3f] deg/s",
             g_gyro_bias_x, g_gyro_bias_y, g_gyro_bias_z);

    return ESP_OK;
}

void icm20948_get_gyro_bias(float *x, float *y, float *z) {
    if (x) *x = g_gyro_bias_x;
    if (y) *y = g_gyro_bias_y;
    if (z) *z = g_gyro_bias_z;
}

/*============================================================================
 * Sensor Fusion (Madgwick Filter) API
 *============================================================================*/

void icm20948_set_fusion_enabled(bool enable) {
    if (enable && !g_fusion_enabled) {
        // Reset filter when enabling
        madgwick_reset();
    }
    g_fusion_enabled = enable;
    ESP_LOGI(TAG, "Sensor fusion %s", enable ? "ENABLED" : "DISABLED");
}

bool icm20948_is_fusion_enabled(void) {
    return g_fusion_enabled;
}

void icm20948_set_fusion_beta(float beta) {
    if (beta >= 0.01f && beta <= 1.0f) {
        g_fusion_beta = beta;
        madgwick_set_beta(beta);
        ESP_LOGI(TAG, "Fusion beta set to %.3f", beta);
    } else {
        ESP_LOGW(TAG, "Invalid beta %.3f, must be 0.01-1.0", beta);
    }
}

float icm20948_get_fusion_beta(void) {
    return g_fusion_beta;
}

bool icm20948_is_fusion_converged(void) {
    return g_fusion_enabled && madgwick_is_converged();
}

void icm20948_reset_fusion(void) {
    madgwick_reset();
    ESP_LOGI(TAG, "Sensor fusion reset");
}

/*============================================================================
 * GPS-Aided Calibration Monitoring
 *============================================================================*/

// Helper to normalize angle difference to -180..+180
static float normalize_angle_diff(float diff) {
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

void icm20948_update_gps_reference(float gps_heading, float speed_knots) {
    // Only use GPS heading when moving fast enough for reliable COG
    if (speed_knots < GPS_OFFSET_MIN_SPEED_KNOTS) {
        return;
    }

    // Get current magnetometer heading
    float mag_heading = 0.0f;
    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        mag_heading = g_data.heading_filtered;
        xSemaphoreGive(g_data_mutex);
    }
    g_last_mag_heading = mag_heading;

    // Compute heading offset (mag - gps)
    // Positive = magnetometer reads higher than GPS
    float offset = normalize_angle_diff(mag_heading - gps_heading);

    // Accumulate using circular statistics (handles wrap-around)
    float offset_rad = offset * M_PI / 180.0f;
    g_gps_offset_sum_sin += sinf(offset_rad);
    g_gps_offset_sum_cos += cosf(offset_rad);
    g_gps_offset_count++;

    // Log occasionally for debugging
    if ((g_gps_offset_count % 100) == 1) {
        float avg_offset = icm20948_get_heading_offset();
        ESP_LOGI(TAG, "GPS cal: mag=%.1f gps=%.1f offset=%.1f avg=%.1f (n=%lu)",
                 mag_heading, gps_heading, offset, avg_offset, g_gps_offset_count);
    }
}

float icm20948_get_heading_offset(void) {
    if (g_gps_offset_count < 10) {
        return NAN;  // Not enough samples
    }

    // Compute circular mean
    float mean_sin = g_gps_offset_sum_sin / g_gps_offset_count;
    float mean_cos = g_gps_offset_sum_cos / g_gps_offset_count;
    float avg_rad = atan2f(mean_sin, mean_cos);
    return avg_rad * 180.0f / M_PI;
}

uint32_t icm20948_get_gps_sample_count(void) {
    return g_gps_offset_count;
}

void icm20948_reset_gps_offset(void) {
    g_gps_offset_sum_sin = 0.0f;
    g_gps_offset_sum_cos = 0.0f;
    g_gps_offset_count = 0;
    ESP_LOGI(TAG, "GPS heading offset estimation reset");
}

esp_err_t icm20948_apply_gps_offset(void) {
    // Require at least 100 samples (about 1 minute at 100Hz)
    if (g_gps_offset_count < 100) {
        ESP_LOGW(TAG, "Not enough GPS samples (%lu), need at least 100", g_gps_offset_count);
        return ESP_ERR_INVALID_STATE;
    }

    float offset = icm20948_get_heading_offset();
    if (isnan(offset)) {
        return ESP_ERR_INVALID_STATE;
    }

    // Only apply if offset is significant (> 2 degrees) but not extreme (< 30 degrees)
    if (fabsf(offset) < 2.0f) {
        ESP_LOGI(TAG, "Heading offset %.1f deg is small, no adjustment needed", offset);
        return ESP_OK;
    }

    if (fabsf(offset) > 30.0f) {
        ESP_LOGW(TAG, "Heading offset %.1f deg is too large, skipping auto-cal", offset);
        ESP_LOGW(TAG, "Manual calibration may be needed");
        return ESP_ERR_INVALID_STATE;
    }

    // Log the correction
    ESP_LOGI(TAG, "Applying GPS-derived heading offset: %.1f deg", offset);
    ESP_LOGI(TAG, "Old hard-iron: [%.2f, %.2f, %.2f] uT",
             g_mag_offset_x, g_mag_offset_y, g_mag_offset_z);

    // Adjust hard-iron offset to reduce heading error
    // This is a simplified correction - assumes flat operation
    // Positive offset means mag reads too high -> rotate calibration
    float offset_rad = offset * M_PI / 180.0f;
    float cos_off = cosf(offset_rad);
    float sin_off = sinf(offset_rad);

    // Rotate the hard-iron offsets in the XY plane
    float new_offset_x = g_mag_offset_x * cos_off - g_mag_offset_y * sin_off;
    float new_offset_y = g_mag_offset_x * sin_off + g_mag_offset_y * cos_off;

    g_mag_offset_x = new_offset_x;
    g_mag_offset_y = new_offset_y;

    ESP_LOGI(TAG, "New hard-iron: [%.2f, %.2f, %.2f] uT",
             g_mag_offset_x, g_mag_offset_y, g_mag_offset_z);

    // Reset the GPS offset tracking
    icm20948_reset_gps_offset();

    return ESP_OK;
}
