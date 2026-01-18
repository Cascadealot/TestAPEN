/**
 * @file gnss_driver.c
 * @brief u-blox GNSS Driver Implementation for TestAP2
 *
 * FSD Reference: TestAP2.FSD.v1.0.0.md Section 10
 */

#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "gnss_driver.h"
#include "autopilot_common.h"

static const char *TAG = "GNSS";

/*============================================================================
 * UBX Protocol Constants
 *============================================================================*/

#define UBX_SYNC_1          0xB5
#define UBX_SYNC_2          0x62

// UBX Class IDs
#define UBX_CLASS_NAV       0x01
#define UBX_CLASS_CFG       0x06

// UBX Message IDs
#define UBX_NAV_PVT         0x07
#define UBX_CFG_PRT         0x00
#define UBX_CFG_MSG         0x01
#define UBX_CFG_RATE        0x08
#define UBX_CFG_VALSET      0x8A
#define UBX_CFG_VALGET      0x8B

// M10 Configuration Keys (UBX-CFG-VALSET)
#define CFG_MSGOUT_UBX_NAV_PVT_UART1    0x20910007
#define CFG_UART1OUTPROT_UBX            0x10740001
#define CFG_UART1OUTPROT_NMEA           0x10740002
#define CFG_RATE_MEAS                   0x30210001

// NAV-PVT message length
#define UBX_NAV_PVT_LEN     92

/*============================================================================
 * UBX-NAV-PVT Structure (per u-blox protocol spec)
 *============================================================================*/

typedef struct __attribute__((packed)) {
    uint32_t iTOW;          // 0: GPS time of week (ms)
    uint16_t year;          // 4: Year (UTC)
    uint8_t month;          // 6: Month (1-12)
    uint8_t day;            // 7: Day (1-31)
    uint8_t hour;           // 8: Hour (0-23)
    uint8_t min;            // 9: Minute (0-59)
    uint8_t sec;            // 10: Second (0-60, 60 for leap second)
    uint8_t valid;          // 11: Validity flags
    uint32_t tAcc;          // 12: Time accuracy (ns)
    int32_t nano;           // 16: Nanoseconds (UTC)
    uint8_t fixType;        // 20: Fix type
    uint8_t flags;          // 21: Fix status flags
    uint8_t flags2;         // 22: Additional flags
    uint8_t numSV;          // 23: Number of satellites
    int32_t lon;            // 24: Longitude (1e-7 degrees)
    int32_t lat;            // 28: Latitude (1e-7 degrees)
    int32_t height;         // 32: Height above ellipsoid (mm)
    int32_t hMSL;           // 36: Height above MSL (mm)
    uint32_t hAcc;          // 40: Horizontal accuracy (mm)
    uint32_t vAcc;          // 44: Vertical accuracy (mm)
    int32_t velN;           // 48: North velocity (mm/s)
    int32_t velE;           // 52: East velocity (mm/s)
    int32_t velD;           // 56: Down velocity (mm/s)
    int32_t gSpeed;         // 60: Ground speed (mm/s)
    int32_t headMot;        // 64: Heading of motion (1e-5 degrees)
    uint32_t sAcc;          // 68: Speed accuracy (mm/s)
    uint32_t headAcc;       // 72: Heading accuracy (1e-5 degrees)
    uint16_t pDOP;          // 76: Position DOP (0.01)
    uint8_t flags3;         // 78: Additional flags
    uint8_t reserved1[5];   // 79-83: Reserved
    int32_t headVeh;        // 84: Heading of vehicle (1e-5 degrees)
    int16_t magDec;         // 88: Magnetic declination (1e-2 degrees)
    uint16_t magAcc;        // 90: Magnetic declination accuracy
} ubx_nav_pvt_t;

/*============================================================================
 * Parser State Machine
 *============================================================================*/

typedef enum {
    PARSE_SYNC1,
    PARSE_SYNC2,
    PARSE_CLASS,
    PARSE_ID,
    PARSE_LEN_L,
    PARSE_LEN_H,
    PARSE_PAYLOAD,
    PARSE_CK_A,
    PARSE_CK_B
} parse_state_t;

/*============================================================================
 * Static Variables
 *============================================================================*/

static int s_uart_num = -1;
static gnss_data_t s_data = {0};
static bool s_initialized = false;

// Parser state
static parse_state_t s_parse_state = PARSE_SYNC1;
static uint8_t s_msg_class = 0;
static uint8_t s_msg_id = 0;
static uint16_t s_msg_len = 0;
static uint16_t s_payload_idx = 0;
static uint8_t s_payload[256];
static uint8_t s_ck_a = 0;
static uint8_t s_ck_b = 0;

// Statistics
static uint32_t s_bytes_received = 0;
static uint32_t s_message_count = 0;
static uint32_t s_error_count = 0;

/*============================================================================
 * COG Validity Constants (FSD Section 10.4)
 *============================================================================*/

#define COG_MIN_SPEED_KTS       1.5f    // Minimum speed for COG validity
#define COG_MAX_ACCURACY_DEG    10.0f   // Maximum COG accuracy for validity
#define COG_BLEND_LOW_KTS       1.5f    // Start blending at this speed
#define COG_BLEND_HIGH_KTS      3.0f    // Full COG at this speed
#define DATA_TIMEOUT_MS         2000    // Consider data stale after 2 seconds

/*============================================================================
 * Helper Functions
 *============================================================================*/

static void reset_parser(void) {
    s_parse_state = PARSE_SYNC1;
    s_ck_a = 0;
    s_ck_b = 0;
    s_payload_idx = 0;
}

static void update_checksum(uint8_t byte) {
    s_ck_a += byte;
    s_ck_b += s_ck_a;
}

/*============================================================================
 * Process NAV-PVT Message
 *============================================================================*/

static void process_nav_pvt(const uint8_t *payload, uint16_t len) {
    if (len != UBX_NAV_PVT_LEN) {
        ESP_LOGW(TAG, "NAV-PVT wrong length: %u (expected %d)", len, UBX_NAV_PVT_LEN);
        s_error_count++;
        return;
    }

    const ubx_nav_pvt_t *pvt = (const ubx_nav_pvt_t *)payload;

    // Update timestamp
    s_data.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // Fix information
    s_data.fix_type = (gnss_fix_type_t)pvt->fixType;
    s_data.num_satellites = pvt->numSV;
    s_data.hdop = pvt->pDOP * 0.01f;

    // Time
    s_data.year = pvt->year;
    s_data.month = pvt->month;
    s_data.day = pvt->day;
    s_data.utc_time = pvt->hour * 10000 + pvt->min * 100 + pvt->sec;

    // Position (convert from 1e-7 degrees to degrees)
    s_data.latitude = pvt->lat * 1e-7f;
    s_data.longitude = pvt->lon * 1e-7f;
    s_data.altitude = pvt->hMSL * 0.001f;  // mm to m

    // Speed (convert from mm/s to knots: 1 m/s = 1.94384 kts)
    s_data.speed_kts = pvt->gSpeed * 0.001f * 1.94384f;

    // COG (convert from 1e-5 degrees to degrees)
    s_data.cog = pvt->headMot * 1e-5f;
    if (s_data.cog < 0.0f) s_data.cog += 360.0f;
    if (s_data.cog >= 360.0f) s_data.cog -= 360.0f;

    // COG accuracy (convert from 1e-5 degrees to degrees)
    s_data.cog_accuracy = pvt->headAcc * 1e-5f;

    // Validity checks
    s_data.valid = (pvt->fixType >= GNSS_FIX_2D);
    s_data.cog_valid = s_data.valid &&
                       (s_data.speed_kts >= COG_MIN_SPEED_KTS) &&
                       (s_data.cog_accuracy < COG_MAX_ACCURACY_DEG);

    s_message_count++;

    ESP_LOGD(TAG, "NAV-PVT: fix=%d sats=%d lat=%.6f lon=%.6f spd=%.1f kts cog=%.1f",
             s_data.fix_type, s_data.num_satellites, s_data.latitude,
             s_data.longitude, s_data.speed_kts, s_data.cog);
}

/*============================================================================
 * Process Received UBX Message
 *============================================================================*/

static void process_ubx_message(void) {
    if (s_msg_class == UBX_CLASS_NAV && s_msg_id == UBX_NAV_PVT) {
        process_nav_pvt(s_payload, s_msg_len);
    }
    // Other messages can be added here
}

/*============================================================================
 * Parse Incoming Byte
 *============================================================================*/

static void parse_byte(uint8_t byte) {
    switch (s_parse_state) {
        case PARSE_SYNC1:
            if (byte == UBX_SYNC_1) {
                s_parse_state = PARSE_SYNC2;
            }
            break;

        case PARSE_SYNC2:
            if (byte == UBX_SYNC_2) {
                s_parse_state = PARSE_CLASS;
                s_ck_a = 0;
                s_ck_b = 0;
            } else {
                reset_parser();
            }
            break;

        case PARSE_CLASS:
            s_msg_class = byte;
            update_checksum(byte);
            s_parse_state = PARSE_ID;
            break;

        case PARSE_ID:
            s_msg_id = byte;
            update_checksum(byte);
            s_parse_state = PARSE_LEN_L;
            break;

        case PARSE_LEN_L:
            s_msg_len = byte;
            update_checksum(byte);
            s_parse_state = PARSE_LEN_H;
            break;

        case PARSE_LEN_H:
            s_msg_len |= (uint16_t)byte << 8;
            update_checksum(byte);
            if (s_msg_len == 0) {
                s_parse_state = PARSE_CK_A;
            } else if (s_msg_len > sizeof(s_payload)) {
                ESP_LOGW(TAG, "Message too long: %u", s_msg_len);
                s_error_count++;
                reset_parser();
            } else {
                s_payload_idx = 0;
                s_parse_state = PARSE_PAYLOAD;
            }
            break;

        case PARSE_PAYLOAD:
            s_payload[s_payload_idx++] = byte;
            update_checksum(byte);
            if (s_payload_idx >= s_msg_len) {
                s_parse_state = PARSE_CK_A;
            }
            break;

        case PARSE_CK_A:
            if (byte == s_ck_a) {
                s_parse_state = PARSE_CK_B;
            } else {
                ESP_LOGW(TAG, "Checksum A mismatch: got 0x%02X, expected 0x%02X", byte, s_ck_a);
                s_error_count++;
                reset_parser();
            }
            break;

        case PARSE_CK_B:
            if (byte == s_ck_b) {
                // Valid message!
                process_ubx_message();
            } else {
                ESP_LOGW(TAG, "Checksum B mismatch: got 0x%02X, expected 0x%02X", byte, s_ck_b);
                s_error_count++;
            }
            reset_parser();
            break;
    }
}

/*============================================================================
 * UBX Configuration Commands
 *============================================================================*/

static uint8_t ubx_checksum_a;
static uint8_t ubx_checksum_b;

static void ubx_ck_reset(void) {
    ubx_checksum_a = 0;
    ubx_checksum_b = 0;
}

static void ubx_ck_add(uint8_t byte) {
    ubx_checksum_a += byte;
    ubx_checksum_b += ubx_checksum_a;
}

static esp_err_t send_ubx_command(uint8_t msg_class, uint8_t msg_id,
                                   const uint8_t *payload, uint16_t len) {
    if (s_uart_num < 0) return ESP_ERR_INVALID_STATE;

    uint8_t header[6] = {
        UBX_SYNC_1, UBX_SYNC_2,
        msg_class, msg_id,
        (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)
    };

    // Calculate checksum
    ubx_ck_reset();
    ubx_ck_add(msg_class);
    ubx_ck_add(msg_id);
    ubx_ck_add(len & 0xFF);
    ubx_ck_add(len >> 8);
    for (uint16_t i = 0; i < len; i++) {
        ubx_ck_add(payload[i]);
    }

    // Send header
    uart_write_bytes(s_uart_num, header, 6);

    // Send payload
    if (len > 0 && payload != NULL) {
        uart_write_bytes(s_uart_num, payload, len);
    }

    // Send checksum
    uint8_t checksum[2] = {ubx_checksum_a, ubx_checksum_b};
    uart_write_bytes(s_uart_num, checksum, 2);

    return ESP_OK;
}

static esp_err_t configure_module(void) {
    esp_err_t err;

    // For M10 series, use UBX-CFG-VALSET to configure
    // Payload: version(1) + layers(1) + reserved(2) + key-value pairs

    // Step 1: Enable UBX protocol output on UART1
    ESP_LOGI(TAG, "Enabling UBX protocol output...");
    uint8_t cfg_ubx_out[] = {
        0x00,                   // version
        0x01,                   // layers: RAM only
        0x00, 0x00,             // reserved
        // Key: CFG-UART1OUTPROT-UBX (0x10740001)
        0x01, 0x00, 0x74, 0x10,
        0x01                    // value: enable
    };
    err = send_ubx_command(UBX_CLASS_CFG, UBX_CFG_VALSET, cfg_ubx_out, sizeof(cfg_ubx_out));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to enable UBX output");
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    // Step 2: Enable NAV-PVT message on UART1
    ESP_LOGI(TAG, "Enabling NAV-PVT messages...");
    uint8_t cfg_nav_pvt[] = {
        0x00,                   // version
        0x01,                   // layers: RAM only
        0x00, 0x00,             // reserved
        // Key: CFG-MSGOUT-UBX_NAV_PVT_UART1 (0x20910007)
        0x07, 0x00, 0x91, 0x20,
        0x01                    // value: output every measurement
    };
    err = send_ubx_command(UBX_CLASS_CFG, UBX_CFG_VALSET, cfg_nav_pvt, sizeof(cfg_nav_pvt));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to enable NAV-PVT");
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    // Also try legacy CFG-MSG for older modules
    ESP_LOGI(TAG, "Sending legacy CFG-MSG (for older modules)...");
    uint8_t cfg_msg[] = {UBX_CLASS_NAV, UBX_NAV_PVT, 0, 1, 0, 0, 0, 0};
    send_ubx_command(UBX_CLASS_CFG, UBX_CFG_MSG, cfg_msg, sizeof(cfg_msg));
    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_LOGI(TAG, "Module configuration sent");
    return ESP_OK;
}

/*============================================================================
 * Public API Implementation
 *============================================================================*/

esp_err_t gnss_init(int uart_num, int tx_gpio, int rx_gpio) {
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing GNSS on UART%d (TX=%d, RX=%d)", uart_num, tx_gpio, rx_gpio);

    // Start at 9600 baud (u-blox default), then configure module
    uart_config_t uart_config = {
        .baud_rate = 9600,  // u-blox M10Q default
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_param_config(uart_num, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_set_pin(uart_num, tx_gpio, rx_gpio, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(err));
        return err;
    }

    // Install UART driver with 1KB RX buffer
    err = uart_driver_install(uart_num, 1024, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(err));
        return err;
    }

    s_uart_num = uart_num;
    s_initialized = true;

    // Reset statistics
    s_bytes_received = 0;
    s_message_count = 0;
    s_error_count = 0;

    // Clear data
    memset(&s_data, 0, sizeof(s_data));

    // Give module time to start up
    vTaskDelay(pdMS_TO_TICKS(100));

    // Configure module for UBX NAV-PVT output
    configure_module();

    ESP_LOGI(TAG, "GNSS driver initialized at 9600 baud");
    return ESP_OK;
}

void gnss_deinit(void) {
    if (!s_initialized) return;

    if (s_uart_num >= 0) {
        uart_driver_delete(s_uart_num);
    }
    s_uart_num = -1;
    s_initialized = false;
    ESP_LOGI(TAG, "GNSS driver deinitialized");
}

esp_err_t gnss_update(void) {
    if (!s_initialized || s_uart_num < 0) {
        return ESP_ERR_INVALID_STATE;
    }

    // Read available data from UART
    uint8_t buf[128];
    int len = uart_read_bytes(s_uart_num, buf, sizeof(buf), 0);

    if (len <= 0) {
        return ESP_ERR_NOT_FOUND;
    }

    s_bytes_received += len;

    // Parse each byte
    for (int i = 0; i < len; i++) {
        parse_byte(buf[i]);
    }

    return ESP_OK;
}

esp_err_t gnss_get_data(gnss_data_t *data) {
    if (data == NULL) return ESP_ERR_INVALID_ARG;
    memcpy(data, &s_data, sizeof(gnss_data_t));
    return ESP_OK;
}

bool gnss_has_fix(void) {
    if (!s_initialized) return false;

    // Check if data is fresh (less than 2 seconds old)
    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t age_ms = now_ms - s_data.timestamp_ms;

    return s_data.valid && (age_ms < DATA_TIMEOUT_MS);
}

bool gnss_cog_valid(void) {
    if (!gnss_has_fix()) return false;
    return s_data.cog_valid;
}

float gnss_get_cog(void) {
    if (!gnss_cog_valid()) return NAN;
    return s_data.cog;
}

float gnss_get_speed_kts(void) {
    if (!gnss_has_fix()) return 0.0f;
    return s_data.speed_kts;
}

esp_err_t gnss_get_position(float *lat, float *lon) {
    if (!gnss_has_fix()) return ESP_ERR_INVALID_STATE;
    if (lat) *lat = s_data.latitude;
    if (lon) *lon = s_data.longitude;
    return ESP_OK;
}

float gnss_get_cog_blend_factor(void) {
    if (!gnss_cog_valid()) return 0.0f;

    float speed = s_data.speed_kts;

    if (speed < COG_BLEND_LOW_KTS) {
        return 0.0f;  // Full compass
    } else if (speed > COG_BLEND_HIGH_KTS) {
        return 1.0f;  // Full COG
    } else {
        // Linear blend
        return (speed - COG_BLEND_LOW_KTS) / (COG_BLEND_HIGH_KTS - COG_BLEND_LOW_KTS);
    }
}

float gnss_blend_heading(float compass_heading) {
    float blend = gnss_get_cog_blend_factor();

    if (blend <= 0.0f) {
        return compass_heading;
    }
    if (blend >= 1.0f) {
        return s_data.cog;
    }

    // Blend the two headings, handling wrap-around
    float diff = wrap180(s_data.cog - compass_heading);
    float blended = compass_heading + blend * diff;
    return wrap360(blended);
}

uint32_t gnss_get_bytes_received(void) {
    return s_bytes_received;
}

uint32_t gnss_get_message_count(void) {
    return s_message_count;
}

uint32_t gnss_get_error_count(void) {
    return s_error_count;
}
