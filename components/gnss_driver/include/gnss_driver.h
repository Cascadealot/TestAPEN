/**
 * @file gnss_driver.h
 * @brief u-blox GNSS Driver for TestAP2
 *
 * FSD Reference: TestAP2.FSD.v1.0.0.md Section 10
 *
 * Supports u-blox M8/M9/M10 series GNSS modules via UART.
 * Uses UBX protocol for NAV-PVT messages at 10Hz.
 */

#ifndef GNSS_DRIVER_H
#define GNSS_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * GNSS Fix Types (UBX-NAV-PVT fixType field)
 *============================================================================*/

typedef enum {
    GNSS_FIX_NONE = 0,          // No fix
    GNSS_FIX_DEAD_RECKONING = 1, // Dead reckoning only
    GNSS_FIX_2D = 2,            // 2D fix
    GNSS_FIX_3D = 3,            // 3D fix
    GNSS_FIX_GPS_DR = 4,        // GPS + dead reckoning
    GNSS_FIX_TIME_ONLY = 5      // Time only fix
} gnss_fix_type_t;

/*============================================================================
 * GNSS Data Structure
 *============================================================================*/

typedef struct {
    // Position
    float latitude;              // Degrees (positive = North)
    float longitude;             // Degrees (positive = East)
    float altitude;              // Meters above MSL

    // Velocity
    float speed_kts;             // Speed over ground in knots
    float cog;                   // Course over ground in degrees (0-360)
    float cog_accuracy;          // COG accuracy in degrees

    // Fix information
    gnss_fix_type_t fix_type;    // Fix type
    uint8_t num_satellites;      // Number of satellites used
    float hdop;                  // Horizontal dilution of precision

    // Timing
    uint32_t utc_time;           // UTC time (HHMMSS * 100 + centiseconds)
    uint16_t year;
    uint8_t month;
    uint8_t day;

    // Status flags
    bool valid;                  // Data is valid
    bool cog_valid;              // COG is valid for steering
    uint32_t timestamp_ms;       // Local timestamp when data was received
} gnss_data_t;

/*============================================================================
 * API Functions
 *============================================================================*/

/**
 * @brief Initialize GNSS driver
 *
 * Configures UART2 for u-blox module communication.
 * Sends UBX commands to configure module for 115200 baud and NAV-PVT at 10Hz.
 *
 * @param uart_num UART port number (typically UART_NUM_2)
 * @param tx_gpio TX pin GPIO number
 * @param rx_gpio RX pin GPIO number
 * @return ESP_OK on success
 */
esp_err_t gnss_init(int uart_num, int tx_gpio, int rx_gpio);

/**
 * @brief Deinitialize GNSS driver
 */
void gnss_deinit(void);

/**
 * @brief Process incoming GNSS data
 *
 * Call this periodically (recommended: 50Hz or faster) to process UART data.
 * This function is non-blocking and returns immediately if no data available.
 *
 * @return ESP_OK if data was processed, ESP_ERR_NOT_FOUND if no data
 */
esp_err_t gnss_update(void);

/**
 * @brief Get latest GNSS data
 *
 * @param data Pointer to structure to fill with GNSS data
 * @return ESP_OK on success
 */
esp_err_t gnss_get_data(gnss_data_t *data);

/**
 * @brief Check if GNSS has valid fix
 *
 * @return true if fix is 2D or 3D with data less than 2 seconds old
 */
bool gnss_has_fix(void);

/**
 * @brief Check if COG is valid for steering
 *
 * COG is valid when:
 * - Speed > 1.5 knots
 * - COG accuracy < 10 degrees
 * - Fix is valid
 *
 * @return true if COG can be used for steering
 */
bool gnss_cog_valid(void);

/**
 * @brief Get COG for steering
 *
 * Returns the COG (Course Over Ground) if valid, or NaN if not.
 *
 * @return COG in degrees (0-360) or NaN
 */
float gnss_get_cog(void);

/**
 * @brief Get speed over ground
 *
 * @return Speed in knots, or 0 if not valid
 */
float gnss_get_speed_kts(void);

/**
 * @brief Get position
 *
 * @param lat Pointer to store latitude (degrees, + = North)
 * @param lon Pointer to store longitude (degrees, + = East)
 * @return ESP_OK if position is valid
 */
esp_err_t gnss_get_position(float *lat, float *lon);

/*============================================================================
 * COG Blending (FSD Section 10.4)
 *============================================================================*/

/**
 * @brief Get COG blend factor based on speed
 *
 * Returns how much to weight COG vs compass heading:
 * - Speed < 1.5 kts: 0.0 (full compass)
 * - Speed 1.5-3.0 kts: linear blend 0.0-1.0
 * - Speed > 3.0 kts: 1.0 (full COG)
 *
 * @return Blend factor 0.0 (compass only) to 1.0 (COG only)
 */
float gnss_get_cog_blend_factor(void);

/**
 * @brief Blend compass heading with COG based on speed
 *
 * @param compass_heading Compass heading in degrees (0-360)
 * @return Blended heading in degrees (0-360)
 */
float gnss_blend_heading(float compass_heading);

/*============================================================================
 * Debug/Status Functions
 *============================================================================*/

/**
 * @brief Get number of bytes received since init
 */
uint32_t gnss_get_bytes_received(void);

/**
 * @brief Get number of valid NAV-PVT messages received
 */
uint32_t gnss_get_message_count(void);

/**
 * @brief Get number of parse errors
 */
uint32_t gnss_get_error_count(void);

#ifdef __cplusplus
}
#endif

#endif // GNSS_DRIVER_H
