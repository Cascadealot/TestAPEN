/**
 * @file espnow_protocol.h
 * @brief ESP-NOW Protocol Definitions for TestAPEN
 *
 * This is a port of the CAN protocol from TestAPEN to ESP-NOW wireless.
 * Message structures are identical to CAN for easy migration back.
 */

#ifndef ESPNOW_PROTOCOL_H
#define ESPNOW_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * ESP-NOW Message Types (matching CAN IDs conceptually)
 *============================================================================*/

typedef enum {
    MSG_MASTER_HEARTBEAT    = 0x01,
    MSG_RUDDER_HEARTBEAT    = 0x02,
    MSG_RUDDER_COMMAND      = 0x10,
    MSG_SYSTEM_COMMAND      = 0x11,
    MSG_CALIBRATION_CMD     = 0x12,
    MSG_UI_COMMAND          = 0x13,
    MSG_PARAM_CONFIG        = 0x14,
    MSG_GNSS_POSITION       = 0x20,
    MSG_GNSS_VELOCITY       = 0x21,
    MSG_E_STOP              = 0xE0,
    MSG_RUDDER_EXT_STATUS   = 0x30,
    MSG_PERFORMANCE         = 0x31,
    MSG_MASTER_ERROR        = 0xF1,
    MSG_RUDDER_ERROR        = 0xF2
} espnow_msg_type_t;

/*============================================================================
 * System Commands (FSD Section 5.5.4)
 *============================================================================*/

#define SYS_CMD_ENGAGE              0x01
#define SYS_CMD_DISENGAGE           0x02
#define SYS_CMD_CAL_ENTER           0x10
#define SYS_CMD_CAL_EXIT            0x11
#define SYS_CMD_FAULT_CLEAR         0x20

/*============================================================================
 * Calibration Commands (FSD Section 5.5.6)
 *============================================================================*/

#define CAL_CMD_CENTER              0x01
#define CAL_CMD_PORT                0x02
#define CAL_CMD_STARBOARD           0x03
#define CAL_CMD_SAVE                0x04

/*============================================================================
 * UI Commands (UiNode -> Master)
 *============================================================================*/

#define UI_CMD_ENGAGE               0x01
#define UI_CMD_DISENGAGE            0x02
#define UI_CMD_HEADING_ADJUST       0x10    // value = delta degrees * 10
#define UI_CMD_HEADING_SET          0x11    // value = absolute heading * 10

/*============================================================================
 * Error Codes (FSD Section 5.6)
 *============================================================================*/

typedef enum {
    ERR_NONE                = 0x00,
    ERR_ESPNOW_TX_FAIL      = 0x01,
    ERR_ESPNOW_RX_TIMEOUT   = 0x02,
    ERR_ESPNOW_INIT_FAIL    = 0x03,
    ERR_SENSOR_FAULT        = 0x10,
    ERR_SENSOR_RANGE        = 0x11,
    ERR_SENSOR_INIT         = 0x12,
    ERR_MOTOR_STALL         = 0x20,
    ERR_MOTOR_OVERCURRENT   = 0x21,
    ERR_MOTOR_TIMEOUT       = 0x22,
    ERR_CAL_INVALID         = 0x30,
    ERR_CAL_RANGE           = 0x31,
    ERR_HEARTBEAT_LOST      = 0x40,
    ERR_STATE_MISMATCH      = 0x41,
    ERR_LOW_VOLTAGE         = 0x50,
    ERR_OVER_VOLTAGE        = 0x51,
    ERR_WATCHDOG            = 0xFE,
    ERR_UNKNOWN             = 0xFF
} error_code_t;

typedef enum {
    SEVERITY_NONE = 0,
    SEVERITY_WARNING = 1,
    SEVERITY_FAULT = 2,
    SEVERITY_CRITICAL = 3
} error_severity_t;

/*============================================================================
 * Master Heartbeat Flags (FSD Section 5.5.1)
 *============================================================================*/

#define FLAG_GNSS_VALID         (1 << 0)
#define FLAG_COG_MODE           (1 << 1)
#define FLAG_LAZY_HELM          (1 << 2)
#define FLAG_SEA_AUTO           (1 << 3)
#define FLAG_CALIBRATED         (1 << 4)

/*============================================================================
 * Motor Status Flags (FSD Section 5.5.2)
 *============================================================================*/

#define MOTOR_FLAG_ENABLED      (1 << 0)
#define MOTOR_FLAG_RUNNING      (1 << 1)
#define MOTOR_FLAG_DIRECTION    (1 << 2)
#define MOTOR_FLAG_IN_DEADBAND  (1 << 3)
#define MOTOR_FLAG_AT_LIMIT     (1 << 4)
#define MOTOR_SPEED_MASK        0xE0
#define MOTOR_SPEED_SHIFT       5

/*============================================================================
 * Message Structures (identical to CAN for easy migration)
 *============================================================================*/

// Master Heartbeat (8 bytes)
typedef struct __attribute__((packed)) {
    uint8_t state;
    uint8_t fault_code;
    int16_t heading_x10;    // Big-endian
    int16_t target_x10;     // Big-endian
    uint8_t sequence;
    uint8_t flags;
} espnow_master_heartbeat_t;

// Rudder Heartbeat (8 bytes)
typedef struct __attribute__((packed)) {
    uint8_t state;
    uint8_t fault_code;
    int16_t angle_x10;      // Big-endian
    uint8_t motor_status;
    uint8_t sequence;
    uint16_t reserved;
} espnow_rudder_heartbeat_t;

// Rudder Command (8 bytes)
typedef struct __attribute__((packed)) {
    int16_t cmd_angle_x10;  // Big-endian
    uint8_t flags;
    uint8_t sequence;
    uint32_t reserved;
} espnow_rudder_command_t;

// System Command (8 bytes)
typedef struct __attribute__((packed)) {
    uint8_t command;        // SYS_CMD_* values
    uint8_t reserved[7];
} espnow_system_command_t;

// Calibration Command (8 bytes)
typedef struct __attribute__((packed)) {
    uint8_t command;        // CAL_CMD_* values
    uint8_t reserved[7];
} espnow_calibration_command_t;

// Error Message (4 bytes)
typedef struct __attribute__((packed)) {
    uint8_t code;
    uint8_t severity;
    uint16_t detail;
} espnow_error_msg_t;

// Parameter Config Message (8 bytes)
typedef struct __attribute__((packed)) {
    uint8_t param_id;       // Parameter ID from param_store.h
    uint8_t flags;          // bit 0: save to NVS
    float value;            // Parameter value (little-endian)
    uint16_t reserved;
} espnow_param_config_t;

#define PARAM_FLAG_SAVE_NVS     (1 << 0)

// UI Command Message (8 bytes)
typedef struct __attribute__((packed)) {
    uint8_t command;        // UI_CMD_* values
    int16_t value_x10;      // Command argument (big-endian)
    uint8_t sequence;       // Sequence counter
    uint32_t reserved;
} espnow_ui_command_t;

// GNSS Position Message (8 bytes)
typedef struct __attribute__((packed)) {
    int32_t lat_x1e7;       // Latitude * 1e7 (degrees, big-endian)
    int32_t lon_x1e7;       // Longitude * 1e7 (degrees, big-endian)
} espnow_gnss_position_t;

// GNSS Velocity Message (8 bytes)
typedef struct __attribute__((packed)) {
    uint16_t sog_x100;      // Speed over ground * 100 (knots, big-endian)
    int16_t cog_x10;        // Course over ground * 10 (degrees, big-endian)
    uint8_t fix_type;       // 0=none, 2=2D, 3=3D
    uint8_t satellites;     // Number of satellites
    uint16_t hdop_x100;     // HDOP * 100 (big-endian)
} espnow_gnss_velocity_t;

/*============================================================================
 * ESP-NOW Frame Structure
 *============================================================================*/

#define ESPNOW_MAX_PAYLOAD      8   // Match CAN payload size

typedef struct __attribute__((packed)) {
    uint8_t msg_type;               // espnow_msg_type_t
    uint8_t data[ESPNOW_MAX_PAYLOAD];
} espnow_frame_t;                   // 9 bytes total

/*============================================================================
 * Timing Constants (FSD Section 5.7)
 *============================================================================*/

#define MASTER_HEARTBEAT_INTERVAL_MS    100
#define RUDDER_HEARTBEAT_INTERVAL_MS    20
#define COMMAND_RATE_MS                 100
#define HEARTBEAT_TIMEOUT_MS            500
#define COMMAND_TIMEOUT_MS              200
#define COMMAND_LOSS_FAULT_MS           500
#define E_STOP_RESPONSE_MS              10

/*============================================================================
 * Node Types
 *============================================================================*/

typedef enum {
    NODE_TYPE_MASTER = 0,
    NODE_TYPE_RUDDER = 1,
    NODE_TYPE_UI = 2
} espnow_node_type_t;

/*============================================================================
 * ESP-NOW Status / Diagnostics
 *============================================================================*/

typedef struct {
    bool initialized;
    uint8_t channel;
    uint8_t peer_count;
    uint32_t tx_success_count;
    uint32_t tx_fail_count;
    uint32_t rx_count;
    uint32_t rx_error_count;
    uint8_t own_mac[6];
} espnow_status_t;

/*============================================================================
 * Receive Callback Type
 *============================================================================*/

/**
 * @brief Callback function type for received ESP-NOW messages
 * @param msg_type Message type (espnow_msg_type_t)
 * @param data Pointer to payload data (8 bytes max)
 * @param len Payload length
 * @param src_mac Source MAC address (6 bytes)
 */
typedef void (*espnow_recv_cb_t)(uint8_t msg_type, const uint8_t *data,
                                  size_t len, const uint8_t *src_mac);

/*============================================================================
 * Function Prototypes
 *============================================================================*/

/**
 * @brief Initialize ESP-NOW protocol
 *
 * Must be called after WiFi is initialized and started.
 * Parses MAC addresses from Kconfig and adds peers.
 *
 * @return ESP_OK on success
 */
esp_err_t espnow_init(void);

/**
 * @brief Deinitialize ESP-NOW protocol
 */
void espnow_deinit(void);

/**
 * @brief Send ESP-NOW message
 * @param msg_type Message type (espnow_msg_type_t)
 * @param data Pointer to payload data
 * @param len Payload length (max 8 bytes)
 * @param dest_mac Destination MAC (NULL for broadcast)
 * @return ESP_OK on success
 */
esp_err_t espnow_send(uint8_t msg_type, const uint8_t *data, size_t len,
                       const uint8_t *dest_mac);

/**
 * @brief Register receive callback
 *
 * Only one callback can be registered at a time.
 * Pass NULL to unregister.
 *
 * @param callback Function to call on message receipt
 */
void espnow_register_recv_callback(espnow_recv_cb_t callback);

/**
 * @brief Get Master node MAC address
 * @return Pointer to 6-byte MAC array
 */
const uint8_t* espnow_get_master_mac(void);

/**
 * @brief Get Rudder node MAC address
 * @return Pointer to 6-byte MAC array
 */
const uint8_t* espnow_get_rudder_mac(void);

/**
 * @brief Get UI node MAC address
 * @return Pointer to 6-byte MAC array
 */
const uint8_t* espnow_get_ui_mac(void);

/**
 * @brief Get ESP-NOW status information
 * @param status Pointer to status structure
 */
void espnow_get_status(espnow_status_t *status);

/**
 * @brief Check if a peer is registered
 * @param mac MAC address to check
 * @return true if peer is registered
 */
bool espnow_peer_exists(const uint8_t *mac);

/**
 * @brief Convert int16 to big-endian bytes
 */
void int16_to_be(int16_t value, uint8_t *buf);

/**
 * @brief Convert big-endian bytes to int16
 */
int16_t be_to_int16(const uint8_t *buf);

/**
 * @brief Convert int32 to big-endian bytes
 */
void int32_to_be(int32_t value, uint8_t *buf);

/**
 * @brief Convert big-endian bytes to int32
 */
int32_t be_to_int32(const uint8_t *buf);

/**
 * @brief Format MAC address as string
 * @param mac 6-byte MAC address
 * @param buf Output buffer (at least 18 bytes)
 */
void espnow_mac_to_str(const uint8_t *mac, char *buf);

#ifdef __cplusplus
}
#endif

#endif // ESPNOW_PROTOCOL_H
