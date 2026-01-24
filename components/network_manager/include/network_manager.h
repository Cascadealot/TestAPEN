/**
 * @file network_manager.h
 * @brief WiFi, OTA, and Debug Console for TestAPEN
 */

#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Command callback function type
 * @param cmd The command string (null-terminated)
 * @param response Buffer to write response
 * @param response_size Size of response buffer
 * @return Number of bytes written to response
 */
typedef size_t (*network_command_callback_t)(const char *cmd, char *response, size_t response_size);

/**
 * @brief Initialize network manager (WiFi + OTA + Debug Server)
 * @param ssid WiFi SSID
 * @param password WiFi password
 * @param hostname OTA hostname
 * @param debug_port TCP port for debug console
 * @return ESP_OK on success
 */
esp_err_t network_manager_init(const char *ssid, const char *password,
                               const char *hostname, uint16_t debug_port);

/**
 * @brief Set command callback for debug console
 * @param callback Function to call when command received
 */
void network_manager_set_command_callback(network_command_callback_t callback);

/**
 * @brief Check if WiFi is connected
 * @return true if connected
 */
bool network_manager_is_connected(void);

/**
 * @brief Get IP address string
 * @param buf Buffer to write IP string
 * @param buf_size Size of buffer
 */
void network_manager_get_ip(char *buf, size_t buf_size);

/**
 * @brief Send message to connected debug client
 * @param msg Message to send (null-terminated)
 */
void network_manager_send(const char *msg);

/**
 * @brief Printf-style send to debug client
 * @param fmt Format string
 * @param ... Arguments
 */
void network_manager_printf(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif // NETWORK_MANAGER_H
