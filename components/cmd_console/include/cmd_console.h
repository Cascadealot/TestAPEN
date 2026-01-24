/**
 * @file cmd_console.h
 * @brief Debug Console Command Handler for TestAPEN
 */

#ifndef CMD_CONSOLE_H
#define CMD_CONSOLE_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize console command handler
 */
void console_init(void);

/**
 * @brief Process a console command
 * @param cmd Command string (null-terminated)
 * @param response Buffer to write response
 * @param response_size Size of response buffer
 * @return Number of bytes written to response
 */
size_t console_process_command(const char *cmd, char *response, size_t response_size);

/**
 * @brief Handle periodic console tasks (e.g., pending reboot)
 */
void console_handle(void);

#ifdef __cplusplus
}
#endif

#endif // CMD_CONSOLE_H
