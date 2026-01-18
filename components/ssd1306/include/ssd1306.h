/**
 * @file ssd1306.h
 * @brief SSD1306 OLED Display Driver for ESP-IDF
 *
 * Simple text-only driver for 128x64 or 128x32 OLED displays.
 * Uses 8x8 font, supporting 16 characters x 4 lines (128x32) or 8 lines (128x64).
 */

#ifndef SSD1306_H
#define SSD1306_H

#include "esp_err.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// I2C Address
#define SSD1306_I2C_ADDR    0x3C

/**
 * @brief Initialize SSD1306 display
 * @param i2c_num I2C port number
 * @param i2c_mutex Mutex for I2C access (can be NULL)
 * @param height Display height in pixels (32 or 64)
 * @return ESP_OK on success
 */
esp_err_t ssd1306_init(i2c_port_t i2c_num, SemaphoreHandle_t i2c_mutex, uint8_t height);

/**
 * @brief Clear the display
 */
void ssd1306_clear(void);

/**
 * @brief Write text to a specific line
 * @param line Line number (0-3 for 32px, 0-7 for 64px)
 * @param text Text to display (max 16 characters)
 */
void ssd1306_write_line(uint8_t line, const char *text);

/**
 * @brief Printf-style write to a specific line
 * @param line Line number
 * @param fmt Format string
 * @param ... Arguments
 */
void ssd1306_printf(uint8_t line, const char *fmt, ...);

/**
 * @brief Update display with current buffer
 */
void ssd1306_update(void);

/**
 * @brief Set display contrast
 * @param contrast Contrast value (0-255)
 */
void ssd1306_set_contrast(uint8_t contrast);

/**
 * @brief Turn display on or off
 * @param on true to turn on, false to turn off
 */
void ssd1306_display_on(bool on);

#ifdef __cplusplus
}
#endif

#endif // SSD1306_H
