/**
 * @file epaper.h
 * @brief e-Paper Display Driver for SSD1680 (2.9" 296x128)
 */

#ifndef EPAPER_H
#define EPAPER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Display dimensions
#define EPAPER_WIDTH    296
#define EPAPER_HEIGHT   128

// Framebuffer size (1 bit per pixel)
#define EPAPER_FB_SIZE  ((EPAPER_WIDTH * EPAPER_HEIGHT) / 8)

/**
 * @brief Initialize e-Paper display
 * @return ESP_OK on success
 */
esp_err_t epaper_init(void);

/**
 * @brief Clear display to white
 */
void epaper_clear(void);

/**
 * @brief Clear display to black
 */
void epaper_clear_black(void);

/**
 * @brief Update display with framebuffer contents (full refresh)
 * Full refresh takes ~3s but eliminates ghosting
 */
void epaper_update(void);

/**
 * @brief Update only a partial region of the display (fast refresh)
 * Partial refresh takes ~0.4s but may have minor ghosting
 * After 5 partial refreshes, automatically does full refresh
 *
 * @param x Starting X coordinate (0-295)
 * @param y Starting Y coordinate (0-127)
 * @param w Width of region
 * @param h Height of region
 */
void epaper_update_partial(int x, int y, int w, int h);

/**
 * @brief Update only the dirty region of the display
 * Uses tracked dirty region from set_pixel calls
 * Falls back to full refresh if region >50% of display
 */
void epaper_update_dirty(void);

/**
 * @brief Reset partial refresh mode (forces next update to be full)
 */
void epaper_reset_partial(void);

/**
 * @brief Mark the entire display as dirty
 */
void epaper_mark_dirty(void);

/**
 * @brief Set a pixel in the framebuffer
 * @param x X coordinate (0-295)
 * @param y Y coordinate (0-127)
 * @param black true for black, false for white
 */
void epaper_set_pixel(int x, int y, bool black);

/**
 * @brief Fill the entire framebuffer
 * @param black true for black, false for white
 */
void epaper_fill(bool black);

/**
 * @brief Draw a character at position
 * @param x X coordinate
 * @param y Y coordinate
 * @param c Character to draw
 * @param size Font size (1=8x8, 2=16x16)
 */
void epaper_draw_char(int x, int y, char c, int size);

/**
 * @brief Draw a string at position
 * @param x X coordinate
 * @param y Y coordinate
 * @param str String to draw
 * @param size Font size (1=8x8, 2=16x16)
 */
void epaper_draw_string(int x, int y, const char *str, int size);

/**
 * @brief Draw formatted string
 * @param x X coordinate
 * @param y Y coordinate
 * @param size Font size
 * @param fmt Format string
 */
void epaper_printf(int x, int y, int size, const char *fmt, ...);

/**
 * @brief Put display into deep sleep mode
 */
void epaper_sleep(void);

/**
 * @brief Wake display from deep sleep
 */
void epaper_wake(void);

/**
 * @brief Check if display is busy
 * @return true if busy
 */
bool epaper_is_busy(void);

/**
 * @brief Get pointer to framebuffer
 * @return Pointer to framebuffer
 */
uint8_t* epaper_get_framebuffer(void);

/**
 * @brief Draw diagnostic test pattern
 * Draws border, crosshairs, corner markers, and labels
 */
void epaper_test_pattern(void);

#ifdef __cplusplus
}
#endif

#endif // EPAPER_H
