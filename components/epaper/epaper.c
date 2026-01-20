/**
 * @file epaper.c
 * @brief e-Paper Display Driver for SSD1680 (2.9" 296x128)
 *
 * GPIO Configuration (from Kconfig):
 *   BUSY: GPIO 3  (input)
 *   RST:  GPIO 27 (output)
 *   DC:   GPIO 19 (output)
 *   CS:   GPIO 17 (output)
 *   CLK:  GPIO 18 (SPI clock)
 *   DIN:  GPIO 23 (SPI MOSI)
 */

#include "epaper.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

static const char *TAG = "EPAPER";

// GPIO pins from Kconfig
#define PIN_BUSY    CONFIG_TESTAP2_EPAPER_BUSY_GPIO
#define PIN_RST     CONFIG_TESTAP2_EPAPER_RST_GPIO
#define PIN_DC      CONFIG_TESTAP2_EPAPER_DC_GPIO
#define PIN_CS      CONFIG_TESTAP2_EPAPER_CS_GPIO
#define PIN_CLK     CONFIG_TESTAP2_EPAPER_CLK_GPIO
#define PIN_DIN     CONFIG_TESTAP2_EPAPER_DIN_GPIO

// SSD1680 Commands
#define CMD_DRIVER_OUTPUT       0x01
#define CMD_GATE_VOLTAGE        0x03
#define CMD_SOURCE_VOLTAGE      0x04
#define CMD_DEEP_SLEEP          0x10
#define CMD_DATA_ENTRY_MODE     0x11
#define CMD_SW_RESET            0x12
#define CMD_TEMP_SENSOR         0x18
#define CMD_MASTER_ACTIVATE     0x20
#define CMD_DISPLAY_UPDATE_1    0x21
#define CMD_DISPLAY_UPDATE_2    0x22
#define CMD_WRITE_RAM_BW        0x24
#define CMD_WRITE_RAM_RED       0x26
#define CMD_VCOM_SENSE          0x28
#define CMD_VCOM_DURATION       0x29
#define CMD_WRITE_VCOM          0x2C
#define CMD_READ_OTP            0x2D
#define CMD_READ_STATUS         0x2F
#define CMD_WRITE_LUT           0x32
#define CMD_SET_DUMMY           0x3A
#define CMD_SET_GATE_TIME       0x3B
#define CMD_BORDER_WAVEFORM     0x3C
#define CMD_SET_RAM_X           0x44
#define CMD_SET_RAM_Y           0x45
#define CMD_SET_RAM_X_COUNTER   0x4E
#define CMD_SET_RAM_Y_COUNTER   0x4F

// Static variables
static spi_device_handle_t s_spi = NULL;
static uint8_t s_framebuffer[EPAPER_FB_SIZE];
static bool s_initialized = false;
static bool s_partial_mode = false;
static int s_partial_refresh_count = 0;

// Dirty region tracking (in landscape coordinates)
static int s_dirty_x1 = 0, s_dirty_y1 = 0;
static int s_dirty_x2 = -1, s_dirty_y2 = -1;  // -1 means no dirty region

// Partial refresh LUT (159 bytes) - from ESPHome PR #5481 for SSD1680
// Correct size is 159 bytes for SSD1680 controller
// This LUT provides fast partial refresh with minimal ghosting
static const uint8_t lut_partial[159] = {
    // LUT0: LUTC (12 bytes)
    0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // LUT1: LUTWW (12 bytes)
    0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // LUT2: LUTKW/LUTR (12 bytes)
    0x40, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // LUT3: LUTWK/LUTW (12 bytes)
    0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // LUT4: LUTKK/LUTB (12 bytes)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // Timing: 12 groups x 7 bytes = 84 bytes
    0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // TP0
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // TP1
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // TP2
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // TP3
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // TP4
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // TP5
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // TP6
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // TP7
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // TP8
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // TP9
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // TP10
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // TP11
    // Frame rate settings (9 bytes)
    0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00,
    // XON settings (6 bytes) - required for 159 byte total
    0x22, 0x17, 0x41, 0x00, 0x32, 0x36,
};

// Simple 8x8 font (ASCII 32-127)
static const uint8_t font_8x8[][8] = {
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // 32 space
    {0x18,0x3C,0x3C,0x18,0x18,0x00,0x18,0x00}, // 33 !
    {0x36,0x36,0x00,0x00,0x00,0x00,0x00,0x00}, // 34 "
    {0x36,0x36,0x7F,0x36,0x7F,0x36,0x36,0x00}, // 35 #
    {0x0C,0x3E,0x03,0x1E,0x30,0x1F,0x0C,0x00}, // 36 $
    {0x00,0x63,0x33,0x18,0x0C,0x66,0x63,0x00}, // 37 %
    {0x1C,0x36,0x1C,0x6E,0x3B,0x33,0x6E,0x00}, // 38 &
    {0x06,0x06,0x03,0x00,0x00,0x00,0x00,0x00}, // 39 '
    {0x18,0x0C,0x06,0x06,0x06,0x0C,0x18,0x00}, // 40 (
    {0x06,0x0C,0x18,0x18,0x18,0x0C,0x06,0x00}, // 41 )
    {0x00,0x66,0x3C,0xFF,0x3C,0x66,0x00,0x00}, // 42 *
    {0x00,0x0C,0x0C,0x3F,0x0C,0x0C,0x00,0x00}, // 43 +
    {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C,0x06}, // 44 ,
    {0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x00}, // 45 -
    {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C,0x00}, // 46 .
    {0x60,0x30,0x18,0x0C,0x06,0x03,0x01,0x00}, // 47 /
    {0x3E,0x63,0x73,0x7B,0x6F,0x67,0x3E,0x00}, // 48 0
    {0x0C,0x0E,0x0C,0x0C,0x0C,0x0C,0x3F,0x00}, // 49 1
    {0x1E,0x33,0x30,0x1C,0x06,0x33,0x3F,0x00}, // 50 2
    {0x1E,0x33,0x30,0x1C,0x30,0x33,0x1E,0x00}, // 51 3
    {0x38,0x3C,0x36,0x33,0x7F,0x30,0x78,0x00}, // 52 4
    {0x3F,0x03,0x1F,0x30,0x30,0x33,0x1E,0x00}, // 53 5
    {0x1C,0x06,0x03,0x1F,0x33,0x33,0x1E,0x00}, // 54 6
    {0x3F,0x33,0x30,0x18,0x0C,0x0C,0x0C,0x00}, // 55 7
    {0x1E,0x33,0x33,0x1E,0x33,0x33,0x1E,0x00}, // 56 8
    {0x1E,0x33,0x33,0x3E,0x30,0x18,0x0E,0x00}, // 57 9
    {0x00,0x0C,0x0C,0x00,0x00,0x0C,0x0C,0x00}, // 58 :
    {0x00,0x0C,0x0C,0x00,0x00,0x0C,0x0C,0x06}, // 59 ;
    {0x18,0x0C,0x06,0x03,0x06,0x0C,0x18,0x00}, // 60 <
    {0x00,0x00,0x3F,0x00,0x00,0x3F,0x00,0x00}, // 61 =
    {0x06,0x0C,0x18,0x30,0x18,0x0C,0x06,0x00}, // 62 >
    {0x1E,0x33,0x30,0x18,0x0C,0x00,0x0C,0x00}, // 63 ?
    {0x3E,0x63,0x7B,0x7B,0x7B,0x03,0x1E,0x00}, // 64 @
    {0x0C,0x1E,0x33,0x33,0x3F,0x33,0x33,0x00}, // 65 A
    {0x3F,0x66,0x66,0x3E,0x66,0x66,0x3F,0x00}, // 66 B
    {0x3C,0x66,0x03,0x03,0x03,0x66,0x3C,0x00}, // 67 C
    {0x1F,0x36,0x66,0x66,0x66,0x36,0x1F,0x00}, // 68 D
    {0x7F,0x46,0x16,0x1E,0x16,0x46,0x7F,0x00}, // 69 E
    {0x7F,0x46,0x16,0x1E,0x16,0x06,0x0F,0x00}, // 70 F
    {0x3C,0x66,0x03,0x03,0x73,0x66,0x7C,0x00}, // 71 G
    {0x33,0x33,0x33,0x3F,0x33,0x33,0x33,0x00}, // 72 H
    {0x1E,0x0C,0x0C,0x0C,0x0C,0x0C,0x1E,0x00}, // 73 I
    {0x78,0x30,0x30,0x30,0x33,0x33,0x1E,0x00}, // 74 J
    {0x67,0x66,0x36,0x1E,0x36,0x66,0x67,0x00}, // 75 K
    {0x0F,0x06,0x06,0x06,0x46,0x66,0x7F,0x00}, // 76 L
    {0x63,0x77,0x7F,0x7F,0x6B,0x63,0x63,0x00}, // 77 M
    {0x63,0x67,0x6F,0x7B,0x73,0x63,0x63,0x00}, // 78 N
    {0x1C,0x36,0x63,0x63,0x63,0x36,0x1C,0x00}, // 79 O
    {0x3F,0x66,0x66,0x3E,0x06,0x06,0x0F,0x00}, // 80 P
    {0x1E,0x33,0x33,0x33,0x3B,0x1E,0x38,0x00}, // 81 Q
    {0x3F,0x66,0x66,0x3E,0x36,0x66,0x67,0x00}, // 82 R
    {0x1E,0x33,0x07,0x0E,0x38,0x33,0x1E,0x00}, // 83 S
    {0x3F,0x2D,0x0C,0x0C,0x0C,0x0C,0x1E,0x00}, // 84 T
    {0x33,0x33,0x33,0x33,0x33,0x33,0x3F,0x00}, // 85 U
    {0x33,0x33,0x33,0x33,0x33,0x1E,0x0C,0x00}, // 86 V
    {0x63,0x63,0x63,0x6B,0x7F,0x77,0x63,0x00}, // 87 W
    {0x63,0x63,0x36,0x1C,0x1C,0x36,0x63,0x00}, // 88 X
    {0x33,0x33,0x33,0x1E,0x0C,0x0C,0x1E,0x00}, // 89 Y
    {0x7F,0x63,0x31,0x18,0x4C,0x66,0x7F,0x00}, // 90 Z
    {0x1E,0x06,0x06,0x06,0x06,0x06,0x1E,0x00}, // 91 [
    {0x03,0x06,0x0C,0x18,0x30,0x60,0x40,0x00}, // 92 backslash
    {0x1E,0x18,0x18,0x18,0x18,0x18,0x1E,0x00}, // 93 ]
    {0x08,0x1C,0x36,0x63,0x00,0x00,0x00,0x00}, // 94 ^
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF}, // 95 _
    {0x0C,0x0C,0x18,0x00,0x00,0x00,0x00,0x00}, // 96 `
    {0x00,0x00,0x1E,0x30,0x3E,0x33,0x6E,0x00}, // 97 a
    {0x07,0x06,0x06,0x3E,0x66,0x66,0x3B,0x00}, // 98 b
    {0x00,0x00,0x1E,0x33,0x03,0x33,0x1E,0x00}, // 99 c
    {0x38,0x30,0x30,0x3e,0x33,0x33,0x6E,0x00}, // 100 d
    {0x00,0x00,0x1E,0x33,0x3f,0x03,0x1E,0x00}, // 101 e
    {0x1C,0x36,0x06,0x0f,0x06,0x06,0x0F,0x00}, // 102 f
    {0x00,0x00,0x6E,0x33,0x33,0x3E,0x30,0x1F}, // 103 g
    {0x07,0x06,0x36,0x6E,0x66,0x66,0x67,0x00}, // 104 h
    {0x0C,0x00,0x0E,0x0C,0x0C,0x0C,0x1E,0x00}, // 105 i
    {0x30,0x00,0x30,0x30,0x30,0x33,0x33,0x1E}, // 106 j
    {0x07,0x06,0x66,0x36,0x1E,0x36,0x67,0x00}, // 107 k
    {0x0E,0x0C,0x0C,0x0C,0x0C,0x0C,0x1E,0x00}, // 108 l
    {0x00,0x00,0x33,0x7F,0x7F,0x6B,0x63,0x00}, // 109 m
    {0x00,0x00,0x1F,0x33,0x33,0x33,0x33,0x00}, // 110 n
    {0x00,0x00,0x1E,0x33,0x33,0x33,0x1E,0x00}, // 111 o
    {0x00,0x00,0x3B,0x66,0x66,0x3E,0x06,0x0F}, // 112 p
    {0x00,0x00,0x6E,0x33,0x33,0x3E,0x30,0x78}, // 113 q
    {0x00,0x00,0x3B,0x6E,0x66,0x06,0x0F,0x00}, // 114 r
    {0x00,0x00,0x3E,0x03,0x1E,0x30,0x1F,0x00}, // 115 s
    {0x08,0x0C,0x3E,0x0C,0x0C,0x2C,0x18,0x00}, // 116 t
    {0x00,0x00,0x33,0x33,0x33,0x33,0x6E,0x00}, // 117 u
    {0x00,0x00,0x33,0x33,0x33,0x1E,0x0C,0x00}, // 118 v
    {0x00,0x00,0x63,0x6B,0x7F,0x7F,0x36,0x00}, // 119 w
    {0x00,0x00,0x63,0x36,0x1C,0x36,0x63,0x00}, // 120 x
    {0x00,0x00,0x33,0x33,0x33,0x3E,0x30,0x1F}, // 121 y
    {0x00,0x00,0x3F,0x19,0x0C,0x26,0x3F,0x00}, // 122 z
    {0x38,0x0C,0x0C,0x07,0x0C,0x0C,0x38,0x00}, // 123 {
    {0x18,0x18,0x18,0x00,0x18,0x18,0x18,0x00}, // 124 |
    {0x07,0x0C,0x0C,0x38,0x0C,0x0C,0x07,0x00}, // 125 }
    {0x6E,0x3B,0x00,0x00,0x00,0x00,0x00,0x00}, // 126 ~
};

// Wait for display not busy
static void wait_busy(void) {
    int timeout = 10000;  // 10 seconds max
    int busy_level = gpio_get_level(PIN_BUSY);
    ESP_LOGI(TAG, "Waiting for BUSY (initial level=%d)", busy_level);

    while (busy_level == 1 && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout -= 10;
        busy_level = gpio_get_level(PIN_BUSY);
    }

    if (timeout <= 0) {
        ESP_LOGE(TAG, "BUSY timeout! Pin stuck high - check GPIO %d wiring", PIN_BUSY);
    } else {
        ESP_LOGI(TAG, "BUSY cleared after %dms", 10000 - timeout);
    }
}

// Send command
static void send_command(uint8_t cmd) {
    gpio_set_level(PIN_DC, 0);  // Command mode
    gpio_set_level(PIN_CS, 0);

    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
    };
    spi_device_transmit(s_spi, &t);

    gpio_set_level(PIN_CS, 1);
}

// Send data byte
static void send_data(uint8_t data) {
    gpio_set_level(PIN_DC, 1);  // Data mode
    gpio_set_level(PIN_CS, 0);

    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &data,
    };
    spi_device_transmit(s_spi, &t);

    gpio_set_level(PIN_CS, 1);
}

// Send data buffer
static void send_data_buffer(const uint8_t *data, size_t len) {
    gpio_set_level(PIN_DC, 1);  // Data mode
    gpio_set_level(PIN_CS, 0);

    // Send in chunks (SPI max transaction size)
    size_t chunk_size = 1024;
    size_t offset = 0;

    while (offset < len) {
        size_t remaining = len - offset;
        size_t to_send = (remaining < chunk_size) ? remaining : chunk_size;

        spi_transaction_t t = {
            .length = to_send * 8,
            .tx_buffer = data + offset,
        };
        spi_device_transmit(s_spi, &t);

        offset += to_send;
    }

    gpio_set_level(PIN_CS, 1);
}

// Hardware reset
static void hw_reset(void) {
    ESP_LOGI(TAG, "Hardware reset...");
    gpio_set_level(PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    wait_busy();
    ESP_LOGI(TAG, "Hardware reset complete");
}

// SSD1680 addresses display as 128 columns (X) x 296 rows (Y) - portrait orientation
// We use rotation 1 (landscape) like GxEPD2 to get 296x128
#define NATIVE_WIDTH   128   // X direction - source driver (portrait width)
#define NATIVE_HEIGHT  296   // Y direction - gate driver (portrait height)

// Initialize display - rotation 3 for correct landscape orientation
static void init_display(void) {
    hw_reset();
    wait_busy();

    // Software reset
    send_command(CMD_SW_RESET);
    vTaskDelay(pdMS_TO_TICKS(10));
    wait_busy();

    // Driver output control - set gate driver to 296 lines
    send_command(CMD_DRIVER_OUTPUT);
    send_data((NATIVE_HEIGHT - 1) & 0xFF);         // 0x27 = 295 low byte
    send_data(((NATIVE_HEIGHT - 1) >> 8) & 0xFF);  // 0x01 = 295 high byte
    send_data(0x00);  // Gate scanning sequence and direction

    // Data entry mode: 0x03 = AM=0 (Y first), Y inc, X inc
    // Display fills column by column (all Y for X=0, then X=1, etc.)
    // Framebuffer is organized to match this (column-major)
    send_command(CMD_DATA_ENTRY_MODE);
    send_data(0x03);

    // Set RAM X address range: 0 to 15 (128 pixels / 8)
    send_command(CMD_SET_RAM_X);
    send_data(0x00);                          // X start = 0
    send_data((NATIVE_WIDTH / 8) - 1);        // X end = 15

    // Set RAM Y address range: 0 to 295 (Y incrementing for rotation 3)
    send_command(CMD_SET_RAM_Y);
    send_data(0x00);                                  // Y start low = 0
    send_data(0x00);                                  // Y start high = 0
    send_data((NATIVE_HEIGHT - 1) & 0xFF);            // Y end low = 0x27
    send_data(((NATIVE_HEIGHT - 1) >> 8) & 0xFF);     // Y end high = 0x01

    // Border waveform control
    send_command(CMD_BORDER_WAVEFORM);
    send_data(0x05);

    // Temperature sensor selection
    send_command(CMD_TEMP_SENSOR);
    send_data(0x80);  // Internal temperature sensor

    // Display update control 1
    send_command(CMD_DISPLAY_UPDATE_1);
    send_data(0x00);
    send_data(0x80);

    // Set RAM counters to start position - GxEPD2 starts at (0, 0)
    send_command(CMD_SET_RAM_X_COUNTER);
    send_data(0x00);  // X = 0

    send_command(CMD_SET_RAM_Y_COUNTER);
    send_data(0x00);  // Y = 0 low
    send_data(0x00);  // Y = 0 high

    wait_busy();

    ESP_LOGI(TAG, "Display initialized (rotation 3: %dx%d)", EPAPER_WIDTH, EPAPER_HEIGHT);
}

esp_err_t epaper_init(void) {
    if (s_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing e-Paper display");
    ESP_LOGI(TAG, "BUSY=%d RST=%d DC=%d CS=%d CLK=%d DIN=%d",
             PIN_BUSY, PIN_RST, PIN_DC, PIN_CS, PIN_CLK, PIN_DIN);

    // Configure GPIO pins
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_RST) | (1ULL << PIN_DC) | (1ULL << PIN_CS),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // BUSY pin is input
    io_conf.pin_bit_mask = (1ULL << PIN_BUSY);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    // Set initial states
    gpio_set_level(PIN_RST, 1);
    gpio_set_level(PIN_DC, 0);
    gpio_set_level(PIN_CS, 1);

    // Configure SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_DIN,
        .miso_io_num = -1,
        .sclk_io_num = PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = EPAPER_FB_SIZE + 100,
    };

    esp_err_t ret = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %d", ret);
        return ret;
    }

    // Add SPI device
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 4 * 1000 * 1000,  // 4 MHz
        .mode = 0,
        .spics_io_num = -1,  // We control CS manually
        .queue_size = 1,
    };

    ret = spi_bus_add_device(SPI3_HOST, &devcfg, &s_spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed: %d", ret);
        return ret;
    }

    // Clear framebuffer to white
    memset(s_framebuffer, 0xFF, EPAPER_FB_SIZE);

    // Initialize display
    init_display();

    s_initialized = true;

    // Full refresh to white to clear any artifacts from previous power cycle
    ESP_LOGI(TAG, "Clearing display...");
    epaper_update();

    ESP_LOGI(TAG, "e-Paper initialized successfully");

    return ESP_OK;
}

void epaper_clear(void) {
    memset(s_framebuffer, 0xFF, EPAPER_FB_SIZE);
}

void epaper_clear_black(void) {
    memset(s_framebuffer, 0x00, EPAPER_FB_SIZE);
}

void epaper_fill(bool black) {
    memset(s_framebuffer, black ? 0x00 : 0xFF, EPAPER_FB_SIZE);
}

void epaper_update(void) {
    if (!s_initialized) {
        ESP_LOGW(TAG, "epaper_update: Not initialized!");
        return;
    }

    ESP_LOGI(TAG, "Updating display (sending %d bytes)...", EPAPER_FB_SIZE);

    // Re-set RAM window (matches init_display settings for rotation 3)
    send_command(CMD_SET_RAM_X);
    send_data(0x00);                          // X start = 0
    send_data((NATIVE_WIDTH / 8) - 1);        // X end = 15

    send_command(CMD_SET_RAM_Y);
    send_data(0x00);                                  // Y start = 0 low
    send_data(0x00);                                  // Y start = 0 high
    send_data((NATIVE_HEIGHT - 1) & 0xFF);            // Y end = 295 low
    send_data(((NATIVE_HEIGHT - 1) >> 8) & 0xFF);     // Y end = 295 high

    // Set RAM counters to start position - GxEPD2 starts at (0, 0)
    send_command(CMD_SET_RAM_X_COUNTER);
    send_data(0x00);  // X = 0

    send_command(CMD_SET_RAM_Y_COUNTER);
    send_data(0x00);  // Y = 0 low
    send_data(0x00);  // Y = 0 high

    // Write B/W RAM directly - data entry mode 0x07 expects row-major order
    // which matches our framebuffer organization
    send_command(CMD_WRITE_RAM_BW);
    send_data_buffer(s_framebuffer, EPAPER_FB_SIZE);

    // Display update sequence
    send_command(CMD_DISPLAY_UPDATE_2);
    send_data(0xF7);  // Full update with LUT from OTP

    send_command(CMD_MASTER_ACTIVATE);
    wait_busy();

    ESP_LOGI(TAG, "Display update complete");

    // Reset to full refresh mode and clear dirty region
    s_partial_mode = false;
    s_partial_refresh_count = 0;
    s_dirty_x1 = 0;
    s_dirty_y1 = 0;
    s_dirty_x2 = -1;
    s_dirty_y2 = -1;
}

/**
 * @brief Initialize partial refresh mode with custom LUT
 *
 * Based on ESPHome PR #5481 and Waveshare documentation:
 * - Hardware reset recommended before partial mode
 * - LUT must be 159 bytes for SSD1680
 * - Use command 0x0F for partial refresh
 */
static void init_partial_mode(void) {
    if (s_partial_mode) return;

    ESP_LOGI(TAG, "Initializing partial refresh mode (LUT size=%d)", (int)sizeof(lut_partial));

    // Hardware reset before entering partial mode (recommended by Waveshare)
    hw_reset();
    wait_busy();

    // Software reset
    send_command(CMD_SW_RESET);
    vTaskDelay(pdMS_TO_TICKS(10));
    wait_busy();

    // Re-initialize display configuration after reset
    // Driver output control
    send_command(CMD_DRIVER_OUTPUT);
    send_data((NATIVE_HEIGHT - 1) & 0xFF);
    send_data(((NATIVE_HEIGHT - 1) >> 8) & 0xFF);
    send_data(0x00);

    // Data entry mode: 0x03 = AM=0 (Y first), Y inc, X inc
    send_command(CMD_DATA_ENTRY_MODE);
    send_data(0x03);

    // Border waveform
    send_command(CMD_BORDER_WAVEFORM);
    send_data(0x05);

    // Temperature sensor
    send_command(CMD_TEMP_SENSOR);
    send_data(0x80);

    // Write partial refresh LUT (159 bytes)
    send_command(CMD_WRITE_LUT);
    send_data_buffer(lut_partial, sizeof(lut_partial));
    wait_busy();

    ESP_LOGI(TAG, "Partial mode initialized successfully");
    s_partial_mode = true;
}

/**
 * @brief Update only a partial region of the display (fast, no flash)
 *
 * Coordinates are in landscape mode (0-295 x, 0-127 y)
 * The region will be expanded to byte boundaries.
 *
 * @param x Starting X coordinate
 * @param y Starting Y coordinate
 * @param w Width of region
 * @param h Height of region
 */
void epaper_update_partial(int x, int y, int w, int h) {
    if (!s_initialized) {
        ESP_LOGW(TAG, "epaper_update_partial: Not initialized!");
        return;
    }

    // Clamp to display bounds
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > EPAPER_WIDTH) w = EPAPER_WIDTH - x;
    if (y + h > EPAPER_HEIGHT) h = EPAPER_HEIGHT - y;

    if (w <= 0 || h <= 0) {
        ESP_LOGW(TAG, "Invalid partial region");
        return;
    }

    // After 5 partial refreshes, do a full refresh to clear ghosting
    if (s_partial_refresh_count >= 5) {
        ESP_LOGI(TAG, "Periodic full refresh to clear ghosting");
        epaper_update();
        return;
    }

    // Initialize partial mode if needed
    init_partial_mode();

    // Transform landscape coordinates to native portrait coordinates
    // Landscape (x,y) -> Native (127-y, x)
    // Region in landscape: x to x+w, y to y+h
    // Region in native:
    //   native_x: (127 - (y+h-1)) to (127 - y) = (128-h-y) to (127-y)
    //   native_y: x to (x+w-1)

    int native_x_start = (NATIVE_WIDTH - 1) - (y + h - 1);  // 128 - y - h
    int native_x_end = (NATIVE_WIDTH - 1) - y;              // 127 - y
    int native_y_start = x;
    int native_y_end = x + w - 1;

    // Align native_x to byte boundaries (8 pixels per byte)
    native_x_start = (native_x_start / 8) * 8;
    native_x_end = ((native_x_end + 7) / 8) * 8 - 1;

    int native_w_bytes = (native_x_end - native_x_start + 1) / 8;
    int native_h = native_y_end - native_y_start + 1;

    ESP_LOGI(TAG, "Partial update: landscape(%d,%d,%d,%d) -> native(%d-%d, %d-%d)",
             x, y, w, h, native_x_start, native_x_end, native_y_start, native_y_end);

    // Set RAM X address range (in bytes)
    send_command(CMD_SET_RAM_X);
    send_data(native_x_start / 8);
    send_data(native_x_end / 8);

    // Set RAM Y address range
    send_command(CMD_SET_RAM_Y);
    send_data(native_y_start & 0xFF);
    send_data((native_y_start >> 8) & 0xFF);
    send_data(native_y_end & 0xFF);
    send_data((native_y_end >> 8) & 0xFF);

    // Set RAM counters to start position
    send_command(CMD_SET_RAM_X_COUNTER);
    send_data(native_x_start / 8);

    send_command(CMD_SET_RAM_Y_COUNTER);
    send_data(native_y_start & 0xFF);
    send_data((native_y_start >> 8) & 0xFF);

    // Write only the partial region data
    send_command(CMD_WRITE_RAM_BW);

    for (int row = native_y_start; row <= native_y_end; row++) {
        for (int col_byte = native_x_start / 8; col_byte <= native_x_end / 8; col_byte++) {
            int fb_idx = col_byte + row * (NATIVE_WIDTH / 8);
            send_data(s_framebuffer[fb_idx]);
        }
    }

    // Partial display update sequence
    // 0x0F = partial update using custom LUT (from ESPHome PR #5481)
    // NOT 0xCF or 0xF7 which are for full refresh
    send_command(CMD_DISPLAY_UPDATE_2);
    send_data(0x0F);

    send_command(CMD_MASTER_ACTIVATE);
    wait_busy();

    s_partial_refresh_count++;
    ESP_LOGI(TAG, "Partial update complete (count=%d)", s_partial_refresh_count);
}

/**
 * @brief Update only the dirty region of the display
 *
 * Uses the tracked dirty region from set_pixel calls.
 * Falls back to full update if no dirty region or region is too large.
 */
void epaper_update_dirty(void) {
    if (!s_initialized) {
        ESP_LOGW(TAG, "epaper_update_dirty: Not initialized!");
        return;
    }

    // No dirty region - nothing to update
    if (s_dirty_x2 < 0) {
        ESP_LOGI(TAG, "No dirty region to update");
        return;
    }

    int w = s_dirty_x2 - s_dirty_x1 + 1;
    int h = s_dirty_y2 - s_dirty_y1 + 1;
    int area = w * h;
    int total_area = EPAPER_WIDTH * EPAPER_HEIGHT;

    // If dirty region is more than 50% of display, do full refresh
    if (area > total_area / 2) {
        ESP_LOGI(TAG, "Dirty region >50%% (%d%%), using full refresh",
                 (area * 100) / total_area);
        epaper_update();
        return;
    }

    ESP_LOGI(TAG, "Dirty region: (%d,%d)-(%d,%d) = %d%% of display",
             s_dirty_x1, s_dirty_y1, s_dirty_x2, s_dirty_y2,
             (area * 100) / total_area);

    epaper_update_partial(s_dirty_x1, s_dirty_y1, w, h);

    // Clear dirty region
    s_dirty_x1 = 0;
    s_dirty_y1 = 0;
    s_dirty_x2 = -1;
    s_dirty_y2 = -1;
}

/**
 * @brief Reset partial refresh mode (forces next update to be full refresh)
 */
void epaper_reset_partial(void) {
    s_partial_mode = false;
    s_partial_refresh_count = 0;
}

/**
 * @brief Mark the entire display as dirty
 */
void epaper_mark_dirty(void) {
    s_dirty_x1 = 0;
    s_dirty_y1 = 0;
    s_dirty_x2 = EPAPER_WIDTH - 1;
    s_dirty_y2 = EPAPER_HEIGHT - 1;
}

void epaper_set_pixel(int x, int y, bool black) {
    // LANDSCAPE MODE: 296 wide x 128 tall
    // User coordinates: x = 0-295 (horizontal), y = 0-127 (vertical)
    //
    // Based on GxEPD2/ESPHome/Waveshare research:
    // - Native display is 128x296 portrait
    // - Framebuffer is ROW-MAJOR: byte_idx = (x/8) + y * 16
    // - Bit order is MSB-first: bit_mask = 0x80 >> (x % 8)
    // - For landscape rotation 1 (90° CW): swap then mirror X
    //   rotated_x = NATIVE_WIDTH - 1 - y = 127 - y
    //   rotated_y = x

    if (x < 0 || x >= EPAPER_WIDTH || y < 0 || y >= EPAPER_HEIGHT) {
        return;
    }

    // Apply rotation 1 transformation (landscape)
    int rotated_x = (NATIVE_WIDTH - 1) - y;  // 127 - y (0-127)
    int rotated_y = x;                        // x (0-295)

    // Row-major framebuffer layout (standard for e-paper)
    // 16 bytes per row, 296 rows
    int byte_idx = (rotated_x / 8) + rotated_y * (NATIVE_WIDTH / 8);
    int bit_mask = 0x80 >> (rotated_x % 8);  // MSB-first

    if (byte_idx < 0 || byte_idx >= EPAPER_FB_SIZE) {
        return;
    }

    if (black) {
        s_framebuffer[byte_idx] &= ~bit_mask;  // 0 = black
    } else {
        s_framebuffer[byte_idx] |= bit_mask;   // 1 = white
    }

    // Track dirty region
    if (s_dirty_x2 < 0) {
        s_dirty_x1 = s_dirty_x2 = x;
        s_dirty_y1 = s_dirty_y2 = y;
    } else {
        if (x < s_dirty_x1) s_dirty_x1 = x;
        if (x > s_dirty_x2) s_dirty_x2 = x;
        if (y < s_dirty_y1) s_dirty_y1 = y;
        if (y > s_dirty_y2) s_dirty_y2 = y;
    }
}

void epaper_draw_char(int x, int y, char c, int size) {
    if (c < 32 || c > 126) {
        c = '?';
    }

    const uint8_t *glyph = font_8x8[c - 32];

    // This font uses LSB (bit 0) as leftmost pixel
    // For column 0 (leftmost), we read bit 0
    // For column 7 (rightmost), we read bit 7
    // Only draw BLACK pixels - skip white since background is already white
    for (int row = 0; row < 8; row++) {
        uint8_t row_data = glyph[row];
        for (int col = 0; col < 8; col++) {
            // Check if this pixel should be black (bit set = black)
            if ((row_data >> col) & 0x01) {
                if (size == 1) {
                    epaper_set_pixel(x + col, y + row, true);
                } else {
                    for (int sy = 0; sy < size; sy++) {
                        for (int sx = 0; sx < size; sx++) {
                            epaper_set_pixel(x + col * size + sx, y + row * size + sy, true);
                        }
                    }
                }
            }
        }
    }
}

void epaper_draw_string(int x, int y, const char *str, int size) {
    int char_width = 8 * size;
    int orig_x = x;

    while (*str) {
        if (*str == '\n') {
            x = orig_x;
            y += 8 * size + 2;
        } else {
            epaper_draw_char(x, y, *str, size);
            x += char_width;
        }
        str++;
    }
}

void epaper_printf(int x, int y, int size, const char *fmt, ...) {
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    epaper_draw_string(x, y, buf, size);
}

void epaper_sleep(void) {
    if (!s_initialized) return;

    send_command(CMD_DEEP_SLEEP);
    send_data(0x01);
    ESP_LOGI(TAG, "Display entering deep sleep");
}

void epaper_wake(void) {
    if (!s_initialized) return;

    hw_reset();
    init_display();
    ESP_LOGI(TAG, "Display woken from sleep");
}

bool epaper_is_busy(void) {
    return gpio_get_level(PIN_BUSY) == 1;
}

uint8_t* epaper_get_framebuffer(void) {
    return s_framebuffer;
}

// Test with row-major framebuffer and proper rotation
void epaper_test_pattern(void) {
    ESP_LOGI(TAG, "=== ROW-MAJOR FRAMEBUFFER TEST ===");
    ESP_LOGI(TAG, "Using GxEPD2-style rotation 1 (landscape)");

    // Clear to white
    epaper_clear();

    // Draw border rectangle to show display bounds
    for (int x = 0; x < EPAPER_WIDTH; x++) {
        epaper_set_pixel(x, 0, true);              // Top edge
        epaper_set_pixel(x, EPAPER_HEIGHT-1, true); // Bottom edge
    }
    for (int y = 0; y < EPAPER_HEIGHT; y++) {
        epaper_set_pixel(0, y, true);              // Left edge
        epaper_set_pixel(EPAPER_WIDTH-1, y, true); // Right edge
    }

    // Draw a filled square at top-left (30x30 at position 10,10)
    for (int py = 10; py < 40; py++) {
        for (int px = 10; px < 40; px++) {
            epaper_set_pixel(px, py, true);
        }
    }

    // Draw text using the font
    epaper_draw_string(50, 15, "HELLO", 2);
    epaper_draw_string(50, 50, "E-Paper Test", 1);
    epaper_draw_string(50, 65, "296x128 Landscape", 1);

    // Draw a smaller square at bottom-right
    for (int py = 90; py < 115; py++) {
        for (int px = 250; px < 285; px++) {
            epaper_set_pixel(px, py, true);
        }
    }

    ESP_LOGI(TAG, "Expected: border, top-left square, text, bottom-right square");
    epaper_update();
    ESP_LOGI(TAG, "Test complete");
}
