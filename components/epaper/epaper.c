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

    // Data entry mode - exact GxEPD2 settings:
    // 0x03 = "x increase, y increase, normal mode"
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

    // Write B/W RAM
    send_command(CMD_WRITE_RAM_BW);
    send_data_buffer(s_framebuffer, EPAPER_FB_SIZE);

    // Display update sequence
    send_command(CMD_DISPLAY_UPDATE_2);
    send_data(0xF7);  // Full update with LUT from OTP

    send_command(CMD_MASTER_ACTIVATE);
    wait_busy();

    ESP_LOGI(TAG, "Display update complete");
}

void epaper_set_pixel(int x, int y, bool black) {
    // LANDSCAPE MODE: 296 wide x 128 tall (matching GxEPD2 setRotation(1))
    // User coordinates: x = 0-295 (horizontal), y = 0-127 (vertical)
    //
    // Adafruit_GFX rotation 1 formula:
    //   native_x = WIDTH - 1 - y   (WIDTH = native width = 128)
    //   native_y = x
    //
    // Native portrait: 128 wide (X) x 296 tall (Y)

    if (x < 0 || x >= EPAPER_WIDTH || y < 0 || y >= EPAPER_HEIGHT) {
        return;
    }

    // Transform to native portrait coordinates (Adafruit rotation 1)
    int native_x = (NATIVE_WIDTH - 1) - y;  // 127 - y
    int native_y = x;

    // Framebuffer: row-major, 16 bytes per row, MSB = leftmost pixel
    int byte_idx = (native_x / 8) + (native_y * 16);
    int bit_idx = 7 - (native_x % 8);  // MSB = leftmost pixel

    if (byte_idx < 0 || byte_idx >= EPAPER_FB_SIZE) {
        return;
    }

    if (black) {
        s_framebuffer[byte_idx] &= ~(1 << bit_idx);  // 0 = black
    } else {
        s_framebuffer[byte_idx] |= (1 << bit_idx);   // 1 = white
    }
}

void epaper_draw_char(int x, int y, char c, int size) {
    if (c < 32 || c > 126) {
        c = '?';
    }

    const uint8_t *glyph = font_8x8[c - 32];

    // No rotation here - set_pixel handles coordinate transformation
    // Font: row 0-7 = top to bottom, col 0-7 = left to right
    // Bit 0 = leftmost pixel of each row (font is designed this way)
    for (int row = 0; row < 8; row++) {
        for (int col = 0; col < 8; col++) {
            bool black = (glyph[row] >> col) & 0x01;

            if (size == 1) {
                epaper_set_pixel(x + col, y + row, black);
            } else {
                for (int sy = 0; sy < size; sy++) {
                    for (int sx = 0; sx < size; sx++) {
                        epaper_set_pixel(x + col * size + sx, y + row * size + sy, black);
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

// Simple rectangle test - no text, just a rectangle
void epaper_test_pattern(void) {
    ESP_LOGI(TAG, "=== SIMPLE RECTANGLE TEST ===");

    epaper_clear();  // All white

    // Draw a simple filled rectangle in the center
    // Rectangle: 100 pixels wide, 50 pixels tall, centered
    int rect_w = 100;
    int rect_h = 50;
    int rect_x = (EPAPER_WIDTH - rect_w) / 2;   // ~98
    int rect_y = (EPAPER_HEIGHT - rect_h) / 2;  // ~39

    ESP_LOGI(TAG, "Drawing rectangle at (%d,%d) size %dx%d", rect_x, rect_y, rect_w, rect_h);

    for (int y = rect_y; y < rect_y + rect_h; y++) {
        for (int x = rect_x; x < rect_x + rect_w; x++) {
            epaper_set_pixel(x, y, true);  // black
        }
    }

    epaper_update();
    ESP_LOGI(TAG, "Rectangle test complete");
}
