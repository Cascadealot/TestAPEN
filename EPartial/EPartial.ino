/**
 * @file EPartial.ino
 * @brief E-Paper Partial Refresh Test Sketch for SSD1680 (2.9" 296x128)
 *
 * Purpose: Test and demonstrate partial screen updates on the e-Paper display
 *          used in the TestAPEN UI Node.
 *
 * Features:
 *   - Partial refresh testing with random numbers (0-360)
 *   - WiFi connectivity
 *   - OTA firmware updates via HTTP POST to /update
 *   - Telnet debug console on port 2323
 *
 * Hardware: ESP32 WROOM-32 + 2.9" e-Paper SSD1680
 *
 * GPIO Configuration (matches UI Node hardware):
 *   CLK:  GPIO 18 (SPI clock)
 *   DIN:  GPIO 23 (SPI MOSI)
 *   CS:   GPIO 17
 *   DC:   GPIO 19
 *   RST:  GPIO 27
 *   BUSY: GPIO 3
 *
 * Network:
 *   OTA:     curl -X POST --data-binary @firmware.bin http://<IP>/update
 *   Console: telnet <IP> 2323
 */

#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>

// ============================================================================
// Configuration - CHANGE THESE FOR YOUR NETWORK
// ============================================================================

const char* WIFI_SSID     = "Boathouse24";
const char* WIFI_PASSWORD = "Waterdog24!";
const char* HOSTNAME      = "epartial";
const int   CONSOLE_PORT  = 2323;

// ============================================================================
// GPIO Pin definitions (matches UI Node hardware)
// ============================================================================

#define PIN_CLK   18
#define PIN_DIN   23
#define PIN_CS    17
#define PIN_DC    19
#define PIN_RST   27
#define PIN_BUSY  3

// Display dimensions (landscape mode)
#define DISPLAY_WIDTH   296
#define DISPLAY_HEIGHT  128

// Native dimensions (portrait - how SSD1680 sees it)
#define NATIVE_WIDTH    128
#define NATIVE_HEIGHT   296

// Framebuffer size (1 bit per pixel)
#define FB_SIZE  ((NATIVE_WIDTH * NATIVE_HEIGHT) / 8)

// SSD1680 Commands
#define CMD_DRIVER_OUTPUT       0x01
#define CMD_DEEP_SLEEP          0x10
#define CMD_DATA_ENTRY_MODE     0x11
#define CMD_SW_RESET            0x12
#define CMD_TEMP_SENSOR         0x18
#define CMD_MASTER_ACTIVATE     0x20
#define CMD_DISPLAY_UPDATE_1    0x21
#define CMD_DISPLAY_UPDATE_2    0x22
#define CMD_WRITE_RAM_BW        0x24
#define CMD_WRITE_LUT           0x32
#define CMD_BORDER_WAVEFORM     0x3C
#define CMD_SET_RAM_X           0x44
#define CMD_SET_RAM_Y           0x45
#define CMD_SET_RAM_X_COUNTER   0x4E
#define CMD_SET_RAM_Y_COUNTER   0x4F

// ============================================================================
// Global state
// ============================================================================

static uint8_t framebuffer[FB_SIZE];
static bool partialMode = false;
static int partialCount = 0;
static int lastValue = -1;
static bool autoUpdateEnabled = true;
static unsigned long lastUpdateTime = 0;
static const unsigned long UPDATE_INTERVAL = 3000;  // 3 seconds

// Network
WebServer httpServer(80);
WiFiServer telnetServer(CONSOLE_PORT);
WiFiClient telnetClient;
String cmdBuffer = "";

// Partial refresh LUT (159 bytes) - optimized for fast partial updates
static const uint8_t lut_partial[159] = {
    // LUT0: LUTC (12 bytes)
    0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // LUT1: LUTWW (12 bytes) - White to White
    0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // LUT2: LUTKW/LUTR (12 bytes) - Black to White
    0x40, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // LUT3: LUTWK/LUTW (12 bytes) - White to Black
    0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // LUT4: LUTKK/LUTB (12 bytes) - Black to Black
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // Timing: 12 groups x 7 bytes = 84 bytes
    0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // Frame rate settings (9 bytes)
    0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00,
    // XON settings (6 bytes)
    0x22, 0x17, 0x41, 0x00, 0x32, 0x36,
};

// Simple 8x8 font for digits 0-9 and degree symbol
static const uint8_t font_8x8_digits[][8] = {
    {0x3E,0x63,0x73,0x7B,0x6F,0x67,0x3E,0x00}, // 0
    {0x0C,0x0E,0x0C,0x0C,0x0C,0x0C,0x3F,0x00}, // 1
    {0x1E,0x33,0x30,0x1C,0x06,0x33,0x3F,0x00}, // 2
    {0x1E,0x33,0x30,0x1C,0x30,0x33,0x1E,0x00}, // 3
    {0x38,0x3C,0x36,0x33,0x7F,0x30,0x78,0x00}, // 4
    {0x3F,0x03,0x1F,0x30,0x30,0x33,0x1E,0x00}, // 5
    {0x1C,0x06,0x03,0x1F,0x33,0x33,0x1E,0x00}, // 6
    {0x3F,0x33,0x30,0x18,0x0C,0x0C,0x0C,0x00}, // 7
    {0x1E,0x33,0x33,0x1E,0x33,0x33,0x1E,0x00}, // 8
    {0x1E,0x33,0x33,0x3E,0x30,0x18,0x0E,0x00}, // 9
    {0x1C,0x36,0x36,0x1C,0x00,0x00,0x00,0x00}, // degree symbol (index 10)
};

// ============================================================================
// Low-level SPI functions
// ============================================================================

void waitBusy() {
    Serial.print("Waiting for BUSY...");
    unsigned long start = millis();
    while (digitalRead(PIN_BUSY) == HIGH) {
        delay(10);
        if (millis() - start > 10000) {
            Serial.println(" TIMEOUT!");
            return;
        }
    }
    Serial.printf(" done (%lu ms)\n", millis() - start);
}

void sendCommand(uint8_t cmd) {
    digitalWrite(PIN_DC, LOW);
    digitalWrite(PIN_CS, LOW);
    SPI.transfer(cmd);
    digitalWrite(PIN_CS, HIGH);
}

void sendData(uint8_t data) {
    digitalWrite(PIN_DC, HIGH);
    digitalWrite(PIN_CS, LOW);
    SPI.transfer(data);
    digitalWrite(PIN_CS, HIGH);
}

void sendDataBuffer(const uint8_t* data, size_t len) {
    digitalWrite(PIN_DC, HIGH);
    digitalWrite(PIN_CS, LOW);
    for (size_t i = 0; i < len; i++) {
        SPI.transfer(data[i]);
    }
    digitalWrite(PIN_CS, HIGH);
}

void hwReset() {
    Serial.println("Hardware reset...");
    digitalWrite(PIN_RST, LOW);
    delay(10);
    digitalWrite(PIN_RST, HIGH);
    delay(10);
    waitBusy();
}

// ============================================================================
// Display initialization
// ============================================================================

void initDisplay() {
    hwReset();
    waitBusy();

    sendCommand(CMD_SW_RESET);
    delay(10);
    waitBusy();

    sendCommand(CMD_DRIVER_OUTPUT);
    sendData((NATIVE_HEIGHT - 1) & 0xFF);
    sendData((NATIVE_HEIGHT - 1) >> 8);
    sendData(0x00);

    sendCommand(CMD_DATA_ENTRY_MODE);
    sendData(0x03);

    sendCommand(CMD_SET_RAM_X);
    sendData(0x00);
    sendData((NATIVE_WIDTH / 8) - 1);

    sendCommand(CMD_SET_RAM_Y);
    sendData(0x00);
    sendData(0x00);
    sendData((NATIVE_HEIGHT - 1) & 0xFF);
    sendData((NATIVE_HEIGHT - 1) >> 8);

    sendCommand(CMD_BORDER_WAVEFORM);
    sendData(0x05);

    sendCommand(CMD_TEMP_SENSOR);
    sendData(0x80);

    sendCommand(CMD_DISPLAY_UPDATE_1);
    sendData(0x00);
    sendData(0x80);

    sendCommand(CMD_SET_RAM_X_COUNTER);
    sendData(0x00);

    sendCommand(CMD_SET_RAM_Y_COUNTER);
    sendData(0x00);
    sendData(0x00);

    waitBusy();
    Serial.println("Display initialized");
}

// ============================================================================
// Framebuffer operations
// ============================================================================

void clearFramebuffer(bool black = false) {
    memset(framebuffer, black ? 0x00 : 0xFF, FB_SIZE);
}

void setPixel(int x, int y, bool black) {
    if (x < 0 || x >= DISPLAY_WIDTH || y < 0 || y >= DISPLAY_HEIGHT) return;

    int rx = (NATIVE_WIDTH - 1) - y;
    int ry = x;

    int byteIdx = (rx / 8) + ry * (NATIVE_WIDTH / 8);
    int bitMask = 0x80 >> (rx % 8);

    if (byteIdx < 0 || byteIdx >= FB_SIZE) return;

    if (black) {
        framebuffer[byteIdx] &= ~bitMask;
    } else {
        framebuffer[byteIdx] |= bitMask;
    }
}

void drawChar(int x, int y, char c, int scale) {
    const uint8_t* glyph;

    if (c >= '0' && c <= '9') {
        glyph = font_8x8_digits[c - '0'];
    } else if (c == '\xB0' || c == 'd') {
        glyph = font_8x8_digits[10];
    } else {
        return;
    }

    for (int row = 0; row < 8; row++) {
        uint8_t rowData = glyph[row];
        for (int col = 0; col < 8; col++) {
            if ((rowData >> col) & 0x01) {
                for (int sy = 0; sy < scale; sy++) {
                    for (int sx = 0; sx < scale; sx++) {
                        setPixel(x + col * scale + sx, y + row * scale + sy, true);
                    }
                }
            }
        }
    }
}

void drawNumber(int x, int y, int value, int scale) {
    char buf[8];
    snprintf(buf, sizeof(buf), "%d", value);

    int charWidth = 8 * scale;
    for (int i = 0; buf[i]; i++) {
        drawChar(x + i * charWidth, y, buf[i], scale);
    }

    int len = strlen(buf);
    drawChar(x + len * charWidth, y, 'd', scale);
}

void fillRect(int x, int y, int w, int h, bool black) {
    for (int py = y; py < y + h; py++) {
        for (int px = x; px < x + w; px++) {
            setPixel(px, py, black);
        }
    }
}

// ============================================================================
// Display update functions
// ============================================================================

void fullUpdate() {
    Serial.println("Full update...");

    sendCommand(CMD_SET_RAM_X);
    sendData(0x00);
    sendData((NATIVE_WIDTH / 8) - 1);

    sendCommand(CMD_SET_RAM_Y);
    sendData(0x00);
    sendData(0x00);
    sendData((NATIVE_HEIGHT - 1) & 0xFF);
    sendData((NATIVE_HEIGHT - 1) >> 8);

    sendCommand(CMD_SET_RAM_X_COUNTER);
    sendData(0x00);

    sendCommand(CMD_SET_RAM_Y_COUNTER);
    sendData(0x00);
    sendData(0x00);

    sendCommand(CMD_WRITE_RAM_BW);
    sendDataBuffer(framebuffer, FB_SIZE);

    sendCommand(CMD_DISPLAY_UPDATE_2);
    sendData(0xF7);

    sendCommand(CMD_MASTER_ACTIVATE);
    waitBusy();

    partialMode = false;
    partialCount = 0;
    Serial.println("Full update complete");
}

void initPartialMode() {
    if (partialMode) return;

    Serial.println("Initializing partial mode...");

    hwReset();
    waitBusy();

    sendCommand(CMD_SW_RESET);
    delay(10);
    waitBusy();

    sendCommand(CMD_DRIVER_OUTPUT);
    sendData((NATIVE_HEIGHT - 1) & 0xFF);
    sendData((NATIVE_HEIGHT - 1) >> 8);
    sendData(0x00);

    sendCommand(CMD_DATA_ENTRY_MODE);
    sendData(0x03);

    sendCommand(CMD_BORDER_WAVEFORM);
    sendData(0x05);

    sendCommand(CMD_TEMP_SENSOR);
    sendData(0x80);

    sendCommand(CMD_WRITE_LUT);
    sendDataBuffer(lut_partial, sizeof(lut_partial));
    waitBusy();

    partialMode = true;
    Serial.println("Partial mode initialized");
}

void partialUpdate(int x, int y, int w, int h) {
    Serial.printf("Partial update region: (%d,%d) %dx%d\n", x, y, w, h);

    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > DISPLAY_WIDTH) w = DISPLAY_WIDTH - x;
    if (y + h > DISPLAY_HEIGHT) h = DISPLAY_HEIGHT - y;

    if (w <= 0 || h <= 0) return;

    if (partialCount >= 5) {
        Serial.println("Periodic full refresh for ghosting");
        fullUpdate();
        return;
    }

    initPartialMode();

    int nativeXStart = (NATIVE_WIDTH - 1) - (y + h - 1);
    int nativeXEnd = (NATIVE_WIDTH - 1) - y;
    int nativeYStart = x;
    int nativeYEnd = x + w - 1;

    nativeXStart = (nativeXStart / 8) * 8;
    nativeXEnd = ((nativeXEnd + 7) / 8) * 8 - 1;

    Serial.printf("Native region: X=%d-%d, Y=%d-%d\n",
                  nativeXStart, nativeXEnd, nativeYStart, nativeYEnd);

    sendCommand(CMD_SET_RAM_X);
    sendData(nativeXStart / 8);
    sendData(nativeXEnd / 8);

    sendCommand(CMD_SET_RAM_Y);
    sendData(nativeYStart & 0xFF);
    sendData(nativeYStart >> 8);
    sendData(nativeYEnd & 0xFF);
    sendData(nativeYEnd >> 8);

    sendCommand(CMD_SET_RAM_X_COUNTER);
    sendData(nativeXStart / 8);

    sendCommand(CMD_SET_RAM_Y_COUNTER);
    sendData(nativeYStart & 0xFF);
    sendData(nativeYStart >> 8);

    sendCommand(CMD_WRITE_RAM_BW);
    for (int row = nativeYStart; row <= nativeYEnd; row++) {
        for (int colByte = nativeXStart / 8; colByte <= nativeXEnd / 8; colByte++) {
            int fbIdx = colByte + row * (NATIVE_WIDTH / 8);
            sendData(framebuffer[fbIdx]);
        }
    }

    sendCommand(CMD_DISPLAY_UPDATE_2);
    sendData(0x0F);

    sendCommand(CMD_MASTER_ACTIVATE);
    waitBusy();

    partialCount++;
    Serial.printf("Partial update complete (count=%d)\n", partialCount);
}

// ============================================================================
// Test display functions
// ============================================================================

#define NUMBER_X      80
#define NUMBER_Y      40
#define NUMBER_SCALE  3
#define NUMBER_WIDTH  (8 * NUMBER_SCALE * 4)
#define NUMBER_HEIGHT (8 * NUMBER_SCALE)

void displayRandomNumber() {
    int value = random(0, 361);

    Serial.printf("\n=== Displaying: %d degrees ===\n", value);

    fillRect(NUMBER_X, NUMBER_Y, NUMBER_WIDTH, NUMBER_HEIGHT, false);
    drawNumber(NUMBER_X, NUMBER_Y, value, NUMBER_SCALE);
    partialUpdate(NUMBER_X, NUMBER_Y, NUMBER_WIDTH, NUMBER_HEIGHT);

    lastValue = value;
}

void displayNumber(int value) {
    Serial.printf("\n=== Displaying: %d degrees ===\n", value);

    fillRect(NUMBER_X, NUMBER_Y, NUMBER_WIDTH, NUMBER_HEIGHT, false);
    drawNumber(NUMBER_X, NUMBER_Y, value, NUMBER_SCALE);
    partialUpdate(NUMBER_X, NUMBER_Y, NUMBER_WIDTH, NUMBER_HEIGHT);

    lastValue = value;
}

void drawStaticContent() {
    for (int x = 0; x < DISPLAY_WIDTH; x++) {
        setPixel(x, 0, true);
        setPixel(x, DISPLAY_HEIGHT - 1, true);
    }
    for (int y = 0; y < DISPLAY_HEIGHT; y++) {
        setPixel(0, y, true);
        setPixel(DISPLAY_WIDTH - 1, y, true);
    }

    for (int x = NUMBER_X - 10; x < NUMBER_X + NUMBER_WIDTH + 10; x++) {
        setPixel(x, NUMBER_Y - 5, true);
        setPixel(x, NUMBER_Y + NUMBER_HEIGHT + 5, true);
    }

    for (int y = NUMBER_Y - 5; y < NUMBER_Y + NUMBER_HEIGHT + 5; y++) {
        setPixel(NUMBER_X - 10, y, true);
        setPixel(NUMBER_X + NUMBER_WIDTH + 10, y, true);
    }
}

// ============================================================================
// WiFi functions
// ============================================================================

void setupWiFi() {
    Serial.printf("Connecting to WiFi: %s\n", WIFI_SSID);

    WiFi.setHostname(HOSTNAME);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected!");
        Serial.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());
        Serial.printf("Hostname: %s\n", HOSTNAME);
    } else {
        Serial.println("\nWiFi connection failed! Continuing without network.");
    }
}

// ============================================================================
// OTA HTTP Server
// ============================================================================

void handleRoot() {
    String html = "EPartial - E-Paper Partial Refresh Test\n";
    html += "IP: " + WiFi.localIP().toString() + "\n";
    html += "Uptime: " + String(millis() / 1000) + " seconds\n";
    html += "Last value: " + String(lastValue) + " degrees\n";
    html += "Partial count: " + String(partialCount) + "\n";
    html += "Auto update: " + String(autoUpdateEnabled ? "ON" : "OFF") + "\n";
    html += "\nOTA: curl -X POST --data-binary @firmware.bin http://" + WiFi.localIP().toString() + "/update\n";
    html += "Console: telnet " + WiFi.localIP().toString() + " " + String(CONSOLE_PORT) + "\n";
    httpServer.send(200, "text/plain", html);
}

void handleUpdate() {
    HTTPUpload& upload = httpServer.upload();

    if (upload.status == UPLOAD_FILE_START) {
        Serial.printf("OTA Update Start: %s\n", upload.filename.c_str());
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
            Update.printError(Serial);
        }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
            Update.printError(Serial);
        }
        if ((upload.totalSize % 65536) == 0) {
            Serial.printf("OTA progress: %u bytes\n", upload.totalSize);
        }
    } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) {
            Serial.printf("OTA Success! Total: %u bytes\n", upload.totalSize);
        } else {
            Update.printError(Serial);
        }
    }
}

void handleUpdateEnd() {
    if (Update.hasError()) {
        httpServer.send(500, "text/plain", "OTA Failed!\n");
    } else {
        httpServer.send(200, "text/plain", "OTA Success! Rebooting...\n");
        delay(500);
        ESP.restart();
    }
}

void setupHTTPServer() {
    httpServer.on("/", HTTP_GET, handleRoot);
    httpServer.on("/update", HTTP_POST, handleUpdateEnd, handleUpdate);
    httpServer.begin();
    Serial.println("HTTP server started on port 80");
    Serial.printf("OTA: curl -X POST --data-binary @firmware.bin http://%s/update\n",
                  WiFi.localIP().toString().c_str());
}

// ============================================================================
// Telnet Console
// ============================================================================

void consolePrint(const String& msg) {
    Serial.print(msg);
    if (telnetClient && telnetClient.connected()) {
        telnetClient.print(msg);
    }
}

void consolePrintln(const String& msg) {
    Serial.println(msg);
    if (telnetClient && telnetClient.connected()) {
        telnetClient.println(msg);
    }
}

void processCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;

    Serial.printf("Command: %s\n", cmd.c_str());

    if (cmd == "help") {
        consolePrintln("EPartial Debug Console");
        consolePrintln("Commands:");
        consolePrintln("  help      - Show this help");
        consolePrintln("  status    - Show status");
        consolePrintln("  random    - Display random number (partial refresh)");
        consolePrintln("  show N    - Display number N (0-360)");
        consolePrintln("  full      - Force full refresh");
        consolePrintln("  partial   - Reset partial mode");
        consolePrintln("  auto      - Toggle auto update");
        consolePrintln("  clear     - Clear display");
        consolePrintln("  reboot    - Reboot device");

    } else if (cmd == "status") {
        consolePrintln("=== EPartial Status ===");
        consolePrint("IP: "); consolePrintln(WiFi.localIP().toString());
        consolePrint("Uptime: "); consolePrint(String(millis() / 1000)); consolePrintln(" seconds");
        consolePrint("Last value: "); consolePrint(String(lastValue)); consolePrintln(" degrees");
        consolePrint("Partial count: "); consolePrintln(String(partialCount));
        consolePrint("Partial mode: "); consolePrintln(partialMode ? "YES" : "NO");
        consolePrint("Auto update: "); consolePrintln(autoUpdateEnabled ? "ON" : "OFF");

    } else if (cmd == "random") {
        displayRandomNumber();
        consolePrint("Displayed: "); consolePrint(String(lastValue)); consolePrintln(" degrees");

    } else if (cmd.startsWith("show ")) {
        int value = cmd.substring(5).toInt();
        if (value >= 0 && value <= 360) {
            displayNumber(value);
            consolePrint("Displayed: "); consolePrint(String(value)); consolePrintln(" degrees");
        } else {
            consolePrintln("Error: Value must be 0-360");
        }

    } else if (cmd == "full") {
        consolePrintln("Forcing full refresh...");
        fullUpdate();
        consolePrintln("Done");

    } else if (cmd == "partial") {
        partialMode = false;
        partialCount = 0;
        consolePrintln("Partial mode reset");

    } else if (cmd == "auto") {
        autoUpdateEnabled = !autoUpdateEnabled;
        consolePrint("Auto update: "); consolePrintln(autoUpdateEnabled ? "ON" : "OFF");

    } else if (cmd == "clear") {
        consolePrintln("Clearing display...");
        clearFramebuffer();
        drawStaticContent();
        fullUpdate();
        consolePrintln("Done");

    } else if (cmd == "reboot") {
        consolePrintln("Rebooting...");
        delay(500);
        ESP.restart();

    } else {
        consolePrint("Unknown command: "); consolePrintln(cmd);
        consolePrintln("Type 'help' for commands");
    }
}

void handleTelnet() {
    // Check for new client
    if (telnetServer.hasClient()) {
        if (telnetClient && telnetClient.connected()) {
            telnetClient.stop();
        }
        telnetClient = telnetServer.available();
        Serial.println("Telnet client connected");
        telnetClient.println("EPartial Debug Console");
        telnetClient.println("Type 'help' for commands");
        telnetClient.print("> ");
        cmdBuffer = "";
    }

    // Read from client
    if (telnetClient && telnetClient.connected()) {
        while (telnetClient.available()) {
            char c = telnetClient.read();
            if (c == '\n' || c == '\r') {
                if (cmdBuffer.length() > 0) {
                    telnetClient.println();
                    processCommand(cmdBuffer);
                    telnetClient.print("> ");
                    cmdBuffer = "";
                }
            } else if (c == 127 || c == 8) {  // Backspace
                if (cmdBuffer.length() > 0) {
                    cmdBuffer.remove(cmdBuffer.length() - 1);
                    telnetClient.print("\b \b");
                }
            } else if (c >= 32 && c < 127) {
                cmdBuffer += c;
                telnetClient.print(c);
            }
        }
    }
}

void setupTelnet() {
    telnetServer.begin();
    telnetServer.setNoDelay(true);
    Serial.printf("Telnet console on port %d\n", CONSOLE_PORT);
}

// ============================================================================
// Arduino setup and loop
// ============================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n========================================");
    Serial.println("E-Paper Partial Refresh Test");
    Serial.println("SSD1680 296x128 Display");
    Serial.println("With WiFi, OTA, and Console");
    Serial.println("========================================\n");

    // Initialize GPIO
    pinMode(PIN_CS, OUTPUT);
    pinMode(PIN_DC, OUTPUT);
    pinMode(PIN_RST, OUTPUT);
    pinMode(PIN_BUSY, INPUT);

    digitalWrite(PIN_CS, HIGH);
    digitalWrite(PIN_RST, HIGH);

    // Initialize SPI
    SPI.begin(PIN_CLK, -1, PIN_DIN, -1);
    SPI.setFrequency(4000000);

    // Initialize display
    initDisplay();

    // Clear and draw initial content
    clearFramebuffer();
    drawStaticContent();
    drawNumber(NUMBER_X, NUMBER_Y, 0, NUMBER_SCALE);

    Serial.println("Performing initial full refresh...");
    fullUpdate();

    // Setup WiFi and network services
    setupWiFi();
    if (WiFi.status() == WL_CONNECTED) {
        setupHTTPServer();
        setupTelnet();
    }

    delay(1000);
    randomSeed(analogRead(0) ^ millis());

    Serial.println("\nStarting partial refresh test loop...");
    Serial.println("Auto-update every 3 seconds (toggle with 'auto' command)\n");

    lastUpdateTime = millis();
}

void loop() {
    // Handle network
    if (WiFi.status() == WL_CONNECTED) {
        httpServer.handleClient();
        handleTelnet();
    }

    // Auto update
    if (autoUpdateEnabled && (millis() - lastUpdateTime >= UPDATE_INTERVAL)) {
        displayRandomNumber();
        lastUpdateTime = millis();
    }

    delay(10);
}
