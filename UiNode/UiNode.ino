// UiNode.ino
// User interface node, buttons, display and canbus
// - CAN (SN65HVD230-style board on ESP32 TWAI)
// - microSD card on HSPI
// - 2.9" e-Paper (Ingcool) using GxEPD2
// - 6 buttons
//
// This version uses the user's confirmed pinout:
//   CAN: TX=26, RX=27
//   microSD (HSPI): SCK=14, MISO=25, MOSI=13, CS=2
//   E-Paper (VSPI): CS=19, DC=18, RST=5, BUSY=17, CLK=4, DIN(MOSI)=23
//   Buttons (active LOW, external 10k pull-ups): 39, 34, 35, 32, 33, 36
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "src/conf/credentials.h"
#include "src/bcOta/bcOta.h"
#include "src/bcDebugNet/bcDebugNet.h"



/*
// ===== Debug / OTA Feature Flags =====
#define BC_ENABLE_WIFI_OTA 1  // Set to 0 to completely remove OTA code at compile-time
#if BC_ENABLE_WIFI_OTA
#include <WiFi.h>
#include <ArduinoOTA.h>
#endif

// Runtime flag: are we in OTA / maintenance mode this boot?
static bool g_otaMode = false;
*/

// ---------- CAN / TWAI ----------
#include "driver/twai.h"

// ---------- E-paper display ----------
#include <GxEPD2_BW.h>
#include <GxEPD2_3C.h>  // harmless include; commonly present in examples

// ---------- Version info ----------
static const char* APP_NAME = "UiNode";
static const char* APP_VERSION = "v0.0.2";  // bumped
static const char* APP_BUILD = __DATE__ " " __TIME__;



// Wi-Fi credentials for bench/network OTA
#define BC_WIFI_SSID "Boathouse24"
#define BC_WIFI_PASSWORD "Landfall40!@"


// ======================= Pin Mapping =======================

// CAN (SN65HVD230 or compatible)
#define PIN_CAN_TX 5
#define PIN_CAN_RX 4

// microSD over HSPI
#define PIN_SD_SCK 14
#define PIN_SD_MISO 25
#define PIN_SD_MOSI 13
#define PIN_SD_CS 2

// e-Paper display (VSPI via global SPI)
/*
#define PIN_EP_CS 19
#define PIN_EP_DC 18
#define PIN_EP_RST 5
#define PIN_EP_BUSY 17
#define PIN_EP_SCK 4    // CLK
#define PIN_EP_MOSI 23  // DIN
*/
#define PIN_EP_CS 17
#define PIN_EP_DC 19
#define PIN_EP_RST 27
#define PIN_EP_BUSY 3
#define PIN_EP_SCK 18   // CLK
#define PIN_EP_MOSI 23  // DIN


// 6 buttons (active LOW) all with external 10k pull-up resistors to the +ve rail.
#define PIN_BTN_1 39
#define PIN_BTN_2 34
#define PIN_BTN_3 35
#define PIN_BTN_4 32
#define PIN_BTN_5 33
#define PIN_BTN_6 36


// Duplicate use of the Button attached to GPIO 36 (Buttong '6')
#define PIN_BTN_MAINT 36      // The chosen GPIO
#define BTN_ACTIVE_LEVEL LOW  // Because this button has extnal resistor to +ve, button pulls the pin LOW when pressed



// *** NEW *** UI_AP_CMD CAN ID and command types
static const uint16_t CAN_ID_UI_AP_CMD = 0x220;
static const uint8_t UI_CMD_SET_MODE = 1;
static const uint8_t UI_CMD_NUDGE_HEADING = 2;
static const uint8_t UI_CMD_SET_ABS_HEADING = 3;  // reserved for future



// *** NEW *** Behavior modes
static const uint8_t UI_CMD_SET_BEHAVIOR = 4;
static const uint8_t UI_BEHAV_COMPASS = 0;
static const uint8_t UI_BEHAV_TRACK = 1;
static const uint8_t UI_BEHAV_WAYPT = 2;


// *** NEW *** Autopilot mode values (must match NavMaster/bcAp)
static const uint8_t UI_MODE_STANDBY = 0;
static const uint8_t UI_MODE_HEADING_HOLD = 1;


// *** NEW *** Local AP mode cache so B3 can toggle ENGAGE/DISENGAGE
uint8_t ui_ap_mode = UI_MODE_STANDBY;

// *** NEW *** Local behavior index
uint8_t ui_ap_behavior = UI_BEHAV_COMPASS;


// ======================= Button config & state =======================

const uint8_t NUM_BUTTONS = 6;
const uint8_t BTN_PINS[NUM_BUTTONS] = { PIN_BTN_1, PIN_BTN_2, PIN_BTN_3, PIN_BTN_4, PIN_BTN_5, PIN_BTN_6 };
const char* BTN_NAMES[NUM_BUTTONS] = { "B1", "B2", "B3", "B4", "B5", "B6" };
const uint16_t BTN_DEBOUNCE_MS = 25;  // debounce filter
const uint16_t BTN_LONG_MS = 800;     // long-press threshold (ms)

struct ButtonState {
  bool stableLevel;       // current debounced level (HIGH=released, LOW=pressed)
  bool lastRawLevel;      // last raw digitalRead() level
  uint32_t lastEdgeMs;    // millis() when raw level last changed
  uint32_t pressStartMs;  // millis() when button became stably pressed
  bool longReported;      // has long-press already been reported?
};

ButtonState g_btn[NUM_BUTTONS];
uint8_t g_stableMask = 0;      // debounced "pressed" bits
uint8_t g_lastStableMask = 0;  // for chord edge detection

// ======================= Globals =======================

// SD uses a dedicated HSPI bus
SPIClass spiSD(HSPI);

bool g_canOK = false;
bool g_sdOK = false;
bool g_epdOK = false;

// GxEPD2 display object: use the driver that matches your Ingcool 2.9" panel.
GxEPD2_BW<GxEPD2_290_T94_V2, GxEPD2_290_T94_V2::HEIGHT> display(
  GxEPD2_290_T94_V2(PIN_EP_CS, PIN_EP_DC, PIN_EP_RST, PIN_EP_BUSY));

// ======================= Helpers =======================


/*
// ----------  OTA helper functions ----------
#if BC_ENABLE_WIFI_OTA

void bcOta_log(const char* msg) {
  // Central place to send OTA-related debug
  Serial.println(msg);
}

void setupOtaIfNeeded() {
  if (!g_otaMode) {
    bcOta_log("[OTA] OTA mode disabled this boot.");
    return;
  }

  bcOta_log("[OTA] Starting Wi-Fi for OTA...");

  WiFi.mode(WIFI_STA);
  WiFi.begin(BC_WIFI_SSID, BC_WIFI_PASSWORD);

  unsigned long start = millis();
  const unsigned long timeoutMs = 15000;  // 15s timeout

  while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeoutMs) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    bcOta_log("[OTA] Wi-Fi connect FAILED. OTA disabled this boot.");
    g_otaMode = false;
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    return;
  }

  Serial.print("[OTA] Wi-Fi connected. IP: ");
  Serial.println(WiFi.localIP());

  ArduinoOTA
    .onStart([]() {
      Serial.println("[OTA] Start");
    })
    .onEnd([]() {
      Serial.println("\n[OTA] End");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("[OTA] Progress: %u%%\r", (progress * 100) / total);
    })
    .onError([](ota_error_t error) {
      Serial.printf("[OTA] Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.setHostname("UiNode-OTA");  // Appears as 'UiNode-OTA' in Arduino IDE
  ArduinoOTA.begin();

  bcOta_log("[OTA] Ready. Use Arduino IDE Network Port: UiNode-OTA");
}

void handleOtaIfNeeded() {
  if (!g_otaMode) return;
  ArduinoOTA.handle();
}

#endif  // BC_ENABLE_WIFI_OTA
*/

//---------- General Helper functions -----------

void printBanner() {
  Serial.println();
  Serial.println(F("========================================"));
  Serial.print(F("[UiNode] "));
  Serial.print(APP_NAME);
  Serial.print(F(" "));
  Serial.println(APP_VERSION);
  Serial.print(F("Build: "));
  Serial.println(APP_BUILD);
  Serial.println(F("========================================"));
  Serial.println();
}

void printPinout() {
  Serial.println(F("Pin mapping:"));
  Serial.print(F("  CAN TX="));
  Serial.println(PIN_CAN_TX);
  Serial.print(F("  CAN RX="));
  Serial.println(PIN_CAN_RX);

  Serial.print(F("  SD  SCK="));
  Serial.println(PIN_SD_SCK);
  Serial.print(F("  SD MISO="));
  Serial.println(PIN_SD_MISO);
  Serial.print(F("  SD MOSI="));
  Serial.println(PIN_SD_MOSI);
  Serial.print(F("  SD  CS ="));
  Serial.println(PIN_SD_CS);

  Serial.print(F("  EP CS  ="));
  Serial.println(PIN_EP_CS);
  Serial.print(F("  EP DC  ="));
  Serial.println(PIN_EP_DC);
  Serial.print(F("  EP RST ="));
  Serial.println(PIN_EP_RST);
  Serial.print(F("  EP BUSY="));
  Serial.println(PIN_EP_BUSY);
  Serial.print(F("  EP SCK ="));
  Serial.println(PIN_EP_SCK);
  Serial.print(F("  EP MOSI="));
  Serial.println(PIN_EP_MOSI);

  Serial.print(F("  BTN1   ="));
  Serial.println(PIN_BTN_1);
  Serial.print(F("  BTN2   ="));
  Serial.println(PIN_BTN_2);
  Serial.print(F("  BTN3   ="));
  Serial.println(PIN_BTN_3);
  Serial.print(F("  BTN4   ="));
  Serial.println(PIN_BTN_4);
  Serial.print(F("  BTN5   ="));
  Serial.println(PIN_BTN_5);
  Serial.print(F("  BTN6   ="));
  Serial.println(PIN_BTN_6);
  Serial.println();
}

// ---------- Button utilities ----------

uint8_t countBits(uint8_t m) {
  uint8_t c = 0;
  while (m) {
    c += (m & 0x01);
    m >>= 1;
  }
  return c;
}

void initButtons() {
  // All with external pull-ups: INPUT is correct.
  for (uint8_t i = 0; i < NUM_BUTTONS; ++i) {
    pinMode(BTN_PINS[i], INPUT);

    int raw = digitalRead(BTN_PINS[i]);
    bool level = (raw != 0);  // HIGH = released
    g_btn[i].stableLevel = level;
    g_btn[i].lastRawLevel = level;
    g_btn[i].lastEdgeMs = millis();
    g_btn[i].pressStartMs = 0;
    g_btn[i].longReported = false;
  }

  g_stableMask = 0;
  g_lastStableMask = 0;

  Serial.println(F("[BTN] Buttons initialized (active LOW, debounced)."));
}

void reportButtons(uint8_t mask) {
  Serial.print(F("Buttons: "));
  for (uint8_t i = 0; i < NUM_BUTTONS; ++i) {
    bool pressed = mask & (1 << i);
    Serial.print(BTN_NAMES[i]);
    Serial.print(pressed ? F("=P ") : F("=. "));
  }
  Serial.println();
}

// *** NEW *** Send UI->AP command frame (0x220)
void sendUiApCmd(uint8_t cmdType, uint8_t arg1, int16_t arg2_i16) {
  if (!g_canOK) return;

  twai_message_t msg;
  memset(&msg, 0, sizeof(msg));
  msg.identifier = CAN_ID_UI_AP_CMD;
  msg.extd = 0;
  msg.rtr = 0;
  msg.data_length_code = 4;

  msg.data[0] = cmdType;
  msg.data[1] = arg1;
  msg.data[2] = (uint8_t)(arg2_i16 & 0xFF);
  msg.data[3] = (uint8_t)((arg2_i16 >> 8) & 0xFF);

  esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(10));
  if (err == ESP_OK) {
    Serial.print(F("[CAN] UI_AP_CMD sent: type="));
    Serial.print(cmdType);
    Serial.print(F(" arg1="));
    Serial.print(arg1);
    Serial.print(F(" arg2="));
    Serial.println(arg2_i16);
  } else {
    Serial.print(F("[CAN] UI_AP_CMD send failed: "));
    Serial.println((int)err);
  }
}

// Poll buttons every loop: debounce + short/long press + chord detection
void pollButtons() {
  uint32_t now = millis();
  uint8_t newMask = 0;

  for (uint8_t i = 0; i < NUM_BUTTONS; ++i) {
    int raw = digitalRead(BTN_PINS[i]);
    bool rawLevel = (raw != 0);  // HIGH=released, LOW=pressed

    if (rawLevel != g_btn[i].lastRawLevel) {
      // raw transition just happened, reset edge timer
      g_btn[i].lastRawLevel = rawLevel;
      g_btn[i].lastEdgeMs = now;
    } else {
      // raw level stable; check if it has been stable long enough to accept as debounced
      if (rawLevel != g_btn[i].stableLevel && (now - g_btn[i].lastEdgeMs) >= BTN_DEBOUNCE_MS) {
        bool previousStable = g_btn[i].stableLevel;
        g_btn[i].stableLevel = rawLevel;

        // ----- State transitions -----
        if (previousStable == HIGH && rawLevel == LOW) {
          // Button has just been stably PRESSED
          g_btn[i].pressStartMs = now;
          g_btn[i].longReported = false;
          Serial.print(F("[BTN] "));
          Serial.print(BTN_NAMES[i]);
          Serial.println(F(" pressed"));
        } else if (previousStable == LOW && rawLevel == HIGH) {
          // Button has just been stably RELEASED
          uint32_t dur = now - g_btn[i].pressStartMs;

          if (!g_btn[i].longReported) {
            if (dur < BTN_LONG_MS) {
              Serial.print(F("[BTN] "));
              Serial.print(BTN_NAMES[i]);
              Serial.print(F(" SHORT release ("));
              Serial.print(dur);
              Serial.println(F(" ms)"));

              // *** NEW LOGIC *** Map SHORT releases to AP commands
              switch (i) {

                case 0:  // B1 short: -10°
                  sendUiApCmd(UI_CMD_NUDGE_HEADING, 0, -100);
                  break;

                case 1:  // B2 short: +10°
                  sendUiApCmd(UI_CMD_NUDGE_HEADING, 0, +100);
                  break;

                case 2:  // B3 short: ENGAGE/DISENGAGE
                  if (ui_ap_mode == UI_MODE_STANDBY) {
                    ui_ap_mode = UI_MODE_HEADING_HOLD;
                    sendUiApCmd(UI_CMD_SET_MODE, UI_MODE_HEADING_HOLD, 0);
                  } else {
                    ui_ap_mode = UI_MODE_STANDBY;
                    sendUiApCmd(UI_CMD_SET_MODE, UI_MODE_STANDBY, 0);
                  }
                  break;

                case 3:  // B4 short: -1°
                  sendUiApCmd(UI_CMD_NUDGE_HEADING, 0, -10);
                  break;

                case 4:  // B5 short: +1°
                  sendUiApCmd(UI_CMD_NUDGE_HEADING, 0, +10);
                  break;

                case 5:  // B6 short: Change Behavior ( UI_BEHAV_COMPASS = 0; UI_BEHAV_TRACK = 1 UI_BEHAV_WAYPT = 2;)
                  if (ui_ap_behavior == UI_BEHAV_COMPASS) {
                    sendUiApCmd(UI_CMD_SET_BEHAVIOR, UI_BEHAV_TRACK, 0);
                  } else if (ui_ap_behavior == UI_BEHAV_TRACK) {
                    sendUiApCmd(UI_CMD_SET_BEHAVIOR, UI_BEHAV_WAYPT, 0);
                  } else {
                    sendUiApCmd(UI_CMD_SET_BEHAVIOR, UI_BEHAV_COMPASS, 0);
                  }
                  break;

                  static const uint8_t UI_BEHAV_COMPASS = 0;
                  static const uint8_t UI_BEHAV_TRACK = 1;
                  static const uint8_t UI_BEHAV_WAYPT = 2;
              }
            }
            // If dur >= BTN_LONG_MS but longReported==false, we just missed the long edge;
            // we could still treat as LONG release if you want. For now, only mid-hold.
          }

          Serial.print(F("[BTN] "));
          Serial.print(BTN_NAMES[i]);
          Serial.println(F(" released"));
        }
      }
    }

    // Long-press detection (while held stably LOW)
    if (g_btn[i].stableLevel == LOW && !g_btn[i].longReported) {
      uint32_t held = now - g_btn[i].pressStartMs;
      if (held >= BTN_LONG_MS) {
        g_btn[i].longReported = true;
        Serial.print(F("[BTN] "));
        Serial.print(BTN_NAMES[i]);
        Serial.print(F(" LONG press ("));
        Serial.print(held);
        Serial.println(F(" ms)"));

        // *** NEW *** Map LONG presses to AP commands
        // *** NEW LONG PRESS LOGIC ***
        switch (i) {
          case 2:                                       // B3 long: cycle behavior mode
            ui_ap_behavior = (ui_ap_behavior + 1) % 3;  // 0→1→2→0
            sendUiApCmd(UI_CMD_SET_BEHAVIOR, ui_ap_behavior, 0);

            Serial.print(F("[BTN] B3 LONG: behavior = "));
            Serial.println(ui_ap_behavior);
            break;

          default:
            break;
        }
      }
    }

    // Build the debounced "pressed" mask
    if (g_btn[i].stableLevel == LOW) {
      newMask |= (1 << i);
    }
  }

  // ----- Chord detection (2 or more buttons pressed together) -----
  if (newMask != g_lastStableMask) {
    uint8_t newBits = newMask;
    uint8_t oldBits = g_lastStableMask;
    uint8_t newCount = countBits(newBits);
    uint8_t oldCount = countBits(oldBits);

    if (newCount > 1 && oldCount <= 1) {
      // Chord started
      Serial.print(F("[BTN] chord start: "));
      bool first = true;
      for (uint8_t i = 0; i < NUM_BUTTONS; ++i) {
        if (newBits & (1 << i)) {
          if (!first) Serial.print(F("+"));
          Serial.print(BTN_NAMES[i]);
          first = false;
        }
      }
      Serial.println();
    } else if (newCount <= 1 && oldCount > 1) {
      // Chord ended
      Serial.print(F("[BTN] chord end: "));
      bool first = true;
      for (uint8_t i = 0; i < NUM_BUTTONS; ++i) {
        if (oldBits & (1 << i)) {
          if (!first) Serial.print(F("+"));
          Serial.print(BTN_NAMES[i]);
          first = false;
        }
      }
      Serial.println();
    }

    g_lastStableMask = newMask;
  }

  g_stableMask = newMask;
}

uint8_t getButtonMask() {
  return g_stableMask;
}

// ---------- CAN (TWAI) ----------

bool initCAN() {
  Serial.println(F("[CAN] Initializing TWAI..."));

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
    (gpio_num_t)PIN_CAN_TX,
    (gpio_num_t)PIN_CAN_RX,
    TWAI_MODE_NORMAL);
  // twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();

  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
  if (err != ESP_OK) {
    Serial.print(F("[CAN] twai_driver_install failed, err="));
    Serial.println((int)err);
    return false;
  }

  err = twai_start();
  if (err != ESP_OK) {
    Serial.print(F("[CAN] twai_start failed, err="));
    Serial.println((int)err);
    return false;
  }

  Serial.println(F("[CAN] TWAI started OK (250 kbps)."));
  return true;
}

void sendCanHeartbeat() {
  if (!g_canOK) return;

  twai_message_t msg;
  memset(&msg, 0, sizeof(msg));
  msg.identifier = 0x123;  // arbitrary test ID
  msg.extd = 0;
  msg.data_length_code = 4;
  msg.data[0] = 'U';
  msg.data[1] = 'I';
  msg.data[2] = 'N';
  msg.data[3] = 'D';

  esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(10));
  if (err == ESP_OK) {
    Serial.println(F("[CAN] Heartbeat frame sent."));
  } else {
    Serial.print(F("[CAN] Heartbeat send failed: "));
    Serial.println(err);
  }
}

// ---------- microSD ----------

bool initSD() {
  Serial.println(F("[SD] Initializing SD over HSPI..."));

  // HSPI for SD card
  spiSD.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);

  if (!SD.begin(PIN_SD_CS, spiSD)) {
    Serial.println(F("[SD] SD.begin() failed. Check wiring, power, and CS pin."));
    return false;
  }

  uint64_t cardSize = SD.cardSize() / (1024ULL * 1024ULL);
  Serial.print(F("[SD] Card size: "));
  Serial.print(cardSize);
  Serial.println(F(" MB"));

  // Quick root listing
  File root = SD.open("/");
  if (!root) {
    Serial.println(F("[SD] Failed to open root directory."));
    return true;  // Card still OK, just no root listing
  }
  Serial.println(F("[SD] Root directory:"));
  File file = root.openNextFile();
  while (file) {
    Serial.print(F("  "));
    Serial.print(file.name());
    if (file.isDirectory()) {
      Serial.println(F("/"));
    } else {
      Serial.print(F("  "));
      Serial.print(file.size());
      Serial.println(F(" bytes"));
    }
    file = root.openNextFile();
  }
  root.close();
  Serial.println(F("[SD] SD init OK."));
  return true;
}

// ---------- E-paper ----------

bool initEPD() {
  Serial.println(F("[EPD] Initializing e-Paper..."));

  // Configure global SPI (VSPI) for the E-Paper panel pins
  SPI.begin(PIN_EP_SCK, -1, PIN_EP_MOSI, PIN_EP_CS);

  // Now init the display; GxEPD2 will use the global SPI object
  display.init();  // default timing; adjust if needed for your panel
  display.setRotation(1);
  display.setTextSize(2);
  display.setFont(NULL);              // use built-in 5x7 font
  display.setTextColor(GxEPD_BLACK);  // ensure text is black, not white

  display.setFullWindow();
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);

    int16_t x = 4;
    int16_t y = 4;

    display.setCursor(x, y);
    display.print(F("UiNode Smoke Test"));

    y += 12;
    display.setCursor(x, y);
    display.print(APP_VERSION);

    y += 12;
    display.setCursor(x, y);
    display.print(F("Build:"));
    display.setCursor(x + 40, y);
    display.print(APP_BUILD);

    y += 16;
    display.setCursor(x, y);
    display.print(F("CAN: "));
    display.print(g_canOK ? F("OK") : F("FAIL"));

    y += 12;
    display.setCursor(x, y);
    display.print(F("SD:  "));
    display.print(g_sdOK ? F("OK") : F("FAIL"));

    y += 12;
    display.setCursor(x, y);
    display.print(F("EPD: "));
    display.print(F("SELF"));

    y += 16;
    display.setCursor(x, y);
    display.print(F("Buttons: B1..B6"));

  } while (display.nextPage());

  Serial.println(F("[EPD] Initial screen drawn."));
  return true;
}
/*
bool initOta() {

  // --- Configure maintenance / OTA button ---
  pinMode(PIN_BTN_MAINT, INPUT);  // or INPUT_PULLUP if that matches your wiring

  // Sample the button *once* at boot to decide OTA mode
  int btnState = digitalRead(PIN_BTN_MAINT);
  bool btnPressed = (btnState == BTN_ACTIVE_LEVEL);

  if (btnPressed) {
    g_otaMode = true;
    Serial.println("[BOOT] Maintenance / OTA mode requested via button.");
  } else {
    g_otaMode = false;
    Serial.println("[BOOT] Normal mode (OTA off).");
  }

#if BC_ENABLE_WIFI_OTA
  setupOtaIfNeeded();
  return true;
#endif
  return false;
}
*/

// ======================= Arduino lifecycle =======================

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  delay(250);

  /*
  bool otaOk = initOta();
  Serial.print("OTA init: ");
  if (otaOk) {
    Serial.println("Ok");
  } else {
    Serial.println("Okfailed");
  };
*/

  /*
  // ---- OTA / Maintenance mode ----
  bcOtaConfig otaCfg;
  otaCfg.maintButtonPin = PIN_BTN_MAINT;
  otaCfg.maintActiveLevel = BTN_ACTIVE_LEVEL;
  otaCfg.wifiSsid = BC_WIFI_SSID;
  otaCfg.wifiPassword = BC_WIFI_PASSWORD;
  otaCfg.hostname = "UiNode-OTA";

  bcOta::begin(otaCfg);
*/

  bcOtaConfig cfg = {
    .maintButtonPin = -1,  // no button needed
    .maintActiveLevel = HIGH,
    .wifiSsid = mySSID,
    .wifiPassword = myPASSWORD,
    .hostname = "UiNode",
    .mode = bcOtaMode::ALWAYS_ON
  };

  bcOta::begin(cfg);

  // Network debugging second
    bcDebugNetConfig dbgCfg = {
        .port = 2323,
        .mirrorToSerial = true
    };
    bcDebugNet::begin(dbgCfg);

    bcDebugNet::println("[BOOT] UiNode with bcDebugNet online.");



  printBanner();
  printPinout();

  initButtons();

  // Initialize subsystems
  g_canOK = initCAN();
  g_sdOK = initSD();
  g_epdOK = initEPD();

  Serial.print(F("[STATUS] CAN="));
  Serial.print(g_canOK ? F("OK") : F("FAIL"));
  Serial.print(F(" SD="));
  Serial.print(g_sdOK ? F("OK") : F("FAIL"));
  Serial.print(F(" EPD="));
  Serial.println(g_epdOK ? F("OK") : F("FAIL"));
}

void loop() {
  /*
#if BC_ENABLE_WIFI_OTA
  handleOtaIfNeeded();
#endif
*/

  // --- Common services first ---
  bcOta::handle();
  bcDebugNet::handle();


  static uint32_t lastButtonReport = 0;
  static uint32_t lastCanHeartbeat = 0;

  uint32_t now = millis();

  // Always keep button state machine running
  pollButtons();

  // Periodic debounced button state report
  if (now - lastButtonReport >= 500) {
    lastButtonReport = now;
    uint8_t mask = getButtonMask();
    reportButtons(mask);
  }

  // Periodic CAN heartbeat
  if (now - lastCanHeartbeat >= 1000) {
    lastCanHeartbeat = now;
    sendCanHeartbeat();
    bcDebugNet::printf("[UiNode] uptime: %lu ms\r\n", millis());
  }
}
