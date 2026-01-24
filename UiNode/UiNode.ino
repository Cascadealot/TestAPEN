// UiNode.ino
// User interface node with e-Paper partial refresh support
// - 2.9" e-Paper (296x128) using GxEPD2 with partial refresh
// - 6 buttons for autopilot control
// - WiFi OTA and telnet debug console
//
// FSD v1.1.0 compliant - Section 13 E-Paper Display System

#include <Arduino.h>
#include <SPI.h>
#include "src/conf/credentials.h"
#include "src/bcOta/bcOta.h"
#include "src/bcDebugNet/bcDebugNet.h"

// ---------- E-paper display ----------
#include <GxEPD2_BW.h>

// ---------- Version info ----------
static const char* APP_NAME = "UiNode";
static const char* APP_VERSION = "v0.1.0";  // Partial refresh update
static const char* APP_BUILD = __DATE__ " " __TIME__;

// ======================= Pin Mapping =======================
// E-Paper display (working pinout from EPaperTest)
#define PIN_EP_CS    17
#define PIN_EP_DC    19
#define PIN_EP_RST   27
#define PIN_EP_BUSY  3
#define PIN_EP_SCK   18
#define PIN_EP_MOSI  23

// 6 buttons (active LOW with external pull-ups)
#define PIN_BTN_DEC10   39  // -10 degrees
#define PIN_BTN_DEC1    34  // -1 degree
#define PIN_BTN_ENGAGE  35  // Engage/Disengage
#define PIN_BTN_MODE    32  // Mode/Page cycle
#define PIN_BTN_INC1    33  // +1 degree
#define PIN_BTN_INC10   36  // +10 degrees

// ======================= Display Layout Constants =======================
// Display: 296x128 pixels, rotation(1) = landscape
// TextSize(2): ~12x16 pixels per character
static const int16_t SCREEN_WIDTH = 296;
static const int16_t SCREEN_HEIGHT = 128;
static const int16_t CHAR_WIDTH = 12;
static const int16_t CHAR_HEIGHT = 16;
static const int16_t LINE_HEIGHT = 20;
static const int16_t X_MARGIN = 4;
static const int16_t Y_START = 2;

// Line Y positions
static const int16_t LINE1_Y = Y_START;
static const int16_t LINE2_Y = Y_START + LINE_HEIGHT;
static const int16_t LINE3_Y = Y_START + 2 * LINE_HEIGHT;
static const int16_t LINE4_Y = Y_START + 3 * LINE_HEIGHT;
static const int16_t LINE5_Y = Y_START + 4 * LINE_HEIGHT;
static const int16_t LINE6_Y = Y_START + 5 * LINE_HEIGHT;

// ======================= Display Data Structures =======================
struct DisplayData {
  // Node connectivity
  bool masterOK;
  bool rudderOK;
  bool gnssOK;

  // Autopilot data
  uint8_t state;          // 0=BOOT, 1=IDLE, 2=ENGAGED, 3=CAL, 0xFF=FAULT
  float heading;          // Current heading (degrees)
  float target;           // Target heading (degrees)
  float rudderAngle;      // Current rudder angle (degrees)
  float headingError;     // Heading error (degrees)

  // Navigation data
  float latitude;
  float longitude;
  float sog;              // Speed over ground (knots)
  float cog;              // Course over ground (degrees)

  // Display state
  uint8_t currentPage;    // 0=Autopilot, 1=Navigation, 2=System
  uint8_t partialCount;   // Partial refresh counter (force full after 5)
};

static DisplayData g_display = {
  .masterOK = false,
  .rudderOK = false,
  .gnssOK = false,
  .state = 0,
  .heading = 0.0f,
  .target = 0.0f,
  .rudderAngle = 0.0f,
  .headingError = 0.0f,
  .latitude = 0.0f,
  .longitude = 0.0f,
  .sog = 0.0f,
  .cog = 0.0f,
  .currentPage = 0,
  .partialCount = 0
};

// Previous values for change detection
static DisplayData g_prevDisplay;

// ======================= Button State =======================
const uint8_t NUM_BUTTONS = 6;
const uint8_t BTN_PINS[NUM_BUTTONS] = { PIN_BTN_DEC10, PIN_BTN_DEC1, PIN_BTN_ENGAGE,
                                         PIN_BTN_MODE, PIN_BTN_INC1, PIN_BTN_INC10 };
const char* BTN_NAMES[NUM_BUTTONS] = { "-10", "-1", "ENG", "MODE", "+1", "+10" };
const uint16_t BTN_DEBOUNCE_MS = 25;
const uint16_t BTN_LONG_MS = 800;

struct ButtonState {
  bool stableLevel;
  bool lastRawLevel;
  uint32_t lastEdgeMs;
  uint32_t pressStartMs;
  bool longReported;
};

ButtonState g_btn[NUM_BUTTONS];

// ======================= Globals =======================
bool g_epdOK = false;

// GxEPD2 display object
GxEPD2_BW<GxEPD2_290_T94_V2, GxEPD2_290_T94_V2::HEIGHT> display(
  GxEPD2_290_T94_V2(PIN_EP_CS, PIN_EP_DC, PIN_EP_RST, PIN_EP_BUSY));

// ======================= Helper Functions =======================

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

const char* stateToString(uint8_t state) {
  switch (state) {
    case 0: return "BOOT";
    case 1: return "IDLE";
    case 2: return "ENGAGED";
    case 3: return "CAL";
    case 0xFF: return "FAULT";
    default: return "???";
  }
}

// ======================= Display Functions =======================

// Draw status bar (Line 1): "M:OK R:OK G:-- Pg1"
void drawStatusBar() {
  int16_t x = X_MARGIN;
  display.setCursor(x, LINE1_Y);
  display.print(F("M:"));
  display.print(g_display.masterOK ? F("OK") : F("--"));
  display.print(F(" R:"));
  display.print(g_display.rudderOK ? F("OK") : F("--"));
  display.print(F(" G:"));
  display.print(g_display.gnssOK ? F("OK") : F("--"));
  display.print(F(" Pg"));
  display.print(g_display.currentPage + 1);
}

// Draw Page 0 - Autopilot
void drawPage0() {
  int16_t x = X_MARGIN;

  // Line 2: HDG and TGT
  display.setCursor(x, LINE2_Y);
  display.print(F("HDG:"));
  char buf[10];
  dtostrf(g_display.heading, 5, 1, buf);
  display.print(buf);
  display.print(F(" TGT:"));
  dtostrf(g_display.target, 5, 1, buf);
  display.print(buf);

  // Line 3: RUD angle and direction
  display.setCursor(x, LINE3_Y);
  display.print(F("RUD:"));
  dtostrf(g_display.rudderAngle, 5, 1, buf);
  display.print(buf);
  display.print(g_display.rudderAngle < -0.5f ? F(" PORT") :
                (g_display.rudderAngle > 0.5f ? F(" STBD") : F(" CNTR")));

  // Line 4: State and error
  display.setCursor(x, LINE4_Y);
  display.print(F("State:"));
  display.print(stateToString(g_display.state));
  display.print(F(" Err:"));
  dtostrf(g_display.headingError, 5, 1, buf);
  display.print(buf);
}

// Draw Page 1 - Navigation
void drawPage1() {
  int16_t x = X_MARGIN;

  // Line 2: Title
  display.setCursor(x + 60, LINE2_Y);
  display.print(F("NAVIGATION"));

  // Line 3: LAT
  display.setCursor(x, LINE3_Y);
  display.print(F("LAT: "));
  if (g_display.gnssOK) {
    char buf[12];
    dtostrf(g_display.latitude, 9, 5, buf);
    display.print(buf);
  } else {
    display.print(F("---.-----"));
  }

  // Line 4: LON
  display.setCursor(x, LINE4_Y);
  display.print(F("LON: "));
  if (g_display.gnssOK) {
    char buf[12];
    dtostrf(g_display.longitude, 10, 5, buf);
    display.print(buf);
  } else {
    display.print(F("----.-----"));
  }

  // Line 5: SOG and COG
  display.setCursor(x, LINE5_Y);
  display.print(F("SOG:"));
  if (g_display.gnssOK) {
    char buf[8];
    dtostrf(g_display.sog, 4, 1, buf);
    display.print(buf);
  } else {
    display.print(F("--.-"));
  }
  display.print(F("kt COG:"));
  if (g_display.gnssOK) {
    char buf[8];
    dtostrf(g_display.cog, 5, 1, buf);
    display.print(buf);
  } else {
    display.print(F("---.-"));
  }
}

// Draw Page 2 - System
void drawPage2() {
  int16_t x = X_MARGIN;

  // Line 2: Title
  display.setCursor(x + 36, LINE2_Y);
  display.print(F("SYSTEM STATUS"));

  // Line 3: Master status
  display.setCursor(x, LINE3_Y);
  display.print(F("Master: "));
  display.print(g_display.masterOK ? F("CONNECTED") : F("OFFLINE"));

  // Line 4: Rudder status
  display.setCursor(x, LINE4_Y);
  display.print(F("Rudder: "));
  display.print(g_display.rudderOK ? F("CONNECTED") : F("OFFLINE"));

  // Line 5: State
  display.setCursor(x, LINE5_Y);
  display.print(F("State: "));
  display.print(stateToString(g_display.state));

  // Line 6: Version
  display.setCursor(x, LINE6_Y);
  display.print(F("Ver: "));
  display.print(APP_VERSION);
}

// Full screen refresh
void displayFullRefresh() {
  Serial.println(F("[EPD] Full refresh"));

  display.setFullWindow();
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    drawStatusBar();

    switch (g_display.currentPage) {
      case 0: drawPage0(); break;
      case 1: drawPage1(); break;
      case 2: drawPage2(); break;
    }
  } while (display.nextPage());

  g_display.partialCount = 0;
  g_prevDisplay = g_display;  // Save current state
}

// Partial update for heading value (Page 0, Line 2)
void partialUpdateHeading() {
  // HDG value position: after "HDG:" = 4 chars * 12 = 48px from margin
  int16_t x = X_MARGIN + 4 * CHAR_WIDTH;
  int16_t w = 5 * CHAR_WIDTH;  // "XXX.X" = 5 chars

  display.setPartialWindow(x, LINE2_Y, w, CHAR_HEIGHT);
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setCursor(x, LINE2_Y);
    char buf[10];
    dtostrf(g_display.heading, 5, 1, buf);
    display.print(buf);
  } while (display.nextPage());

  g_display.partialCount++;
  Serial.printf("[EPD] Partial HDG: %.1f\n", g_display.heading);
}

// Partial update for target value (Page 0, Line 2)
void partialUpdateTarget() {
  // TGT value position: after "HDG:XXX.X TGT:" = 14 chars
  int16_t x = X_MARGIN + 14 * CHAR_WIDTH;
  int16_t w = 5 * CHAR_WIDTH;

  display.setPartialWindow(x, LINE2_Y, w, CHAR_HEIGHT);
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setCursor(x, LINE2_Y);
    char buf[10];
    dtostrf(g_display.target, 5, 1, buf);
    display.print(buf);
  } while (display.nextPage());

  g_display.partialCount++;
  Serial.printf("[EPD] Partial TGT: %.1f\n", g_display.target);
}

// Partial update for rudder angle (Page 0, Line 3)
void partialUpdateRudder() {
  // RUD value: after "RUD:" = 4 chars
  int16_t x = X_MARGIN + 4 * CHAR_WIDTH;
  int16_t w = 10 * CHAR_WIDTH;  // "XX.X STBD" = ~10 chars

  display.setPartialWindow(x, LINE3_Y, w, CHAR_HEIGHT);
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setCursor(x, LINE3_Y);
    char buf[10];
    dtostrf(g_display.rudderAngle, 5, 1, buf);
    display.print(buf);
    display.print(g_display.rudderAngle < -0.5f ? F(" PORT") :
                  (g_display.rudderAngle > 0.5f ? F(" STBD") : F(" CNTR")));
  } while (display.nextPage());

  g_display.partialCount++;
  Serial.printf("[EPD] Partial RUD: %.1f\n", g_display.rudderAngle);
}

// Partial update for state (Page 0, Line 4)
void partialUpdateState() {
  // State value: after "State:" = 6 chars
  int16_t x = X_MARGIN + 6 * CHAR_WIDTH;
  int16_t w = 8 * CHAR_WIDTH;  // "ENGAGED" = 7 chars + margin

  display.setPartialWindow(x, LINE4_Y, w, CHAR_HEIGHT);
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setCursor(x, LINE4_Y);
    display.print(stateToString(g_display.state));
  } while (display.nextPage());

  g_display.partialCount++;
  Serial.printf("[EPD] Partial State: %s\n", stateToString(g_display.state));
}

// Partial update for SOG (Page 1, Line 5)
void partialUpdateSOG() {
  int16_t x = X_MARGIN + 4 * CHAR_WIDTH;  // After "SOG:"
  int16_t w = 4 * CHAR_WIDTH;

  display.setPartialWindow(x, LINE5_Y, w, CHAR_HEIGHT);
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setCursor(x, LINE5_Y);
    char buf[8];
    dtostrf(g_display.sog, 4, 1, buf);
    display.print(buf);
  } while (display.nextPage());

  g_display.partialCount++;
  Serial.printf("[EPD] Partial SOG: %.1f\n", g_display.sog);
}

// Partial update for COG (Page 1, Line 5)
void partialUpdateCOG() {
  int16_t x = X_MARGIN + 14 * CHAR_WIDTH;  // After "SOG:XX.Xkt COG:"
  int16_t w = 5 * CHAR_WIDTH;

  display.setPartialWindow(x, LINE5_Y, w, CHAR_HEIGHT);
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setCursor(x, LINE5_Y);
    char buf[8];
    dtostrf(g_display.cog, 5, 1, buf);
    display.print(buf);
  } while (display.nextPage());

  g_display.partialCount++;
  Serial.printf("[EPD] Partial COG: %.1f\n", g_display.cog);
}

// Check for changes and update display
void updateDisplay() {
  // Force full refresh after 5 partial updates (ghosting prevention per FSD 13.3)
  if (g_display.partialCount >= 5) {
    displayFullRefresh();
    return;
  }

  // Page change requires full refresh
  if (g_display.currentPage != g_prevDisplay.currentPage) {
    displayFullRefresh();
    return;
  }

  // Connectivity change requires full refresh
  if (g_display.masterOK != g_prevDisplay.masterOK ||
      g_display.rudderOK != g_prevDisplay.rudderOK ||
      g_display.gnssOK != g_prevDisplay.gnssOK) {
    displayFullRefresh();
    return;
  }

  // Page-specific partial updates
  if (g_display.currentPage == 0) {
    // Autopilot page
    if (abs(g_display.heading - g_prevDisplay.heading) > 0.05f) {
      partialUpdateHeading();
      g_prevDisplay.heading = g_display.heading;
    }
    if (abs(g_display.target - g_prevDisplay.target) > 0.05f) {
      partialUpdateTarget();
      g_prevDisplay.target = g_display.target;
    }
    if (abs(g_display.rudderAngle - g_prevDisplay.rudderAngle) > 0.05f) {
      partialUpdateRudder();
      g_prevDisplay.rudderAngle = g_display.rudderAngle;
    }
    if (g_display.state != g_prevDisplay.state) {
      partialUpdateState();
      g_prevDisplay.state = g_display.state;
    }
  }
  else if (g_display.currentPage == 1) {
    // Navigation page
    if (abs(g_display.sog - g_prevDisplay.sog) > 0.05f) {
      partialUpdateSOG();
      g_prevDisplay.sog = g_display.sog;
    }
    if (abs(g_display.cog - g_prevDisplay.cog) > 0.05f) {
      partialUpdateCOG();
      g_prevDisplay.cog = g_display.cog;
    }
    // LAT/LON partial updates could be added similarly
  }
}

// ======================= Button Functions =======================

void initButtons() {
  for (uint8_t i = 0; i < NUM_BUTTONS; ++i) {
    pinMode(BTN_PINS[i], INPUT);
    int raw = digitalRead(BTN_PINS[i]);
    g_btn[i].stableLevel = (raw != 0);
    g_btn[i].lastRawLevel = g_btn[i].stableLevel;
    g_btn[i].lastEdgeMs = millis();
    g_btn[i].pressStartMs = 0;
    g_btn[i].longReported = false;
  }
  Serial.println(F("[BTN] Buttons initialized"));
}

void handleButtonPress(uint8_t btn, bool longPress) {
  Serial.printf("[BTN] %s %s\n", BTN_NAMES[btn], longPress ? "LONG" : "SHORT");

  if (!longPress) {
    switch (btn) {
      case 0:  // -10
        g_display.target -= 10.0f;
        if (g_display.target < 0) g_display.target += 360.0f;
        break;
      case 1:  // -1
        g_display.target -= 1.0f;
        if (g_display.target < 0) g_display.target += 360.0f;
        break;
      case 2:  // ENGAGE
        g_display.state = (g_display.state == 2) ? 1 : 2;  // Toggle IDLE/ENGAGED
        break;
      case 3:  // MODE - cycle page
        g_display.currentPage = (g_display.currentPage + 1) % 3;
        break;
      case 4:  // +1
        g_display.target += 1.0f;
        if (g_display.target >= 360.0f) g_display.target -= 360.0f;
        break;
      case 5:  // +10
        g_display.target += 10.0f;
        if (g_display.target >= 360.0f) g_display.target -= 360.0f;
        break;
    }
  }
}

void pollButtons() {
  uint32_t now = millis();

  for (uint8_t i = 0; i < NUM_BUTTONS; ++i) {
    int raw = digitalRead(BTN_PINS[i]);
    bool rawLevel = (raw != 0);

    if (rawLevel != g_btn[i].lastRawLevel) {
      g_btn[i].lastRawLevel = rawLevel;
      g_btn[i].lastEdgeMs = now;
    } else {
      if (rawLevel != g_btn[i].stableLevel &&
          (now - g_btn[i].lastEdgeMs) >= BTN_DEBOUNCE_MS) {
        bool prev = g_btn[i].stableLevel;
        g_btn[i].stableLevel = rawLevel;

        if (prev == HIGH && rawLevel == LOW) {
          // Button pressed
          g_btn[i].pressStartMs = now;
          g_btn[i].longReported = false;
        } else if (prev == LOW && rawLevel == HIGH) {
          // Button released
          if (!g_btn[i].longReported) {
            handleButtonPress(i, false);
          }
        }
      }
    }

    // Long press detection
    if (g_btn[i].stableLevel == LOW && !g_btn[i].longReported) {
      if ((now - g_btn[i].pressStartMs) >= BTN_LONG_MS) {
        g_btn[i].longReported = true;
        handleButtonPress(i, true);
      }
    }
  }
}

// ======================= E-Paper Initialization =======================

bool initEPD() {
  Serial.println(F("[EPD] Initializing e-Paper..."));

  SPI.begin(PIN_EP_SCK, -1, PIN_EP_MOSI, PIN_EP_CS);
  display.init(115200);
  display.setRotation(1);
  display.setTextSize(2);
  display.setFont(NULL);
  display.setTextColor(GxEPD_BLACK);

  // Initial full refresh
  displayFullRefresh();

  Serial.println(F("[EPD] Init complete"));
  return true;
}

// ======================= Simulated Data Updates =======================
// For testing - simulate changing values

void simulateDataUpdates() {
  static uint32_t lastSim = 0;
  uint32_t now = millis();

  if (now - lastSim >= 1000) {
    lastSim = now;

    // Simulate heading drift
    g_display.heading += random(-10, 11) / 10.0f;
    if (g_display.heading < 0) g_display.heading += 360.0f;
    if (g_display.heading >= 360.0f) g_display.heading -= 360.0f;

    // Simulate rudder movement toward target
    g_display.headingError = g_display.target - g_display.heading;
    if (g_display.headingError > 180.0f) g_display.headingError -= 360.0f;
    if (g_display.headingError < -180.0f) g_display.headingError += 360.0f;

    g_display.rudderAngle = constrain(g_display.headingError * 0.5f, -35.0f, 35.0f);

    // Simulate SOG/COG
    g_display.sog = 2.5f + random(-5, 6) / 10.0f;
    g_display.cog = g_display.heading + random(-5, 6) / 10.0f;

    // Mark nodes as connected for testing
    g_display.masterOK = true;
    g_display.rudderOK = true;
  }
}

// ======================= Arduino Lifecycle =======================

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  delay(500);

  // WiFi OTA
  bcOtaConfig cfg = {
    .maintButtonPin = -1,
    .maintActiveLevel = HIGH,
    .wifiSsid = mySSID,
    .wifiPassword = myPASSWORD,
    .hostname = "UiNode",
    .mode = bcOtaMode::ALWAYS_ON
  };
  bcOta::begin(cfg);

  // Debug console
  bcDebugNetConfig dbgCfg = {
    .port = 2323,
    .mirrorToSerial = true
  };
  bcDebugNet::begin(dbgCfg);

  printBanner();

  initButtons();
  g_epdOK = initEPD();

  // Initialize test values
  g_display.heading = 245.0f;
  g_display.target = 245.0f;
  g_display.state = 1;  // IDLE
  g_prevDisplay = g_display;

  Serial.println(F("[BOOT] Setup complete"));
  bcDebugNet::println("[BOOT] UiNode with partial refresh online");
}

void loop() {
  bcOta::handle();
  bcDebugNet::handle();

  pollButtons();

  // Simulate data updates for testing
  simulateDataUpdates();

  // Update display with partial refresh
  static uint32_t lastDisplayUpdate = 0;
  uint32_t now = millis();
  if (now - lastDisplayUpdate >= 500) {  // 2 Hz display update per FSD 6.4
    lastDisplayUpdate = now;
    updateDisplay();
  }

  // Debug output
  static uint32_t lastDebug = 0;
  if (now - lastDebug >= 5000) {
    lastDebug = now;
    bcDebugNet::printf("[UiNode] HDG:%.1f TGT:%.1f RUD:%.1f State:%s Page:%d\r\n",
      g_display.heading, g_display.target, g_display.rudderAngle,
      stateToString(g_display.state), g_display.currentPage);
  }
}
