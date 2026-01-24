// EPaperTest.ino
// Minimal e-Paper display test with partial refresh demo
// Based on UiNode.ino - e-Paper only

#include <Arduino.h>
#include <SPI.h>
#include <GxEPD2_BW.h>

// ---------- Version info ----------
static const char* APP_NAME = "EPaperTest";
static const char* APP_BUILD = __DATE__ " " __TIME__;

// Version digits (will be updated randomly)
static uint8_t verDigit[3] = {0, 0, 1};  // v0.0.1

// ======================= E-Paper Pin Mapping =======================
#define PIN_EP_CS    17
#define PIN_EP_DC    19
#define PIN_EP_RST   27
#define PIN_EP_BUSY  3
#define PIN_EP_SCK   18
#define PIN_EP_MOSI  23

// ======================= Display Object =======================
// GxEPD2 display object for 2.9" Ingcool panel
GxEPD2_BW<GxEPD2_290_T94_V2, GxEPD2_290_T94_V2::HEIGHT> display(
  GxEPD2_290_T94_V2(PIN_EP_CS, PIN_EP_DC, PIN_EP_RST, PIN_EP_BUSY));

// ======================= Layout constants =======================
// Display is 296x128, rotation(1) = landscape
// TextSize(2) = ~12x16 pixels per char
static const int16_t LINE_HEIGHT = 22;
static const int16_t X_MARGIN = 4;
static const int16_t Y_START = 8;

// Version line position (line 2, y = Y_START + LINE_HEIGHT)
static const int16_t VERSION_Y = Y_START + LINE_HEIGHT;
// Version text starts with "v" then 3 digits with dots: "vX.X.X"
// Character width at textSize(2) is 12 pixels
static const int16_t CHAR_WIDTH = 12;
static const int16_t CHAR_HEIGHT = 16;

// ======================= Helpers =======================

void printBanner() {
  Serial.println();
  Serial.println(F("========================================"));
  Serial.println(F("[EPaperTest] Partial Refresh Demo"));
  Serial.print(F("Build: "));
  Serial.println(APP_BUILD);
  Serial.println(F("========================================"));
  Serial.println();
}

void printPinout() {
  Serial.println(F("E-Paper Pin mapping:"));
  Serial.print(F("  CS   = ")); Serial.println(PIN_EP_CS);
  Serial.print(F("  DC   = ")); Serial.println(PIN_EP_DC);
  Serial.print(F("  RST  = ")); Serial.println(PIN_EP_RST);
  Serial.print(F("  BUSY = ")); Serial.println(PIN_EP_BUSY);
  Serial.print(F("  SCK  = ")); Serial.println(PIN_EP_SCK);
  Serial.print(F("  MOSI = ")); Serial.println(PIN_EP_MOSI);
  Serial.println();
}

// Draw the version string at line 2
void drawVersionLine() {
  int16_t x = X_MARGIN;
  int16_t y = VERSION_Y;

  display.setCursor(x, y);
  display.print(F("v"));
  display.print(verDigit[0]);
  display.print(F("."));
  display.print(verDigit[1]);
  display.print(F("."));
  display.print(verDigit[2]);
}

// Draw full screen (initial)
void drawFullScreen() {
  display.setFullWindow();
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);

    int16_t x = X_MARGIN;
    int16_t y = Y_START;

    // Line 1: Title
    display.setCursor(x, y);
    display.print(F("EPaper Test"));

    // Line 2: Version
    drawVersionLine();

    // Line 3
    y = Y_START + 2 * LINE_HEIGHT;
    display.setCursor(x, y);
    display.print(F("296x128 px"));

    // Line 4
    y = Y_START + 3 * LINE_HEIGHT;
    display.setCursor(x, y);
    display.print(F("Partial demo"));

    // Line 5
    y = Y_START + 4 * LINE_HEIGHT;
    display.setCursor(x, y);
    display.print(F("Watch line 2"));

  } while (display.nextPage());
}

// Partial update: just redraw the version number area
void updateVersionPartial() {
  // Calculate the bounding box for "vX.X.X" (7 characters)
  // x starts at X_MARGIN, width = 7 chars * 12 pixels = 84
  int16_t x = X_MARGIN;
  int16_t y = VERSION_Y;
  int16_t w = 7 * CHAR_WIDTH;  // "vX.X.X" = 7 chars including dots
  int16_t h = CHAR_HEIGHT;     // exact height, no padding to avoid overwriting line 3

  display.setPartialWindow(x, y, w, h);
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setCursor(x, y);
    display.print(F("v"));
    display.print(verDigit[0]);
    display.print(F("."));
    display.print(verDigit[1]);
    display.print(F("."));
    display.print(verDigit[2]);
  } while (display.nextPage());
}

// ---------- E-paper initialization ----------

bool initEPD() {
  Serial.println(F("[EPD] Initializing e-Paper..."));

  // Configure global SPI (VSPI) for the E-Paper panel pins
  Serial.println(F("[EPD] Configuring SPI..."));
  SPI.begin(PIN_EP_SCK, -1, PIN_EP_MOSI, PIN_EP_CS);

  // Initialize the display
  Serial.println(F("[EPD] Calling display.init()..."));
  display.init(115200);

  Serial.println(F("[EPD] Setting rotation and text properties..."));
  display.setRotation(1);
  display.setTextSize(2);
  display.setFont(NULL);
  display.setTextColor(GxEPD_BLACK);

  // Draw initial full screen
  Serial.println(F("[EPD] Drawing initial screen (full refresh)..."));
  drawFullScreen();

  Serial.println(F("[EPD] Init complete."));
  return true;
}

// ======================= Arduino lifecycle =======================

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  delay(500);

  printBanner();
  printPinout();

  // Seed random number generator
  randomSeed(analogRead(0) ^ micros());

  bool epdOK = initEPD();

  Serial.println();
  Serial.print(F("[STATUS] EPD = "));
  Serial.println(epdOK ? F("OK") : F("FAIL"));
  Serial.println();
  Serial.println(F("Starting partial refresh demo..."));
  Serial.println(F("One version digit will change each second."));
}

void loop() {
  static uint32_t lastUpdate = 0;
  uint32_t now = millis();

  if (now - lastUpdate >= 1000) {
    lastUpdate = now;

    // Pick a random digit position (0, 1, or 2)
    uint8_t pos = random(0, 3);

    // Pick a random new value (0-9)
    uint8_t newVal = random(0, 10);

    // Update that digit
    verDigit[pos] = newVal;

    Serial.print(F("[EPD] Partial update: v"));
    Serial.print(verDigit[0]);
    Serial.print(F("."));
    Serial.print(verDigit[1]);
    Serial.print(F("."));
    Serial.println(verDigit[2]);

    // Do partial screen update
    updateVersionPartial();
  }
}
