// ================= bcOta.cpp (patched) ===================
#include "bcOta.h"
#include <WiFi.h>
#include <ArduinoOTA.h>

// Internal state
static bcOtaConfig g_cfg;
static bool g_otaMode = false;
static bool g_externalFlag = false;   // for EXTERNAL_FLAG mode

// Forward
static void startOtaServices();

bool bcOta::begin(const bcOtaConfig& cfg) {
    g_cfg = cfg;

    Serial.println("[bcOta] begin()");

    // ----------------------------
    // Mode: ALWAYS_ON
    // ----------------------------
    if (cfg.mode == bcOtaMode::ALWAYS_ON) {
        Serial.println("[bcOta] Mode: ALWAYS_ON — enabling OTA unconditionally.");
        g_otaMode = true;
        startOtaServices();
        return true;
    }

    // ----------------------------
    // Mode: EXTERNAL_FLAG
    // ----------------------------
    if (cfg.mode == bcOtaMode::EXTERNAL_FLAG) {
        Serial.println("[bcOta] Mode: EXTERNAL_FLAG — waiting for external enable flag.");
        g_otaMode = g_externalFlag;
        if (g_otaMode) {
            Serial.println("[bcOta] External flag is TRUE — enabling OTA.");
            startOtaServices();
        } else {
            Serial.println("[bcOta] OTA disabled until external flag is set.");
        }
        return true;
    }

    // ----------------------------
    // Mode: BUTTON_ONLY (legacy behavior)
    // ----------------------------
    Serial.println("[bcOta] Mode: BUTTON_ONLY");

    if (cfg.maintButtonPin >= 0) {
        pinMode(cfg.maintButtonPin, INPUT);
        delay(10);
        int level = digitalRead(cfg.maintButtonPin);

        if (level == cfg.maintActiveLevel) {
            Serial.println("[bcOta] Maintenance button held — enabling OTA.");
            g_otaMode = true;
            startOtaServices();
            return true;
        }
    }

    Serial.println("[bcOta] Normal mode (OTA disabled).");
    g_otaMode = false;
    return true;
}


// =======================================================
// Helper: start WiFi + ArduinoOTA
// =======================================================
static void startOtaServices() {
    Serial.println("[bcOta] Starting WiFi...");

    WiFi.mode(WIFI_STA);
    WiFi.begin(g_cfg.wifiSsid, g_cfg.wifiPassword);

    Serial.print("[bcOta] Connecting to WiFi");
    int tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries < 40) { // ~8 sec
        Serial.print(".");
        delay(200);
        tries++;
    }
    Serial.println();

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[bcOta] ERROR: WiFi failed to connect.");
        return;
    }

    Serial.printf("[bcOta] WiFi connected. IP: %s\n", WiFi.localIP().toString().c_str());

    ArduinoOTA.setHostname(g_cfg.hostname);
    ArduinoOTA.begin();

    Serial.println("[bcOta] OTA Ready.");
    Serial.printf("[bcOta] Hostname: %s\n", g_cfg.hostname);
}


// =======================================================
void bcOta::handle() {
    if (g_otaMode) {
        ArduinoOTA.handle();
    }
}


// =======================================================
// External enable/disable (for EXTERNAL_FLAG mode)
// =======================================================
void bcOta::setExternalEnable(bool enable) {
    g_externalFlag = enable;

    if (g_cfg.mode == bcOtaMode::EXTERNAL_FLAG) {
        if (enable && !g_otaMode) {
            Serial.println("[bcOta] External flag set TRUE — enabling OTA now.");
            g_otaMode = true;
            startOtaServices();
        }

        if (!enable && g_otaMode) {
            Serial.println("[bcOta] External flag set FALSE — OTA disabled.");
            g_otaMode = false;
            // We intentionally do NOT shut down WiFi or OTA server
            // to avoid half-disabling during a sketch upload.
        }
    }
}


// =======================================================
bool bcOta::getOtaEnabled() {
    return g_otaMode;
}
// =======================================================
