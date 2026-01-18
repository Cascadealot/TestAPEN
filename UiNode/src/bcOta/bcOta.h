// ================= bcOta.h (patched) ===================
#pragma once
#include <Arduino.h>

enum class bcOtaMode : uint8_t {
    BUTTON_ONLY,    // existing behavior (requires button press at boot)
    ALWAYS_ON,      // always enable OTA at boot (for development)
    EXTERNAL_FLAG   // OTA controlled by external flag (CAN/NVS) - future
};

struct bcOtaConfig {
    // Existing fields
    int maintButtonPin;      // -1 if unused
    int maintActiveLevel;    // HIGH or LOW
    const char* wifiSsid;
    const char* wifiPassword;
    const char* hostname;

    // NEW FIELD
    bcOtaMode mode = bcOtaMode::BUTTON_ONLY;
};

namespace bcOta {

bool begin(const bcOtaConfig& cfg);
void handle();

// NEW: allow external enable/disable (EXTERNAL_FLAG mode)
void setExternalEnable(bool enable);
bool getOtaEnabled();        // query current state

} // namespace bcOta
// =======================================================
