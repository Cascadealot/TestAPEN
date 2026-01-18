#pragma once
#include <Arduino.h>

struct bcDebugNetConfig {
    uint16_t port = 2323;       // Default TCP port for debugging
    bool mirrorToSerial = true; // Duplicate output to Serial?
};

namespace bcDebugNet {

void begin(const bcDebugNetConfig& cfg);
void handle();

// Write output
void print(const String& s);
void println(const String& s);
void printf(const char* fmt, ...);

// Client status
bool clientConnected();

} // namespace bcDebugNet
