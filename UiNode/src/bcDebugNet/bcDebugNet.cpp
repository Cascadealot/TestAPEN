#include "bcDebugNet.h"
#include <WiFi.h>
#include <stdarg.h>

namespace bcDebugNet {

static WiFiServer server(2323);
static WiFiClient client;
static bool g_mirror = true;
static bool g_started = false;

// ======================================================
// Begin
// ======================================================
void begin(const bcDebugNetConfig& cfg) {
    if (g_started) return;

    g_mirror = cfg.mirrorToSerial;

    server = WiFiServer(cfg.port);
    server.begin();
    server.setNoDelay(true);

    g_started = true;

    if (g_mirror) {
        Serial.printf("[bcDebugNet] Listening on TCP port %u\n", cfg.port);
    }
}

// ======================================================
// Handle: Manage connection + cleanup
// ======================================================
void handle() {
    if (!g_started) return;

    // If no active client, accept a new one
    if (!client || !client.connected()) {
        if (client) client.stop();     // clean old
        client = server.available();   // check for new
        if (client) {
            if (g_mirror) Serial.println("[bcDebugNet] Client connected");
        }
    }

    // Drain client input (unused for now)
    if (client && client.connected()) {
        while (client.available()) {
            (void)client.read();
        }
    }
}

// ======================================================
// Internal helper
// ======================================================
static void sendToClient(const char* s) {
    if (client && client.connected()) {
        client.print(s);
    }
}

// ======================================================
// Print helpers
// ======================================================
void print(const String& s) {
    if (g_mirror) Serial.print(s);
    sendToClient(s.c_str());
}

void println(const String& s) {
    if (g_mirror) Serial.println(s);
    sendToClient((s + "\n").c_str());
}

void printf(const char* fmt, ...) {
    char buf[256];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (g_mirror) Serial.print(buf);
    sendToClient(buf);
}

// ======================================================
bool clientConnected() {
    return (client && client.connected());
}

} // namespace bcDebugNet
