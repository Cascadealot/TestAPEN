# E-Paper Partial Refresh Test (EPartial)

Test sketch to debug and demonstrate partial screen updates on the SSD1680 e-Paper display used in the TestAPEN UI Node.

## Features

- Partial refresh testing with random numbers (0-360)
- WiFi connectivity
- OTA firmware updates via HTTP POST
- Telnet debug console with commands

## Hardware Setup

**ESP32 WROOM-32 + 2.9" e-Paper Display (SSD1680)**

| Signal | GPIO | Description |
|--------|------|-------------|
| CLK    | 18   | SPI Clock |
| DIN    | 23   | SPI MOSI |
| CS     | 17   | Chip Select |
| DC     | 19   | Data/Command |
| RST    | 27   | Reset |
| BUSY   | 3    | Busy Status |

## Network Configuration

Edit these lines in `EPartial.ino` for your network:

```cpp
const char* WIFI_SSID     = "Boathouse24";
const char* WIFI_PASSWORD = "Waterdog24!";
const char* HOSTNAME      = "epartial";
const int   CONSOLE_PORT  = 2323;
```

## How to Use

### Arduino IDE Setup

1. Install Arduino IDE 2.x
2. Add ESP32 board support:
   - File → Preferences → Additional Board Manager URLs:
   - `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
3. Tools → Board → Boards Manager → Search "esp32" → Install
4. Select: Tools → Board → ESP32 Dev Module
5. Set: Tools → Partition Scheme → "Minimal SPIFFS (1.9MB APP with OTA)"

### Upload and Test

1. Connect ESP32 via USB
2. Select correct port: Tools → Port
3. Click Upload
4. Open Serial Monitor (115200 baud)
5. Note the IP address displayed

### OTA Updates

After initial USB upload, update firmware over WiFi:

```bash
# Export compiled binary from Arduino IDE:
# Sketch → Export Compiled Binary

# Upload via OTA:
curl -X POST --data-binary @EPartial.ino.esp32.bin http://<IP>/update
```

### Telnet Console

Connect to the debug console:

```bash
telnet <IP> 2323
```

**Available Commands:**

| Command | Description |
|---------|-------------|
| `help` | Show available commands |
| `status` | Show device status |
| `random` | Display random number (partial refresh) |
| `show N` | Display specific number (0-360) |
| `full` | Force full refresh |
| `partial` | Reset partial mode counter |
| `auto` | Toggle auto-update on/off |
| `clear` | Clear display |
| `reboot` | Reboot device |

## What This Test Does

1. Connects to WiFi and starts HTTP/Telnet servers
2. Initializes the e-Paper display with full refresh
3. Every 3 seconds (auto mode), displays a new random number (0-360)
4. Uses **partial refresh** to update only the number region
5. After 5 partial refreshes, does a full refresh to clear ghosting

## Expected Behavior

- Initial display shows "0°" with a border
- Every 3 seconds, a new random number appears
- Partial updates should complete in ~400ms (vs ~3s for full)
- Minimal ghosting/artifacts around the number
- Every 5th update triggers a full refresh

## Debugging Partial Refresh

### Serial Output Example

```
========================================
E-Paper Partial Refresh Test
SSD1680 296x128 Display
With WiFi, OTA, and Console
========================================

Connecting to WiFi: Boathouse24
.....
WiFi connected!
IP Address: 192.168.1.186
HTTP server started on port 80
Telnet console on port 2323

=== Displaying: 247 degrees ===
Partial update region: (80,40) 96x24
Initializing partial mode...
Waiting for BUSY... done (387 ms)
Partial update complete (count=1)
```

### Common Issues

1. **Partial update shows no change**
   - Check that LUT is loaded (serial shows "Partial mode initialized")
   - Verify command 0x0F is used (not 0xF7)

2. **Severe ghosting**
   - Use `full` command to force full refresh
   - Reduce auto-update frequency

3. **BUSY timeout**
   - Check BUSY pin wiring (GPIO 3)
   - Verify display is powered

### Key Differences: Full vs Partial

| Aspect | Full Refresh | Partial Refresh |
|--------|--------------|-----------------|
| Command | 0xF7 | 0x0F |
| LUT | OTP (built-in) | Custom 159-byte |
| Time | ~3 seconds | ~400ms |
| Ghosting | None | Possible |

## Files

- `EPartial.ino` - Arduino sketch with WiFi, OTA, console
- `README.md` - This file
