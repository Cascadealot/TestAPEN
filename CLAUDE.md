# TestAPEN Autopilot - Project Context

This file is automatically read at the start of each Claude Code session.

---

## Project Overview

**TestAPEN** is a fork of TestAP2 that replaces CAN bus communication with ESP-NOW wireless protocol. This enables boat testing while CAN hardware issues are resolved separately.

### Architecture

Three specialized nodes communicating via ESP-NOW (WiFi frames):

| Node | Function |
|------|----------|
| Master | Autopilot control, IMU (ICM-20948), Display (SSD1306), Network, BLE |
| Rudder | AS5600 encoder, Cytron MD13S motor driver, Servo control |
| UI | E-Paper display, physical buttons, status display |

### ESP-NOW vs CAN Comparison

| Aspect | TestAP2 (CAN) | TestAPEN (ESP-NOW) |
|--------|---------------|---------------------|
| Transport | CAN bus 500kbps | ESP-NOW WiFi frames |
| Latency | <10ms deterministic | 5-50ms variable |
| Wiring | CAN-H, CAN-L, GND | None (wireless) |
| Range | ~50m wired | 250m+ open air |
| Payload | 8 bytes fixed | Up to 250 bytes |

### Rudder Drive System (CRITICAL)

**Physical Setup:**
- AS5600 encoder magnet is on **motor drive shaft** (NOT the rudder)
- Motor uses **worm drive gearbox** (non-backdriveable)
- **2.25 motor shaft rotations** = full rudder travel (lock-to-lock)
- Rudder range: **±35 degrees** (70 degrees total)

**Manual Override:**
- Boat has foot pedal steering and continuous line rudder control
- Manual controls **always override** autopilot
- When user manually moves rudder, encoder position becomes stale
- **No safety risk** from autopilot - manual controls disconnect load via worm drive

**Re-centering Workflow:**
1. User manually positions rudder to actual center
2. User issues `cal center` command via console
3. System resets virtual position to zero
4. Encoder tracking now synchronized with actual rudder position

**Engage Behavior:**
- At ENGAGE: Use last known rudder position from previous session
- If no previous data (first boot): Assume rudder is CENTERED (0°)
- Worm drive holds position, so last known position remains valid
- User responsible for re-centering if rudder moved manually between sessions

### Platform

- **MCU**: ESP32-WROOM-32 (38-pin, all nodes)
- **Build System**: ESP-IDF v5.1+ (idf.py)
- **Framework**: FreeRTOS

---

## ESP-NOW Configuration

### MAC Addresses (Kconfig)

Each node has a hardcoded MAC address configured in Kconfig:
```
CONFIG_ESPNOW_MASTER_MAC="AA:BB:CC:DD:EE:01"
CONFIG_ESPNOW_RUDDER_MAC="AA:BB:CC:DD:EE:02"
CONFIG_ESPNOW_UI_MAC="AA:BB:CC:DD:EE:03"
```

### Discovering Node MACs

```bash
# On each node via telnet
status  # Shows WiFi MAC

# Or use esptool
esptool.py --port /dev/ttyUSB0 read_mac
```

---

## Key Files

| File | Purpose |
|------|---------|
| `docs/TestAPEN.FSD.v1.0.0.md` | **Authoritative specification** - Single source of truth |
| `agent/feature-list.json` | Feature tracking for LRA workflow |
| `agent/coding-prompt.md` | Development session instructions |
| `docs/claude-progress.txt` | Cross-session work log |

---

## Safety Rules (CRITICAL)

When working on autopilot code:

### NEVER
- Exceed rudder limits: **±35°**
- Bypass watchdog timers
- Allow motors without valid sensor feedback
- Remove existing safety checks

### ALWAYS
- Clamp all outputs to safe ranges
- Include timeout handling for sensors
- Log state transitions and faults
- Build and test before committing

---

## Directory Structure

```
TestAPEN/
├── agent/                    # LRA prompts and feature list
├── android/                  # Android control app
│   └── app/src/main/
│       ├── java/.../testap2/ # Kotlin source files
│       └── res/              # Layouts, colors, strings
├── docs/                     # FSD and progress tracking
├── components/
│   ├── as5600/              # AS5600 magnetic encoder driver
│   ├── autopilot_common/    # Shared constants, utilities
│   ├── ble_manager/         # BLE GATT service (NimBLE)
│   ├── cmd_console/         # Debug console commands
│   ├── epaper/              # E-Paper display driver (UI node)
│   ├── espnow_protocol/     # ESP-NOW message definitions
│   ├── gnss_driver/         # GNSS parsing
│   ├── icm20948/            # IMU driver
│   ├── network_manager/     # WiFi, OTA, debug console
│   ├── param_store/         # Runtime parameters with NVS
│   ├── ssd1306/             # OLED display driver
│   └── state_machine/       # System state machine
├── main/
│   ├── main.c               # Entry point
│   ├── master_node.c        # Master node implementation
│   ├── rudder_node.c        # Rudder node implementation
│   └── ui_node.c            # UI node implementation
├── sdkconfig                 # ESP-IDF configuration
└── CMakeLists.txt           # Build configuration
```

---

## Quick Commands

```bash
# Configure (first time or after Kconfig change)
idf.py menuconfig

# Build
idf.py build

# Flash via USB
idf.py -p /dev/ttyUSB0 flash

# Monitor serial output
idf.py -p /dev/ttyUSB0 monitor

# Build + Flash + Monitor
idf.py -p /dev/ttyUSB0 flash monitor

# Clean build
idf.py fullclean
```

### Node Selection (Kconfig)

The same codebase builds for any node. Select in menuconfig:
- `TestAPEN Configuration` → `Node Type` → `Master`, `Rudder`, or `UI`

---

## Firmware Flashing Policy (IMPORTANT)

### ALWAYS Prefer OTA Updates

**Use OTA whenever possible** - even when USB is available. This policy helps:
1. Expose and debug OTA-related issues early
2. Validate the OTA update path continuously
3. Allow debugging via Console (telnet:2323) during/after update
4. Simulate real-world deployment conditions

**OTA Command** (use raw binary, NOT multipart form-data):
```bash
curl -X POST --data-binary @build/testapen.bin \
  -H "Content-Type: application/octet-stream" \
  http://<ip>/update
```

**USB Flash ONLY when**:
- OTA is broken and needs fixing
- Partition table changes (requires full flash)
- First-time flash of new hardware
- Recovery from bricked firmware

---

## Session Checklist

Before ending any session:

- [ ] All modified code compiles (`idf.py build`)
- [ ] No new warnings introduced
- [ ] Feature marked complete (if finished)
- [ ] Progress file updated
- [ ] Changes committed

---

## Rollback Path

If ESP-NOW proves unsuitable for production:
1. Fix CAN hardware issues in TestAP2
2. Message structures are identical - easy to port back
3. TestAPEN remains available for wireless testing

---

## References

- **FSD**: `docs/TestAPEN.FSD.v1.0.0.md` (always authoritative)
- **TestAP2**: Original CAN-based project at `/home/cas/TestAP2/`

---

## CRITICAL USER DIRECTIVES (MUST FOLLOW)

### STOP MEANS STOP - IMMEDIATELY
When the user types "stop", "STOP", or any variation:
- STOP ALL ACTIONS IMMEDIATELY
- Do NOT finish current thought
- Do NOT complete current tool call
- Do NOT explain anything
- Just respond "Stopped." and WAIT

This has been emphasized THREE TIMES. No exceptions.

### NO FLASHING WITHOUT PERMISSION
NEVER flash firmware (USB or OTA) without the user's EXPRESS PERMISSION.
- Always ask first: "Ready to flash. Proceed?"
- Wait for explicit "yes" or confirmation
- Do NOT assume permission

### USB SERIAL FREEZES TERMINAL
Do NOT attempt to read data from ESP32 via USB serial (cat, monitor, etc.).
The terminal session freezes and cannot capture data.
Use OTA/HTTP methods instead.

---
