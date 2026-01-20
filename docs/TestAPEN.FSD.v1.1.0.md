# TestAPEN Simple Autopilot

## Gold Standard Functional Specification Document (FSD)

**Project:** TestAPEN Autopilot (ESP-NOW wireless variant)
**Document Type:** Functional Specification (Single Source of Truth)
**FSD Version:** v1.1.0
**Status:** Authoritative
**Generated From:** FSD-GENERATION-PROMPT-v3.5

------

## Revision History

| Version | Date       | Author      | Description                    |
| ------- | ---------- | ----------- | ------------------------------ |
| v1.0.0  | 2026-01-13 | Claude Code | Initial release from v3.5 spec |
| v1.0.1  | 2026-01-19 | Claude Code | Renamed to TestAPEN, added Self Validating Agents (Sec 14) |
| v1.1.0  | 2026-01-20 | Claude Code | Major update: ESP-NOW protocol, UI Node, e-Paper display |

------

## Table of Contents

1. Introduction
2. System Overview
3. System Architecture
4. Hardware Specification
5. ESP-NOW Communication Protocol
6. Software Architecture
7. Control System
8. State Machine
9. Safety Protections
10. GNSS Subsystem
11. BLE Interface
12. Calibration
13. E-Paper Display System
14. Self Validating Agents
15. Acceptance Criteria
16. Appendices

------

## 1. Introduction

### 1.1 Purpose

This document specifies the complete functional requirements for the TestAPEN marine autopilot system. It serves as the single source of truth for all implementation decisions.

### 1.2 Scope

**In Scope:**
- Heading-hold autopilot control
- Tri-node ESP-NOW wireless architecture
- UI Node with e-Paper display and buttons
- Safety protections and fault handling
- Network debugging and OTA updates

**Non-Goals:**
- Waypoint/route navigation
- Wind angle steering
- AIS integration
- Multi-vessel coordination

### 1.3 Writing Conventions

| Directive | Meaning | Usage |
|-----------|---------|-------|
| SHALL | Mandatory requirement | Critical behaviors |
| MUST | Absolute requirement | Safety constraints |
| SHOULD | Recommended | Best practices |
| MAY | Optional | Flexibility points |

------

## 2. System Overview

### 2.1 Application Parameters

| Parameter | Value | Impact |
|-----------|-------|--------|
| Vessel | Lightweight canoe | Low inertia, quick response |
| Speed | ~3 knots typical | Reduced rudder authority |
| Behavior | Lively, responsive | High yaw rate sensitivity |
| Environment | Waves, wind | Continuous disturbances |

### 2.2 Design Philosophy

1. **Simplicity:** Heading-hold only, no waypoint navigation
2. **Modularity:** Tri-node ESP-NOW wireless architecture
3. **Safety:** Multiple independent protection layers
4. **Debuggability:** Network console and OTA updates on all nodes
5. **Evolvability:** Agent-assisted iterative development

### 2.3 Why ESP-NOW Instead of CAN

TestAPEN uses ESP-NOW wireless instead of CAN bus for:
- **Zero wiring:** No physical interconnects between nodes
- **Flexibility:** Nodes can be placed anywhere on vessel
- **Debugging:** CAN hardware issues eliminated
- **Migration path:** 8-byte message format identical to CAN for easy rollback

------

## 3. System Architecture

### 3.1 Node Topology

```
┌─────────────────┐                          ┌─────────────────┐
│   MASTER NODE   │◄─────── ESP-NOW ────────►│   RUDDER NODE   │
│                 │        (wireless)         │                 │
│  - IMU          │                          │  - AS5600       │
│  - GNSS         │                          │  - Motor Driver │
│  - OLED Display │                          │  - OLED Display │
│  - BLE          │                          │                 │
│  - WiFi/OTA     │                          │  - WiFi/OTA     │
└─────────────────┘                          └─────────────────┘
         │                                            │
         │              ESP-NOW                       │
         │             (wireless)                     │
         │                                            │
         ▼                                            ▼
┌─────────────────────────────────────────────────────────────┐
│                         UI NODE                              │
│                                                              │
│  - 2.9" e-Paper Display (SSD1680, 296x128)                  │
│  - 6 Physical Buttons (heading adjust, engage, mode)         │
│  - WiFi/OTA/Telnet Console                                  │
│  - Receives heartbeats from Master and Rudder               │
│  - Sends UI commands to Master                              │
└─────────────────────────────────────────────────────────────┘
```

### 3.2 Node Functions

| Node | Function | ESP-NOW Role |
|------|----------|--------------|
| Master | Autopilot control, IMU, GNSS, BLE | TX: Heartbeat, Commands |
| Rudder | AS5600 encoder, Motor driver | TX: Heartbeat, RX: Commands |
| UI | e-Paper display, Buttons | RX: Heartbeats, TX: Commands |

------

## 4. Hardware Specification

### 4.1 Master Node

```
ESP32 WROOM-32 (38-pin)
├── I2C Bus (SDA=21, SCL=22)
│   ├── ICM-20948 IMU @ 0x68
│   └── SSD1306 OLED @ 0x3C (128x64)
├── UART2 (RX=16, TX=17)
│   └── u-blox GNSS @ 9600→115200 baud
├── WiFi
│   └── Debug console + OTA
└── BLE
    └── User interface (Android app)
```

### 4.2 Rudder Node

```
ESP32 WROOM-32 (38-pin)
├── I2C Bus (SDA=21, SCL=22)
│   ├── AS5600 encoder @ 0x36 (mounted on motor shaft)
│   └── SSD1306 OLED @ 0x3C (128x32)
├── PWM+GPIO (PWM=25, DIR=26)
│   └── Cytron MD13S motor driver
└── Motor + Gearbox
    └── 12V DC motor with worm drive gearbox (non-backdriveable)
```

### 4.3 UI Node (NEW)

```
ESP32 WROOM-32 (38-pin)
├── SPI Bus (SPI3_HOST)
│   └── SSD1680 e-Paper @ 296x128
│       ├── CLK:  GPIO 18
│       ├── DIN:  GPIO 23 (MOSI)
│       ├── CS:   GPIO 5
│       ├── DC:   GPIO 17
│       ├── RST:  GPIO 16
│       └── BUSY: GPIO 4
├── Button GPIOs
│   ├── BTN_DEC10:  GPIO 36 (input-only)
│   ├── BTN_DEC1:   GPIO 39 (input-only)
│   ├── BTN_ENGAGE: GPIO 34 (input-only)
│   ├── BTN_MODE:   GPIO 35 (input-only)
│   ├── BTN_INC1:   GPIO 32 (pullup)
│   └── BTN_INC10:  GPIO 33 (pullup)
└── WiFi
    └── Debug console + OTA
```

### 4.4 Rudder Drive System

**Physical Configuration:**
- AS5600 magnetic encoder magnet mounted on **motor drive shaft** (not rudder)
- Worm drive gearbox: **non-backdriveable** (motor holds position when unpowered)
- Motor shaft turns **2.25 rotations** for full rudder travel (lock-to-lock)
- Rudder range: **±35 degrees** (70 degrees total)

**Manual Override:**
- Boat has foot pedal steering and continuous line rudder control
- Manual controls **always override** autopilot (worm drive disconnects load)
- When user manually moves rudder, motor shaft position becomes stale
- **No safety risk** from autopilot malfunction due to manual override capability

**Encoder Position Tracking:**
- AS5600 provides 12-bit single-turn reading (0-4095 counts = 0-360°)
- Firmware tracks wrap-around to build **multi-turn virtual position**
- Virtual position range: ±4608 counts (±1.125 motor turns = ±35° rudder)

### 4.5 GPIO Pin Assignments

**Master Node:**

| GPIO | Function | Direction | Notes |
|------|----------|-----------|-------|
| 16 | GNSS_RX | Input | UART2 |
| 17 | GNSS_TX | Output | UART2 |
| 21 | I2C_SDA | Bidirectional | Hardware I2C |
| 22 | I2C_SCL | Output | Hardware I2C |
| 2 | STATUS_LED | Output | Built-in LED |

**Rudder Node:**

| GPIO | Function | Direction | Notes |
|------|----------|-----------|-------|
| 21 | I2C_SDA | Bidirectional | Hardware I2C |
| 22 | I2C_SCL | Output | Hardware I2C |
| 25 | MOTOR_PWM | Output | LEDC channel 0 |
| 26 | MOTOR_DIR | Output | Direction control |
| 27 | MOTOR_EN | Output | Enable (optional) |
| 2 | STATUS_LED | Output | Built-in LED |

**UI Node:**

| GPIO | Function | Direction | Notes |
|------|----------|-----------|-------|
| 18 | EPAPER_CLK | Output | SPI clock |
| 23 | EPAPER_DIN | Output | SPI MOSI |
| 5 | EPAPER_CS | Output | Chip select |
| 17 | EPAPER_DC | Output | Data/Command |
| 16 | EPAPER_RST | Output | Reset |
| 4 | EPAPER_BUSY | Input | Status |
| 36 | BTN_DEC10 | Input | -10° button |
| 39 | BTN_DEC1 | Input | -1° button |
| 34 | BTN_ENGAGE | Input | Engage button |
| 35 | BTN_MODE | Input | Mode button |
| 32 | BTN_INC1 | Input | +1° button |
| 33 | BTN_INC10 | Input | +10° button |

------

## 5. ESP-NOW Communication Protocol

### 5.1 Physical Layer

| Parameter | Value |
|-----------|-------|
| Protocol | ESP-NOW (IEEE 802.11) |
| Range | 250m+ open air |
| Latency | 5-50ms typical |
| Payload | 8 bytes (matching CAN) |
| Addressing | MAC-based peer registration |

### 5.2 Node MAC Addresses (Kconfig)

Configured in menuconfig → ESP-NOW Configuration:

| Node | Config Key | Example |
|------|------------|---------|
| Master | CONFIG_ESPNOW_MASTER_MAC | 78:42:1C:6C:FA:58 |
| Rudder | CONFIG_ESPNOW_RUDDER_MAC | 78:42:1C:6D:28:94 |
| UI | CONFIG_ESPNOW_UI_MAC | 78:42:1C:6B:E5:F0 |

### 5.3 Message Frame Structure

```c
typedef struct __attribute__((packed)) {
    uint8_t msg_type;           // espnow_msg_type_t
    uint8_t data[8];            // Payload (same as CAN)
} espnow_frame_t;               // 9 bytes total
```

### 5.4 Message Type Assignments

| Type | Name | Direction | Rate |
|------|------|-----------|------|
| 0x01 | MSG_MASTER_HEARTBEAT | Master → All | 10 Hz |
| 0x02 | MSG_RUDDER_HEARTBEAT | Rudder → All | 50 Hz |
| 0x10 | MSG_RUDDER_COMMAND | Master → Rudder | 10 Hz |
| 0x11 | MSG_SYSTEM_COMMAND | Any → All | On demand |
| 0x12 | MSG_CALIBRATION_CMD | Master → Rudder | On demand |
| 0x13 | MSG_UI_COMMAND | UI → Master | On demand |
| 0x14 | MSG_PARAM_CONFIG | Master → Rudder | On demand |
| 0x20 | MSG_GNSS_POSITION | Master → All | 1 Hz |
| 0x21 | MSG_GNSS_VELOCITY | Master → All | 1 Hz |
| 0xE0 | MSG_E_STOP | Any → All | Emergency |

### 5.5 Message Structures

**1. Master Heartbeat (8 bytes)**

| Byte | Field | Type |
|------|-------|------|
| 0 | state | uint8 |
| 1 | fault_code | uint8 |
| 2-3 | heading×10 | int16 BE |
| 4-5 | target×10 | int16 BE |
| 6 | sequence | uint8 |
| 7 | flags | uint8 |

Flags: [0]=GNSS_VALID, [1]=COG_MODE, [2]=LAZY_HELM, [3]=SEA_AUTO, [4]=CALIBRATED

**2. Rudder Heartbeat (8 bytes)**

| Byte | Field | Type |
|------|-------|------|
| 0 | state | uint8 |
| 1 | fault_code | uint8 |
| 2-3 | angle×10 | int16 BE |
| 4 | motor_status | uint8 |
| 5 | sequence | uint8 |
| 6-7 | reserved | - |

Motor status: [0]=ENABLED, [1]=RUNNING, [2]=DIRECTION, [3]=IN_DEADBAND, [4]=AT_LIMIT

**3. Rudder Command (8 bytes)**

| Byte | Field | Type |
|------|-------|------|
| 0-1 | cmd_angle×10 | int16 BE |
| 2 | flags | uint8 |
| 3 | sequence | uint8 |
| 4-7 | reserved | - |

**4. UI Command (8 bytes)**

| Byte | Field | Type |
|------|-------|------|
| 0 | command | uint8 |
| 1-2 | value×10 | int16 BE |
| 3 | sequence | uint8 |
| 4-7 | reserved | - |

UI Commands: 0x01=ENGAGE, 0x02=DISENGAGE, 0x10=HEADING_ADJUST, 0x11=HEADING_SET

**5. System Command (8 bytes)**

| Byte | Field | Type |
|------|-------|------|
| 0 | command | uint8 |
| 1-7 | reserved | - |

Commands: 0x01=ENGAGE, 0x02=DISENGAGE, 0x10=CAL_ENTER, 0x11=CAL_EXIT, 0x20=FAULT_CLEAR

### 5.6 Error Code Table

| Code | Name | Severity |
|------|------|----------|
| 0x00 | NONE | - |
| 0x01 | ESPNOW_TX_FAIL | Warning |
| 0x02 | ESPNOW_RX_TIMEOUT | Fault |
| 0x03 | ESPNOW_INIT_FAIL | Critical |
| 0x10 | SENSOR_FAULT | Fault |
| 0x11 | SENSOR_RANGE | Warning |
| 0x20 | MOTOR_STALL | Fault |
| 0x22 | MOTOR_TIMEOUT | Fault |
| 0x40 | HEARTBEAT_LOST | Fault |

### 5.7 Timing Requirements

| Parameter | Value |
|-----------|-------|
| Master heartbeat | 100 ms |
| Rudder heartbeat | 20 ms |
| Command rate | 100 ms |
| Heartbeat timeout | 500 ms |
| E_STOP response | 10 ms max |

------

## 6. Software Architecture

### 6.1 Framework: ESP-IDF v5.1+

| Function | ESP-IDF Module |
|----------|----------------|
| RTOS | FreeRTOS |
| I2C | driver/i2c.h |
| SPI | driver/spi_master.h |
| UART | driver/uart.h |
| GPIO | driver/gpio.h |
| PWM | driver/ledc.h |
| Storage | nvs_flash.h |
| WiFi | esp_wifi.h |
| ESP-NOW | esp_now.h |
| OTA | esp_ota_ops.h |
| BLE | nimble/nimble_port.h |

### 6.2 Master Node Tasks

| Task | Pri | Rate | Stack |
|------|-----|------|-------|
| Task_ESPNOW | 5 | Event | 4096 |
| Task_IMU | 4 | 50Hz | 4096 |
| Task_GNSS | 4 | Event | 4096 |
| Task_Autopilot | 3 | 10Hz | 2048 |
| Task_Display | 2 | 5Hz | 4096 |
| Task_BLE | 1 | Event | 4096 |

### 6.3 Rudder Node Tasks

| Task | Pri | Rate | Stack |
|------|-----|------|-------|
| Task_ESPNOW | 5 | Event | 4096 |
| Task_Rudder | 4 | 50Hz | 4096 |
| Task_Network | 1 | 10Hz | 4096 |

### 6.4 UI Node Tasks

| Task | Pri | Rate | Stack |
|------|-----|------|-------|
| Task_ESPNOW | 5 | 50Hz | 4096 |
| Task_Buttons | 4 | 50Hz | 4096 |
| Task_Display | 2 | 2Hz | 4096 |
| Task_Network | 1 | 10Hz | 8192 |

### 6.5 Critical Rule: I2C Mutex

ALL I2C transactions MUST acquire global mutex. Violation = system defect.

------

## 7. Control System

### 7.1 Control Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    MASTER NODE (10 Hz)                       │
│  ┌─────────┐   ┌──────────┐   ┌─────────┐   ┌─────────────┐│
│  │ IMU     │──→│ Adaptive │──→│   PID   │──→│ ESP-NOW TX  ││
│  │ Heading │   │ Filter   │   │ Control │   │ Rudder Cmd  ││
│  └─────────┘   └──────────┘   └─────────┘   └─────────────┘│
└─────────────────────────────────────────────────────────────┘
                              │ ESP-NOW
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                   RUDDER NODE (50 Hz)                        │
│  ┌─────────┐   ┌──────────┐   ┌─────────┐   ┌─────────────┐│
│  │ ESP-NOW │──→│  Servo   │──→│  Motor  │──→│ Actuator    ││
│  │ RX Cmd  │   │ Control  │   │  PWM    │   │             ││
│  └─────────┘   └──────────┘   └─────────┘   └─────────────┘│
│       ▲                                            │        │
│       │           ┌─────────┐                      │        │
│       └───────────│ AS5600  │◀─────────────────────┘        │
│                   │ Encoder │   Feedback                    │
│                   └─────────┘                               │
└─────────────────────────────────────────────────────────────┘
```

### 7.2 PID Heading Control

Default parameters:
- Kp_heading = 0.8 (°/°)
- Ki_heading = 0.05 (°/(°·s))
- Kd_heading = 0.5 (°/(°/s))
- INTEGRAL_LIMIT = 5.0°
- RUDDER_CMD_MAX = 35.0°

### 7.3 Servo Control

Default parameters:
- Kp_servo = 10.0 (%/°)
- DEADBAND_ENTER = 1.0°
- DEADBAND_EXIT = 1.5°
- RUDDER_SLEW_RATE = 15.0°/s
- MIN_MOTOR_SPEED = 20%
- MAX_MOTOR_SPEED = 100%

------

## 8. State Machine

### 8.1 States

| Value | State | Motor |
|-------|-------|-------|
| 0x00 | BOOT | Off |
| 0x01 | IDLE | Off |
| 0x02 | ENGAGED | On |
| 0x03 | CALIBRATION | Off |
| 0xFF | FAULTED | Off |

### 8.2 Valid Transitions

```
BOOT ──(POST pass)──→ IDLE
BOOT ──(POST fail)──→ FAULTED
IDLE ──(ENGAGE+precond)──→ ENGAGED
IDLE ──(CAL_ENTER)──→ CALIBRATION
IDLE ──(fault)──→ FAULTED
ENGAGED ──(DISENGAGE)──→ IDLE
ENGAGED ──(fault)──→ FAULTED
CALIBRATION ──(CAL_EXIT)──→ IDLE
CALIBRATION ──(fault)──→ FAULTED
FAULTED ──(CLEAR+cond)──→ IDLE
```

### 8.3 Engage Preconditions

ALL required:
1. Valid heading (not NaN, < 500ms old)
2. Valid rudder feedback (heartbeat OK)
3. Calibration valid (range ≥ 5°)
4. No active faults

------

## 9. Safety Protections

### 9.1 Protection Thresholds

| Protection | Threshold | Action |
|------------|-----------|--------|
| Heartbeat timeout | 500 ms | FAULTED |
| Motor stall | < 0.5° in 500ms | FAULTED (0x20) |
| Drive timeout | 5000 ms | FAULTED (0x22) |
| Course deviation warn | 15° for 5s | Log |
| Course deviation alarm | 30° for 5s | Alarm |

### 9.2 Command Fallback (Rudder Node)

| Time since last cmd | Action |
|---------------------|--------|
| 0-200 ms | Hold |
| 200-500 ms | Ramp to center |
| > 500 ms | Stop, FAULTED |

------

## 10. GNSS Subsystem

### 10.1 Module Specifications

| Parameter | Value |
|-----------|-------|
| Module | u-blox MIA-M10Q |
| Protocol | UBX-NAV-PVT |
| Default Baud | 9600 |
| Configured Baud | 115200 |
| Update Rate | 10 Hz |

### 10.2 COG Validity

- COG valid when: speed > 1.5 kts AND accuracy < 10°
- Blend with compass: 1.5-3.0 kts
- Full COG: > 3.0 kts

**IMPORTANT:** GNSS faults do NOT cause system FAULTED. Fallback to compass.

------

## 11. BLE Interface

### 11.1 Service Configuration

| Service UUID | 12345678-1234-1234-1234-123456789abc |
|--------------|--------------------------------------|

| Characteristic | UUID suffix | Properties |
|----------------|-------------|------------|
| Command | ...9001 | Write |
| Status | ...9002 | Read, Notify |
| Heading | ...9003 | Read, Notify |
| Rudder | ...9004 | Read, Notify |
| Parameters | ...9005 | Read, Write, Notify |
| Mag Calibration | ...9006 | Read, Write, Notify |

### 11.2 BLE Commands

| Code | Command | Description |
|------|---------|-------------|
| 0x01 | ENGAGE | Engage autopilot |
| 0x02 | DISENGAGE | Disengage autopilot |
| 0x03 | SET_HEADING | Set target heading |
| 0x04 | ADJUST_HEADING | Adjust heading ±127° |
| 0x10-0x15 | CAL_* | Calibration commands |
| 0x50-0x53 | PARAM_* | Parameter commands |

------

## 12. Calibration

### 12.1 Rudder Calibration Procedure

1. `cal enter` - Enter CALIBRATION state
2. Move rudder to center → `cal center`
3. Move to port limit → `cal port`
4. Move to starboard limit → `cal stbd`
5. `cal save` - Save to NVS
6. `cal exit` - Return to IDLE

### 12.2 Magnetometer Calibration

1. `magcal start` - Begin guided calibration
2. Rotate device in figure-8 pattern
3. `magcal stop` - Calculate offsets
4. `magcal save` - Save to NVS

------

## 13. E-Paper Display System

### 13.1 Hardware Specification

| Parameter | Value |
|-----------|-------|
| Display | 2.9" e-Paper (SSD1680) |
| Resolution | 296 x 128 pixels |
| Colors | Black/White |
| Interface | SPI (4 MHz) |
| Full Refresh | ~3 seconds |
| Partial Refresh | ~400ms |

### 13.2 Display Pages

**Page 0 - Autopilot (Primary)**
```
M:OK R:OK G:-- Pg1
HDG:245.0 TGT:245.
RUD: -1.0 PORT
State: IDLE Err:+0.1
```

**Page 1 - Navigation**
```
M:OK R:OK G:-- Pg2
    NAVIGATION
LAT: (data pending)
LON: (data pending)
SOG: -- kts
COG: -- deg
```

**Page 2 - System**
```
M:OK R:OK G:-- Pg3
=== SYSTEM STATUS ===
Master: CONNECTED HB: 20ms
State: IDLE Fault: 0
Rudder: CONNECTED HB: 12ms
Angle: -1.0 deg
IP: 192.168.1.186
Version: 1.1.0-espnow
```

### 13.3 Partial Refresh Strategy

- Value changes (heading, angle): Use partial refresh (~400ms)
- Page changes: Use full refresh (clear ghosting)
- After 5 partial refreshes: Force full refresh (ghosting prevention)
- Connectivity changes: Full refresh

### 13.4 Button Functions

| Button | GPIO | Short Press | Long Press |
|--------|------|-------------|------------|
| -10 | 36 | Target -10° | Repeat at 5Hz |
| -1 | 39 | Target -1° | Repeat at 5Hz |
| ENGAGE | 34 | Toggle Engage | None |
| MODE | 35 | Cycle Page | None |
| +1 | 32 | Target +1° | Repeat at 5Hz |
| +10 | 33 | Target +10° | Repeat at 5Hz |

------

## 14. Self Validating Agents

### 14.1 Heartbeat Monitoring

| Node | Transmits | Monitors |
|------|-----------|----------|
| Master | Heartbeat @ 10Hz | Rudder heartbeat |
| Rudder | Heartbeat @ 50Hz | Master heartbeat |
| UI | None | Master + Rudder heartbeats |

### 14.2 Console Commands for Diagnostics

**All Nodes:**
```
help, status, state, version, reboot
espnow    - ESP-NOW statistics (TX/RX counts, peer status)
```

**Master Node:**
```
heading, imu, engage, disengage, set heading N, adjust N
heading sim N, heading real
magcal start/stop/status/save
fusion on/off/beta N
pid [Kp Ki Kd]
```

**Rudder Node:**
```
rudder, motor, engage, disengage
servo [Kp deadband_in deadband_out]
cal enter/exit/center/port/stbd/save
motor params <min> <max> <slew>
```

**UI Node:**
```
display, display refresh
page [0-2]
nodes
buttons
btn list, btn N, btn N long   (button simulation)
```

### 14.3 Button Simulation for Automated Testing

```bash
# Test engage/disengage cycle
(echo "btn 2"; sleep 2) | nc -q 3 192.168.1.186 2323
(echo "status") | nc -q 2 192.168.1.186 2323
```

------

## 15. Acceptance Criteria

| # | Criterion | Test Method |
|---|-----------|-------------|
| AC-1 | System boots to IDLE within 10 seconds | Console: `state` |
| AC-2 | Autopilot engages only when preconditions met | Console: `engage` |
| AC-3 | Heading error < 3° RMS in CALM | Console: `perf` |
| AC-4 | Rudder responds within 200ms | Console: `heading sim` |
| AC-5 | Stall detection triggers within 600ms | Block rudder |
| AC-6 | Heartbeat loss triggers FAULTED within 500ms | Power off node |
| AC-7 | DISENGAGE stops motor within 100ms | Console: `disengage` |
| AC-8 | ESP-NOW communication works | Console: `espnow` |
| AC-9 | UI buttons adjust heading | Console: `btn 4` |
| AC-10 | OTA update completes on all nodes | curl POST |
| AC-11 | E-Paper display shows values | Camera verify |
| AC-12 | Partial refresh works | Camera verify animate |

------

## Appendix A: ESP-NOW Quick Reference

| Message | Type | Direction | Rate |
|---------|------|-----------|------|
| Master Heartbeat | 0x01 | M→All | 10 Hz |
| Rudder Heartbeat | 0x02 | R→All | 50 Hz |
| Rudder Command | 0x10 | M→R | 10 Hz |
| System Command | 0x11 | Any→All | Event |
| Calibration Cmd | 0x12 | M→R | Event |
| UI Command | 0x13 | UI→M | Event |
| Param Config | 0x14 | M→R | Event |
| GNSS Position | 0x20 | M→All | 1 Hz |
| GNSS Velocity | 0x21 | M→All | 1 Hz |
| E_STOP | 0xE0 | Any→All | Event |

------

## Appendix B: Node Network Configuration

| Node | IP Address | Telnet Port | OTA Endpoint |
|------|------------|-------------|--------------|
| Master | 192.168.1.118 | 2323 | /update |
| Rudder | 192.168.1.157 | 2323 | /update |
| UI | 192.168.1.186 | 2323 | /update |

**OTA Command:**
```bash
curl -X POST --data-binary @build/testap2.bin \
  -H "Content-Type: application/octet-stream" http://<ip>/update
```

------

## Appendix C: Default Parameters

### C.1 Control (Runtime Configurable)
| Parameter | Default | Range |
|-----------|---------|-------|
| Kp_heading | 0.8 | 0.1-5.0 |
| Ki_heading | 0.05 | 0.0-1.0 |
| Kd_heading | 0.5 | 0.0-5.0 |
| Kp_servo | 10.0 | 1.0-50.0 |
| DEADBAND_ENTER | 1.0° | 0.1-5.0 |
| DEADBAND_EXIT | 1.5° | 0.2-6.0 |
| MIN_MOTOR_SPEED | 20% | 0-50 |
| MAX_MOTOR_SPEED | 100% | 50-100 |
| RUDDER_SLEW_RATE | 15.0°/s | 1.0-60.0 |

### C.2 Timing
- HEARTBEAT_TIMEOUT_MS = 500
- COMMAND_TIMEOUT_MS = 200
- STALL_TIMEOUT_MS = 500
- DRIVE_TIMEOUT_MS = 5000

------

**End of FSD v1.1.0**
