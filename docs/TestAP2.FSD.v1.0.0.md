# TestAP2 Simple Autopilot

## Gold Standard Functional Specification Document (FSD)

**Project:** TestAP2 Autopilot
**Document Type:** Functional Specification (Single Source of Truth)
**FSD Version:** v1.0.0
**Status:** Authoritative
**Generated From:** FSD-GENERATION-PROMPT-v3.5

------

## Revision History

| Version | Date       | Author      | Description                    |
| ------- | ---------- | ----------- | ------------------------------ |
| v1.0.0  | 2026-01-13 | Claude Code | Initial release from v3.5 spec |

------

## Table of Contents

1. Introduction
2. System Overview
3. Operating Environment
4. Hardware Specification
5. CAN Bus Protocol
6. Software Architecture
7. Control System
8. State Machine
9. Safety Protections
10. GNSS Subsystem
11. BLE Interface
12. Network Debugging and OTA
13. Calibration
14. Acceptance Criteria
15. Appendices

------

## 1. Introduction

### 1.1 Purpose

This document specifies the complete functional requirements for the TestAP2 marine autopilot system. It serves as the single source of truth for all implementation decisions.

### 1.2 Scope

**In Scope:**
- Heading-hold autopilot control
- Dual-node CAN architecture
- Safety protections and fault handling
- Network debugging and OTA updates

**Non-Goals:**
- Waypoint/route navigation
- Wind angle steering
- AIS integration
- Multi-vessel coordination
- Radar interface
- Anchor watch
- Route planning

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
2. **Modularity:** Dual-node CAN architecture
3. **Safety:** Multiple independent protection layers
4. **Debuggability:** Network console and OTA updates
5. **Evolvability:** Agent-assisted iterative development

### 2.3 Operating Environment

| Parameter | Value | Notes |
|-----------|-------|-------|
| Temperature Range | -10°C to +50°C | Marine environment |
| Humidity | Up to 95% RH non-condensing | Splash resistant |
| Enclosure Rating | IP65 minimum | Water jets protected |
| Vibration | Marine vessel typical | Secure mounting required |

**Note:** All electronic components MUST be rated for these conditions.

------

## 3. System Architecture

### 3.1 Node Topology

```
┌─────────────────┐         CAN Bus         ┌─────────────────┐
│   MASTER NODE   │◄───────────────────────►│   RUDDER NODE   │
│                 │       500 kbps          │                 │
│  - IMU          │       29-bit ID         │  - AS5600       │
│  - GNSS         │       120Ω termination  │  - Motor Driver │
│  - Display      │                         │                 │
│  - BLE          │                         │                 │
│  - WiFi/OTA     │                         │                 │
└─────────────────┘                         └─────────────────┘
```

------

## 4. Hardware Specification

### 4.1 Master Node

```
ESP32 WROOM-32 (38-pin)
├── I2C Bus (SDA=21, SCL=22)
│   ├── ICM-20948 IMU @ 0x68
│   └── SSD1306 OLED @ 0x3C
├── UART2 (RX=16, TX=17)
│   └── u-blox GNSS @ 115200 baud
├── CAN (RX=4, TX=5)
│   └── SN65HVD230 transceiver
├── WiFi
│   └── Debug console + OTA
└── BLE
    └── User interface
```

### 4.2 Rudder Node

```
ESP32 WROOM-32 (38-pin)
├── I2C Bus (SDA=21, SCL=22)
│   └── AS5600 encoder @ 0x36 (mounted on motor shaft)
├── PWM+GPIO (PWM=25, DIR=26)
│   └── Cytron MD13S motor driver
├── CAN (RX=4, TX=5)
│   └── SN65HVD230 transceiver
└── Motor + Gearbox
    └── 12V DC motor with worm drive gearbox (non-backdriveable)
```

### 4.2.1 Rudder Drive System

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
- Constants:
  - MOTOR_TURNS_LOCK_TO_LOCK = 2.25
  - COUNTS_CENTER_TO_LIMIT = 4608 (1.125 turns × 4096 counts)

**Re-centering Requirement:**
- After manual rudder movement, encoder position no longer matches actual rudder
- User must manually position rudder to center, then issue `cal center` command
- System resets virtual position to zero at current encoder reading
- This synchronizes encoder tracking with actual rudder position

**Engage Behavior:**
- At ENGAGE: Use last known rudder position from previous engage session
- If no previous data (first boot): Assume rudder is CENTERED (0 degrees)
- Worm drive holds position, so last known position remains valid
- User responsible for re-centering if rudder moved manually between sessions

### 4.3 GPIO Pin Assignments

**Master Node:**

| GPIO | Function | Direction | Notes |
|------|----------|-----------|-------|
| 4 | CAN_RX | Input | TWAI peripheral |
| 5 | CAN_TX | Output | TWAI peripheral |
| 16 | GNSS_RX | Input | UART2 |
| 17 | GNSS_TX | Output | UART2 |
| 21 | I2C_SDA | Bidirectional | Hardware I2C |
| 22 | I2C_SCL | Output | Hardware I2C |
| 2 | STATUS_LED | Output | Built-in LED |

**Rudder Node:**

| GPIO | Function | Direction | Notes |
|------|----------|-----------|-------|
| 4 | CAN_RX | Input | TWAI peripheral |
| 5 | CAN_TX | Output | TWAI peripheral |
| 21 | I2C_SDA | Bidirectional | Hardware I2C |
| 22 | I2C_SCL | Output | Hardware I2C |
| 25 | MOTOR_PWM | Output | LEDC channel 0 |
| 26 | MOTOR_DIR | Output | Direction control |
| 27 | MOTOR_EN | Output | Enable (optional) |
| 2 | STATUS_LED | Output | Built-in LED |

### 4.4 Power System

| Component | Spec |
|-----------|------|
| Battery | 8S1P LiFePO4, 25.6V, 280Ah |
| Master fuse | 10A blade |
| Rudder fuse | 15A blade |
| Regulation | 25.6V → 5V buck → 3.3V LDO |
| Protection | Reverse polarity (Schottky/P-FET) |

### 4.5 Power Protection Requirements

| Protection | Implementation | Specification |
|------------|----------------|---------------|
| Main Fuse | ANL or MIDI fuse | 30A, at battery |
| Master Node Fuse | ATO/ATC blade | 10A |
| Rudder Node Fuse | ATO/ATC blade | 15A |
| Reverse Polarity | Schottky or P-FET | >30A capability |
| Undervoltage Lockout | Firmware | Warn 22V, disable 20V |

### 4.6 Voltage Monitoring

| Condition | Threshold | Action |
|-----------|-----------|--------|
| Low voltage warning | V < 22.0V | Set flag, display warning |
| Low voltage fault | V < 20.0V | Disengage, enter FAULTED |
| High voltage warning | V > 29.0V | Set flag, log warning |

### 4.7 Motor Driver Limits

| Parameter | Value | Notes |
|-----------|-------|-------|
| Input voltage | 6-30V DC | Cytron MD13S |
| Continuous current | 13A | Per motor |
| Peak current | 30A | Short duration |
| PWM frequency | 20 kHz maximum | Higher may cause heating |

### 4.8 PWM Configuration

| Parameter | Value | Notes |
|-----------|-------|-------|
| Frequency | 20 kHz | Above audible range |
| Resolution | 8 bits (0-255) | PWM duty cycle |
| DIR LOW | Port | Motor direction |
| DIR HIGH | Starboard | Motor direction |

------

## 5. CAN Bus Protocol

### 5.1 Physical Layer

| Parameter | Value |
|-----------|-------|
| Bit rate | 500 kbps |
| ID format | 29-bit extended (CAN 2.0B) |
| Termination | 120Ω at each end |
| Cable | Twisted pair, shielded |

### 5.2 Message ID Formula

```c
// ID = (Priority << 26) | (Source << 22) | (Type << 18) | Code
#define CAN_MAKE_ID(pri, src, type, code) \
    (((uint32_t)(pri) << 26) | ((uint32_t)(src) << 22) | \
     ((uint32_t)(type) << 18) | ((uint32_t)(code) & 0x3FFFF))
```

### 5.3 Node ID Assignments

| ID | Node | Purpose |
|----|------|---------|
| 0 | Broadcast | System-wide |
| 1 | Master | Control hub |
| 2 | Rudder | Servo drive |
| 3-6 | Reserved | GPS, Wind, AIS, Battery |

### 5.4 Message Type Assignments

| Type | Name |
|------|------|
| 0 | Heartbeat |
| 1 | Command |
| 2 | Status |
| 5 | Error |
| 6 | Param Config |
| 7 | Calibration |

### 5.5 Complete Message Catalog

**1. Master Heartbeat (0x10400001)**
- Priority: 4, Source: 1, Type: 0, Code: 1
- Rate: 10 Hz

| Byte | Field | Type |
|------|-------|------|
| 0 | state | uint8 |
| 1 | fault_code | uint8 |
| 2-3 | heading×10 | int16 BE |
| 4-5 | target×10 | int16 BE |
| 6 | sequence | uint8 |
| 7 | flags | uint8 |

Flags: [0]=GNSS_VALID, [1]=COG_MODE, [2]=LAZY_HELM, [3]=SEA_AUTO, [4]=CALIBRATED

**2. Rudder Heartbeat (0x10800001)**
- Priority: 4, Source: 2, Type: 0, Code: 1
- Rate: 50 Hz

| Byte | Field | Type |
|------|-------|------|
| 0 | state | uint8 |
| 1 | fault_code | uint8 |
| 2-3 | angle×10 | int16 BE |
| 4 | motor_status | uint8 |
| 5 | sequence | uint8 |
| 6-7 | reserved | - |

Motor status: [0]=ENABLED, [1]=RUNNING, [2]=DIRECTION, [3]=IN_DEADBAND, [4]=AT_LIMIT, [5-7]=SPEED(0-7)

**3. Rudder Command (0x08440001)**
- Priority: 2, Source: 1, Type: 1, Code: 1
- Rate: 10 Hz when ENGAGED

| Byte | Field | Type |
|------|-------|------|
| 0-1 | cmd_angle×10 | int16 BE |
| 2 | flags | uint8 |
| 3 | sequence | uint8 |
| 4-7 | reserved | - |

**4. System Command (0x08040001)**
- Priority: 2, Source: 0, Type: 1, Code: 1
- Commands: 0x01=ENGAGE, 0x02=DISENGAGE, 0x10=CAL_ENTER, 0x11=CAL_EXIT, 0x20=FAULT_CLEAR

**5. E_STOP (0x00040001)**
- Priority: 0 (HIGHEST), Source: 0, Type: 1, Code: 1
- Action: ALL nodes immediately stop motors and enter FAULTED

**6. Calibration Command (0x085C0001)**
- Priority: 2, Source: 1, Type: 7, Code: 1
- Commands: 0x01=CENTER, 0x02=PORT, 0x03=STBD, 0x04=SAVE

**7. Rudder Extended Status (0x10880001)**
- Priority: 4, Source: 2, Type: 2, Code: 1
- Rate: 2 Hz

**8. Performance Telemetry (0x10480001)**
- Priority: 4, Source: 1, Type: 2, Code: 1
- Rate: 1 Hz

**9. Master Error (0x04440001)**
- Priority: 1, Source: 1, Type: 5, Code: 1

**10. Rudder Error (0x04840001)**
- Priority: 1, Source: 2, Type: 5, Code: 1

Error format: [0]=code, [1]=severity(0-3), [2-3]=detail

**11. Parameter Config (0x08580001)**
- Priority: 2, Source: 1, Type: 6, Code: 1
- Direction: Master→Rudder (for Rudder params only)

| Byte | Field | Type |
|------|-------|------|
| 0 | param_id | uint8 |
| 1 | flags | uint8 |
| 2-5 | value | float LE |
| 6-7 | reserved | - |

Flags: [0]=SAVE_TO_NVS

Parameter IDs:
| ID | Name | Node | Default | Range |
|----|------|------|---------|-------|
| 0 | KP_HEADING | Master | 0.8 | 0.1-5.0 |
| 1 | KI_HEADING | Master | 0.05 | 0.0-1.0 |
| 2 | KD_HEADING | Master | 0.5 | 0.0-5.0 |
| 3 | KP_SERVO | Rudder | 10.0 | 1.0-50.0 |
| 4 | DEADBAND_ENTER | Rudder | 1.0° | 0.1-5.0 |
| 5 | DEADBAND_EXIT | Rudder | 1.5° | 0.2-6.0 |
| 6 | MIN_MOTOR_SPEED | Rudder | 20% | 0-50 |
| 7 | MAX_MOTOR_SPEED | Rudder | 100% | 50-100 |
| 8 | RUDDER_SLEW_RATE | Rudder | 15.0°/s | 1.0-60.0 |

### 5.6 Error Code Table

| Code | Name | Severity |
|------|------|----------|
| 0x00 | NONE | - |
| 0x01 | CAN_TX_FAIL | Warning |
| 0x02 | CAN_RX_TIMEOUT | Fault |
| 0x03 | CAN_BUS_OFF | Critical |
| 0x10 | SENSOR_FAULT | Fault |
| 0x11 | SENSOR_RANGE | Warning |
| 0x12 | SENSOR_INIT | Fault |
| 0x20 | MOTOR_STALL | Fault |
| 0x21 | MOTOR_OVERCURRENT | Fault |
| 0x22 | MOTOR_TIMEOUT | Fault |
| 0x30 | CAL_INVALID | Warning |
| 0x31 | CAL_RANGE | Warning |
| 0x40 | HEARTBEAT_LOST | Fault |
| 0x41 | STATE_MISMATCH | Warning |
| 0x50 | LOW_VOLTAGE | Warning |
| 0x51 | OVER_VOLTAGE | Warning |
| 0xFE | WATCHDOG | Critical |
| 0xFF | UNKNOWN | Fault |

### 5.7 Timing Requirements

| Parameter | Value |
|-----------|-------|
| Master heartbeat | 100 ms |
| Rudder heartbeat | 20 ms |
| Command rate | 100 ms |
| Heartbeat timeout | 500 ms |
| Command timeout | 200 ms |
| Command loss fault | 500 ms |
| E_STOP response | 10 ms max |

------

## 6. Software Architecture

### 6.1 Framework: ESP-IDF v5.1+

| Function | ESP-IDF Module |
|----------|----------------|
| RTOS | FreeRTOS |
| I2C | driver/i2c.h |
| UART | driver/uart.h |
| CAN | driver/twai.h |
| GPIO | driver/gpio.h |
| PWM | driver/ledc.h |
| Storage | nvs_flash.h |
| WiFi | esp_wifi.h |
| OTA | esp_ota_ops.h |
| BLE | nimble/nimble_port.h |
| Logging | esp_log.h |

### 6.2 Master Node Tasks

| Task | Pri | Rate | Stack |
|------|-----|------|-------|
| Task_CAN | 5 | Event | 4096 |
| Task_IMU | 4 | 50Hz | 4096 |
| Task_GNSS | 4 | Event | 4096 |
| Task_Autopilot | 3 | 10Hz | 2048 |
| Task_Display | 2 | 5Hz | 4096 |
| Task_BLE | 1 | Event | 4096 |

### 6.3 Rudder Node Tasks

| Task | Pri | Rate | Stack |
|------|-----|------|-------|
| Task_CAN | 5 | Event | 4096 |
| Task_Rudder | 4 | 50Hz | 4096 |

### 6.4 Critical Rule: I2C Mutex

ALL I2C transactions MUST acquire global mutex. Violation = system defect.

------

## 7. Control System

### 7.1 Control Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    MASTER NODE (10 Hz)                       │
│  ┌─────────┐   ┌──────────┐   ┌─────────┐   ┌─────────────┐│
│  │ IMU     │──→│ Adaptive │──→│   PID   │──→│ CAN TX      ││
│  │ Heading │   │ Filter   │   │ Control │   │ Rudder Cmd  ││
│  └─────────┘   └──────────┘   └─────────┘   └─────────────┘│
└─────────────────────────────────────────────────────────────┘
                              │ CAN Bus
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                   RUDDER NODE (50 Hz)                        │
│  ┌─────────┐   ┌──────────┐   ┌─────────┐   ┌─────────────┐│
│  │ CAN RX  │──→│  Servo   │──→│  Motor  │──→│ Actuator    ││
│  │ Command │   │ Control  │   │  PWM    │   │             ││
│  └─────────┘   └──────────┘   └─────────┘   └─────────────┘│
│       ▲                                            │        │
│       │           ┌─────────┐                      │        │
│       └───────────│ AS5600  │◀─────────────────────┘        │
│                   │ Encoder │   Feedback                    │
│                   └─────────┘                               │
└─────────────────────────────────────────────────────────────┘
```

### 7.2 PID Heading Control Algorithm

```c
// 10 Hz execution on Master Node
float error = wrap180(target_heading - filtered_heading);

// Proportional
float P = Kp_heading * error;

// Integral with anti-windup
static float integral = 0;
integral += Ki_heading * error * dt;
integral = clamp(integral, -INTEGRAL_LIMIT, +INTEGRAL_LIMIT);
if (fabs(error) > INTEGRAL_RESET_THRESHOLD) integral = 0;

// Derivative (using gyro, not differentiated error)
float D = -Kd_heading * yaw_rate_filtered;

// Output
float rudder_cmd = P + integral + D;
rudder_cmd = clamp(rudder_cmd, -RUDDER_CMD_MAX, +RUDDER_CMD_MAX);
```

### 7.3 Control Parameters

| Parameter | Value | Unit |
|-----------|-------|------|
| Kp_heading | 0.8 | °/° |
| Ki_heading | 0.05 | °/(°·s) |
| Kd_heading | 0.5 | °/(°/s) |
| INTEGRAL_LIMIT | 5.0 | ° |
| INTEGRAL_RESET_THRESHOLD | 20.0 | ° |
| MAX_YAW_RATE_CMD | 10.0 | °/s |
| RUDDER_CMD_MAX | 35.0 | ° |

### 7.4 Adaptive Heading Filter

```c
float variance = calculate_variance(heading_history, 50); // 1-second window

if (variance < 4.0)       alpha = 0.15;  // CALM
else if (variance < 16.0) alpha = 0.08;  // NORMAL
else if (variance < 36.0) alpha = 0.05;  // ROUGH
else                      alpha = 0.03;  // STORM

filtered = alpha * raw + (1 - alpha) * filtered;
```

| Sea State | Variance | Alpha |
|-----------|----------|-------|
| CALM | < 4° | 0.15 |
| NORMAL | 4-16° | 0.08 |
| ROUGH | 16-36° | 0.05 |
| STORM | > 36° | 0.03 |

### 7.5 Rudder Servo Control

```c
// 50 Hz on Rudder Node
float error = commanded - actual;

// Hysteresis deadband
if (in_deadband && fabs(error) > DEADBAND_EXIT) in_deadband = false;
if (!in_deadband && fabs(error) < DEADBAND_ENTER) {
    in_deadband = true;
    motor_stop();
    return;
}

// Slew-limited proportional
float speed = Kp_servo * error;
float max_delta = RUDDER_SLEW_RATE * dt;
speed = clamp_delta(speed, last_speed, max_delta);
motor_drive(clamp(speed, MIN_SPEED, MAX_SPEED), sign(error));
```

### 7.6 Servo Parameters

| Parameter | Value | Unit |
|-----------|-------|------|
| Kp_servo | 10.0 | %/° |
| DEADBAND_ENTER | 1.0 | ° |
| DEADBAND_EXIT | 1.5 | ° |
| RUDDER_SLEW_RATE | 15.0 | °/s |
| MIN_MOTOR_SPEED | 20 | % |
| MAX_MOTOR_SPEED | 100 | % |

### 7.7 Rudder Command Modes

| Mode | Behavior | Motor |
|------|----------|-------|
| IDLE | Motor disabled, hold position | Disabled |
| SERVO | Active servo to commanded angle | Enabled |
| CENTER | Servo to 0° (center position) | Enabled |
| STOP | Immediate motor stop | Disabled |

**Motor Enable Logic:**
- Motor MUST be disabled when mode = IDLE or STOP
- Motor MAY be enabled when mode = SERVO or CENTER
- Motor SHALL stop immediately on any fault condition

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

### 8.4 State-Task Activation Matrix

| Task | BOOT | IDLE | ENGAGED | CALIBRATION | FAULTED |
|------|------|------|---------|-------------|---------|
| Task_CAN | ✓ | ✓ | ✓ | ✓ | ✓ |
| Task_IMU | - | ✓ | ✓ | ✓ | ✓ |
| Task_GNSS | - | ✓ | ✓ | - | - |
| Task_Autopilot | - | - | ✓ | - | - |
| Task_Rudder | - | ✓ | ✓ | ✓ | - |
| Task_Display | - | ✓ | ✓ | ✓ | ✓ |
| Task_BLE | - | ✓ | ✓ | ✓ | ✓ |

### 8.5 State Invariants

**BOOT State:**
- Motor MUST be disabled
- No commands accepted
- Duration < 10 seconds

**IDLE State:**
- Motor MUST be disabled
- Accepts ENGAGE, CAL_ENTER commands
- Heartbeats transmitted
- Heading sensor active

**ENGAGED State:**
- Motor MAY be enabled (via Task_Rudder)
- Autopilot control active
- All safety monitors active
- Accepts DISENGAGE command

**CALIBRATION State:**
- Motor MUST be disabled
- Manual positioning only
- Timeout: 5 minutes (auto-exit to IDLE)
- Accepts calibration commands only

**FAULTED State:**
- Motor MUST be disabled
- Fault code latched
- Only FAULT_CLEAR accepted
- Heartbeats indicate fault

------

## 9. Safety Protections

### 9.1 Distributed Safety Architecture

| Layer | Location | Responsibility |
|-------|----------|----------------|
| 1 (Local) | Rudder Node | Sensor faults, motor faults, stall detection |
| 2 (System) | Master Node | State management, engage preconditions |
| 3 (Cross-node) | Both | Heartbeat monitoring, command timeout |
| 4 (Hardware) | Both | Watchdog timer, fuses, reverse polarity |

**Global Safety Rule:** Any node detecting a safety fault SHALL broadcast FAULTED state and stop all motor actuation.

### 9.2 Protection Thresholds

| Protection | Threshold | Action |
|------------|-----------|--------|
| Heartbeat timeout | 500 ms | FAULTED |
| Motor stall | < 0.5° in 500ms | FAULTED (0x20) |
| Drive timeout | 5000 ms | FAULTED (0x22) |
| Course deviation warn | 15° for 5s | Log |
| Course deviation alarm | 30° for 5s | Alarm |
| Calibration invalid | range < 5° | Reject ENGAGE |

### 9.3 Sensor Fault Conditions

| Fault Condition | Detection Method | Action |
|-----------------|------------------|--------|
| I2C read failure | NACK or timeout | FAULTED (0x10) |
| Magnet not detected | AS5600 MD bit = 0 | FAULTED (0x10) |
| Magnet too weak | AS5600 ML bit = 1 | Warning (0x11) |
| Magnet too strong | AS5600 MH bit = 1 | Warning (0x11) |
| Heading invalid | NaN or out of range | FAULTED (0x10) |
| IMU I2C failure | NACK or timeout | FAULTED (0x10) |

### 9.4 Command Fallback (Rudder Node)

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
| Module | u-blox (M8/M9 series) |
| Constellations | GPS, GLONASS, Galileo, BeiDou |
| Update Rate | 10 Hz |
| Position Accuracy | 1.5m CEP |
| Velocity Accuracy | 0.05 m/s |
| TTFF (Hot start) | < 2 seconds |
| TTFF (Cold start) | < 26 seconds |
| Default Baud Rate | 9600 |
| Configured Baud Rate | 115200 |

### 10.2 UBX Protocol

- UART: 115200-8N1
- Message: UBX-NAV-PVT (Class 0x01, ID 0x07)
- Rate: 10 Hz

### 10.3 NAV-PVT Key Fields

| Offset | Field | Conversion |
|--------|-------|------------|
| 20 | fixType | 0-5 |
| 24 | lon | ×1e-7 → deg |
| 28 | lat | ×1e-7 → deg |
| 60 | gSpeed | mm/s → kts (×0.00194384) |
| 64 | headMot | ×1e-5 → deg (COG) |

### 10.4 COG Validity

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

### 11.2 BLE Commands

| Code | Command | Parameters | Description |
|------|---------|------------|-------------|
| 0x01 | ENGAGE | - | Engage autopilot |
| 0x02 | DISENGAGE | - | Disengage autopilot |
| 0x03 | SET_HEADING | 2B: hdg×10 BE | Set target heading |
| 0x04 | ADJUST_HEADING | 1B: signed | Adjust heading ±127° |
| 0x10 | CAL_ENTER | - | Enter calibration mode |
| 0x11 | CAL_EXIT | - | Exit calibration mode |
| 0x12 | CAL_CENTER | - | Mark rudder center |
| 0x13 | CAL_PORT | - | Mark port limit |
| 0x14 | CAL_STBD | - | Mark starboard limit |
| 0x15 | CAL_SAVE | - | Save calibration |
| 0x20 | FAULT_CLEAR | - | Clear fault state |
| 0x30 | STATUS_REQ | - | Request status update |
| 0x40 | SEA_STATE_MODE | 1B: mode (0-4) | Set sea state |
| 0x41 | LAZY_HELM | 1B: enable (0/1) | Enable/disable lazy helm |
| 0x42 | STEER_MODE | 1B: mode (0/1) | Steering mode (HDG/COG) |
| 0x43 | GNSS_STATUS_REQ | - | Request GNSS status |
| 0x50 | PARAM_GET | 1B: param_id | Get parameter value |
| 0x51 | PARAM_SET | 1B: param_id + 4B: value (float LE) | Set parameter value |
| 0x52 | PARAM_SAVE | 1B: param_id (0xFF=all) | Save parameter(s) to NVS |
| 0x53 | PARAM_RESET | - | Reset all to defaults |

### 11.3 BLE Timing

| Parameter | Value |
|-----------|-------|
| Advertising Interval | 100-200 ms |
| Connection Interval | 15-30 ms |
| MTU | 256 bytes |

**Security:** Open connection (no pairing required for local boat use).

### 11.4 Parameters Characteristic Format

**Read Response / Notify:** 5 bytes
| Byte | Field | Type |
|------|-------|------|
| 0 | param_id | uint8 |
| 1-4 | value | float LE |

**Write Request:** Same as PARAM_SET command (5 bytes: param_id + float)

------

## 12. Calibration

### 12.1 Procedure

1. `cal enter` - Enter CALIBRATION state
2. Move rudder to center → `cal center`
3. Move to port limit → `cal port`
4. Move to starboard limit → `cal stbd`
5. `cal save` - Save to NVS
6. `cal exit` - Return to IDLE

### 12.2 Validation

- Reject save if range < 5°
- Reject ENGAGE if calibration invalid

------

## 13. Acceptance Criteria

| # | Criterion | Test Method |
|---|-----------|-------------|
| AC-1 | System boots to IDLE within 10 seconds | Console: `state` |
| AC-2 | Autopilot engages only when preconditions met | Console: `engage` |
| AC-3 | Heading error < 3° RMS in CALM | Console: `perf` |
| AC-4 | Rudder responds within 200ms | Console: `heading sim` |
| AC-5 | Stall detection triggers within 600ms | Block rudder |
| AC-6 | Heartbeat loss triggers FAULTED within 500ms | Disconnect CAN |
| AC-7 | DISENGAGE stops motor within 100ms | Console: `disengage` |
| AC-8 | Calibration saves to NVS | Reboot test |
| AC-9 | BLE commands processed within 50ms | BLE test |
| AC-10 | OTA update completes | `idf.py ota` |
| AC-11 | Network console connects within 2 seconds | `telnet` |
| AC-12 | Low voltage warning at 22V | Simulate |
| AC-13 | GNSS loss does not trigger FAULTED | Disconnect GNSS |

------

## Appendix A: CAN Quick Reference

| Message | ID | Pri | Src | Type | Rate |
|---------|-----|-----|-----|------|------|
| Master Heartbeat | 0x10400001 | 4 | 1 | 0 | 10 Hz |
| Rudder Heartbeat | 0x10800001 | 4 | 2 | 0 | 50 Hz |
| Rudder Command | 0x08440001 | 2 | 1 | 1 | 10 Hz |
| System Command | 0x08040001 | 2 | 0 | 1 | Event |
| E_STOP | 0x00040001 | 0 | 0 | 1 | Event |
| Calibration Cmd | 0x085C0001 | 2 | 1 | 7 | Event |
| Rudder Ext Status | 0x10880001 | 4 | 2 | 2 | 2 Hz |
| Performance | 0x10480001 | 4 | 1 | 2 | 1 Hz |
| Master Error | 0x04440001 | 1 | 1 | 5 | Event |
| Rudder Error | 0x04840001 | 1 | 2 | 5 | Event |
| Param Config | 0x08580001 | 2 | 1 | 6 | Event |

------

## Appendix B: Default Parameters

### B.1 Control (Runtime Configurable via BLE/Console/CAN)
| Parameter | Default | Range | NVS Key |
|-----------|---------|-------|---------|
| Kp_heading | 0.8 | 0.1-5.0 | `kp_hdg` |
| Ki_heading | 0.05 | 0.0-1.0 | `ki_hdg` |
| Kd_heading | 0.5 | 0.0-5.0 | `kd_hdg` |

Fixed parameters:
- INTEGRAL_LIMIT = 5.0
- RUDDER_CMD_MAX = 35.0

### B.2 Filter
- CALM_ALPHA = 0.15
- NORMAL_ALPHA = 0.08
- ROUGH_ALPHA = 0.05
- STORM_ALPHA = 0.03

### B.3 Servo (Runtime Configurable via BLE/Console/CAN)
| Parameter | Default | Range | NVS Key |
|-----------|---------|-------|---------|
| Kp_servo | 10.0 | 1.0-50.0 | `kp_srv` |
| DEADBAND_ENTER | 1.0° | 0.1-5.0 | `db_ent` |
| DEADBAND_EXIT | 1.5° | 0.2-6.0 | `db_ext` |

### B.4 Motor (Runtime Configurable via BLE/Console/CAN)
| Parameter | Default | Range | NVS Key |
|-----------|---------|-------|---------|
| MIN_MOTOR_SPEED | 20% | 0-50 | `mot_min` |
| MAX_MOTOR_SPEED | 100% | 50-100 | `mot_max` |
| RUDDER_SLEW_RATE | 15.0°/s | 1.0-60.0 | `slew` |

### B.5 Timing
- HEARTBEAT_TIMEOUT_MS = 500
- COMMAND_TIMEOUT_MS = 200
- STALL_TIMEOUT_MS = 500
- DRIVE_TIMEOUT_MS = 5000

### B.6 Parameter Control Points

Parameters can be modified via three interfaces:
1. **BLE** - Mobile app using Parameters characteristic (0x50-0x53 commands)
2. **Console** - `param`, `pid`, `servo`, `motor` commands via telnet/serial
3. **CAN** - MSG_TYPE_PARAM_CONFIG (0x08580001) from Master→Rudder

All parameters persist to NVS flash when saved.

------

**End of FSD v1.0.0**
