# FSD Generation Instructions for AI Assistants

## Overview

This document provides instructions for AI assistants to generate a Functional Specification Document (FSD) for the TestAPEN marine autopilot project. The FSD serves as the single source of truth for all implementation decisions.

---

## Project Summary

**TestAPEN** is a tri-node marine autopilot system using:
- **ESP-NOW wireless communication** (not CAN bus)
- **Three specialized nodes**: Master, Rudder, UI
- **ESP-IDF framework** (v5.1+) with FreeRTOS
- **ESP32-WROOM-32** microcontrollers on all nodes

### Node Functions

| Node | Primary Function | Communication |
|------|-----------------|---------------|
| **Master** | Autopilot control (PID heading), IMU (ICM-20948), GNSS (u-blox), BLE | Sends heartbeat + commands |
| **Rudder** | Steering servo (AS5600 encoder + motor), Position feedback | Receives commands, sends heartbeat |
| **UI** | User interface (e-Paper display, 6 buttons), Status monitoring | Receives heartbeats, sends commands |

---

## FSD Structure Requirements

### Required Sections

An FSD for TestAPEN MUST include these sections:

1. **Introduction** - Purpose, scope, writing conventions
2. **System Overview** - Application parameters, design philosophy
3. **System Architecture** - Node topology diagram, node functions
4. **Hardware Specification** - GPIO assignments for ALL nodes, physical components
5. **Communication Protocol** - ESP-NOW message types, structures, timing
6. **Software Architecture** - Tasks, priorities, stack sizes for each node
7. **Control System** - PID algorithm, servo control, parameters
8. **State Machine** - States, transitions, preconditions
9. **Safety Protections** - Timeouts, fault detection, recovery
10. **Subsystems** - GNSS, BLE, Display
11. **Calibration** - Procedures for rudder and sensors
12. **Acceptance Criteria** - Measurable test conditions
13. **Appendices** - Quick reference tables, default parameters

### Document Conventions

Use these directives consistently:
- **SHALL** - Mandatory requirement
- **MUST** - Absolute requirement (safety-critical)
- **SHOULD** - Recommended
- **MAY** - Optional

---

## Technical Details to Include

### ESP-NOW Protocol

1. **Message Types** (1-byte type ID + 8-byte payload):
   - 0x01: Master Heartbeat (state, fault, heading, target, flags)
   - 0x02: Rudder Heartbeat (state, fault, angle, motor status)
   - 0x10: Rudder Command (commanded angle)
   - 0x11: System Command (engage, disengage, calibration)
   - 0x13: UI Command (heading adjust from buttons)
   - 0xE0: E-STOP

2. **Message Rates**:
   - Master heartbeat: 10 Hz
   - Rudder heartbeat: 50 Hz
   - Rudder command: 10 Hz (when engaged)

3. **MAC Address Configuration**:
   - Configured via Kconfig (menuconfig)
   - Each node registers other nodes as ESP-NOW peers

### Hardware Specifications

**Master Node:**
- IMU: ICM-20948 (I2C @ 0x68) - 9-axis with sensor fusion
- GNSS: u-blox MIA-M10Q (UART2, 9600→115200 baud)
- Display: SSD1306 OLED 128x64 (I2C @ 0x3C)
- BLE: NimBLE stack for Android app

**Rudder Node:**
- Encoder: AS5600 magnetic encoder (I2C @ 0x36) on motor shaft
- Motor: Cytron MD13S driver (PWM + DIR pins)
- Multi-turn tracking: 2.25 motor rotations = 70° rudder travel

**UI Node:**
- Display: SSD1680 e-Paper 296x128 (SPI)
- Buttons: 6 physical buttons (GPIOs 32-36, 39)
- Partial refresh: ~400ms vs 3s full refresh

### Safety Requirements

Document these safety features:
1. **Heartbeat timeout** (500ms) - Node enters FAULTED
2. **Motor stall detection** (<0.5° in 500ms) - Motor stopped
3. **Drive timeout** (5s continuous motor) - Motor stopped
4. **Manual override** - Worm drive allows physical override

### State Machine

States: BOOT → IDLE → ENGAGED → FAULTED
        IDLE → CALIBRATION → IDLE

Document preconditions for ENGAGE:
- Valid heading
- Valid rudder feedback
- Valid calibration
- No active faults

---

## Verification Requirements

### Console Commands to Document

Each node provides telnet console (port 2323). Document:

**All Nodes:**
- `help`, `status`, `state`, `version`, `reboot`
- `espnow` - ESP-NOW statistics

**Master:**
- `heading`, `imu`, `engage`, `disengage`
- `set heading N`, `adjust N`, `heading sim N`
- `magcal start/stop/status/save`

**Rudder:**
- `rudder`, `motor`, `cal enter/exit/center/port/stbd/save`
- `servo [Kp deadband_in deadband_out]`

**UI:**
- `display`, `page [0-2]`, `nodes`, `buttons`
- `btn list`, `btn N`, `btn N long` (button simulation)

### Test Methods

Include acceptance criteria with specific test methods:
- Console commands for functional tests
- OTA update verification
- Button simulation for UI tests
- Camera verification for display tests

---

## Style Guidelines

### Tables

Use markdown tables for:
- GPIO assignments
- Message structures (byte-by-byte)
- State transitions
- Timing requirements
- Default parameters

### Code Blocks

Use code blocks for:
- ASCII diagrams (node topology)
- Console command examples
- Algorithm pseudocode

### Section References

When referencing implementation details, use format:
- "See Section X.Y for details"
- "Per FSD Section X.Y.Z"

---

## Common Mistakes to Avoid

1. **Don't describe CAN bus** - TestAPEN uses ESP-NOW wireless
2. **Don't forget UI Node** - It's the third node type with display + buttons
3. **Include all GPIO assignments** - All nodes have different configurations
4. **Document partial refresh** - E-paper supports partial updates
5. **Include button simulation** - Critical for automated testing
6. **Document MAC addresses** - Required for ESP-NOW peer registration

---

## Example FSD Generation Prompt

When asked to generate an FSD for TestAPEN, use this context:

```
Generate a Functional Specification Document for TestAPEN, a marine autopilot with:
- 3 nodes communicating via ESP-NOW wireless
- Master node: IMU heading, PID control, GNSS, BLE
- Rudder node: AS5600 encoder, motor driver, servo control
- UI node: 2.9" e-Paper display, 6 buttons, status display
- ESP-IDF framework with FreeRTOS
- WiFi for OTA updates and telnet console on all nodes
- Heading-hold only (no waypoint navigation)
- Safety: heartbeat monitoring, motor stall detection, manual override

Include sections for: hardware specs, ESP-NOW protocol, control algorithms,
state machine, safety protections, calibration procedures, and acceptance criteria.
```

---

## Validation Checklist

Before finalizing an FSD, verify:

- [ ] All three nodes documented (Master, Rudder, UI)
- [ ] ESP-NOW protocol fully specified (not CAN)
- [ ] GPIO assignments for all nodes
- [ ] Button functions documented with simulation commands
- [ ] E-paper display pages described
- [ ] Partial refresh strategy explained
- [ ] State machine with all transitions
- [ ] Safety timeouts specified
- [ ] Console commands for each node
- [ ] Acceptance criteria with test methods
- [ ] Node IP addresses and OTA commands
- [ ] Version history maintained

---

## Version Information

| FSD Version | Date | Key Changes |
|-------------|------|-------------|
| v1.0.0 | 2026-01-13 | Initial CAN-based design |
| v1.0.1 | 2026-01-19 | Renamed to TestAPEN |
| v1.1.0 | 2026-01-20 | ESP-NOW, UI Node, e-Paper |

---

**End of FSD Generation Instructions**
