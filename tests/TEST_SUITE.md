# TestAPEN Comprehensive Test Suite

**Version:** 1.0.0
**Date:** 2026-01-18
**Based on:** FSD v1.0.0 Acceptance Criteria

---

## Table of Contents

1. [Test Environment](#1-test-environment)
2. [Quick Validation Tests](#2-quick-validation-tests)
3. [Acceptance Criteria Tests](#3-acceptance-criteria-tests)
4. [Display Tests](#4-display-tests)
5. [Button Tests](#5-button-tests)
6. [Communication Tests](#6-communication-tests)
7. [Motor Control Tests](#7-motor-control-tests)
8. [Error and Fault Tests](#8-error-and-fault-tests)
9. [BLE Interface Tests](#9-ble-interface-tests)
10. [Calibration Tests](#10-calibration-tests)
11. [Parameter Tests](#11-parameter-tests)
12. [Integration Tests](#12-integration-tests)
13. [Automated Test Scripts](#13-automated-test-scripts)

---

## 1. Test Environment

### 1.1 Node IP Addresses
| Node | IP Address | Telnet Port |
|------|------------|-------------|
| Master | 192.168.1.118 | 2323 |
| Rudder | 192.168.1.157 | 2323 |
| UI | 192.168.1.186 | 2323 |

### 1.2 Node MAC Addresses (ESP-NOW)
| Node | MAC Address |
|------|-------------|
| Master | 78:42:1C:6C:FA:58 |
| Rudder | 78:42:1C:6D:28:94 |
| UI | 78:42:1C:6B:E5:F0 |

### 1.3 Test Equipment Required
- Camera (for display verification)
- Bluetooth adapter (for BLE tests)
- WiFi connectivity to all nodes
- Physical access to UI buttons (or remote actuation)
- Ability to manually position rudder (for calibration tests)

### 1.4 Test Utilities
```bash
# Connect to node console
nc -q 2 <IP> 2323

# Send command and capture output
echo "command" | nc -q 2 <IP> 2323

# Run automated test script
./tests/scripts/run_test.sh <test_name>
```

---

## 2. Quick Validation Tests

### QV-01: All Nodes Online
**Purpose:** Verify all three nodes are running and responsive
**Method:** Console
```bash
# Check Master
(echo "version"; sleep 1) | nc -q 2 192.168.1.118 2323

# Check Rudder
(echo "version"; sleep 1) | nc -q 2 192.168.1.157 2323

# Check UI
(echo "version"; sleep 1) | nc -q 2 192.168.1.186 2323
```
**Pass Criteria:** All nodes respond with version string containing "1.1.0-espnow"

### QV-02: ESP-NOW Communication Active
**Purpose:** Verify ESP-NOW links between all nodes
**Method:** Console
```bash
# Check UI node ESP-NOW stats
(echo "espnow"; sleep 1) | nc -q 2 192.168.1.186 2323
```
**Pass Criteria:**
- RX count > 0 (receiving heartbeats)
- TX failed = 0
- Peers = 3

### QV-03: System State Normal
**Purpose:** Verify system in expected IDLE state
**Method:** Console
```bash
(echo "status"; sleep 1) | nc -q 2 192.168.1.118 2323
```
**Pass Criteria:**
- State = IDLE
- Fault code = 0x00

---

## 3. Acceptance Criteria Tests (from FSD)

### AC-01: Boot to IDLE within 10 seconds
**FSD Requirement:** System boots to IDLE within 10 seconds
**Method:** Reboot and time
```bash
# Trigger reboot
(echo "reboot"; sleep 1) | nc -q 2 192.168.1.118 2323

# Start timer, then check state
sleep 10
(echo "state"; sleep 1) | nc -q 2 192.168.1.118 2323
```
**Pass Criteria:** State = IDLE within 10 seconds of reboot

### AC-02: Engage Preconditions Enforced
**FSD Requirement:** Autopilot engages only when ALL preconditions met
**Method:** Console engage attempts

**Test AC-02a: Normal engage (all preconditions met)**
```bash
# Ensure calibration valid on rudder
(echo "cal center"; sleep 1) | nc -q 2 192.168.1.157 2323
(echo "cal save"; sleep 1) | nc -q 2 192.168.1.157 2323

# Attempt engage
(echo "engage"; sleep 1) | nc -q 2 192.168.1.118 2323
(echo "state"; sleep 1) | nc -q 2 192.168.1.118 2323
```
**Pass Criteria:** State transitions to ENGAGED

**Test AC-02b: Engage with active fault (should fail)**
```bash
# First inject a fault by simulating (if possible)
# Or check that engage fails when already faulted
(echo "disengage"; sleep 1) | nc -q 2 192.168.1.118 2323
```

### AC-03: Heading Error < 3 deg RMS in CALM
**FSD Requirement:** Heading error < 3° RMS in CALM sea state
**Method:** Enable heading simulation, engage, measure error
```bash
# Set simulated heading
(echo "heading sim 180"; sleep 1) | nc -q 2 192.168.1.118 2323

# Set target
(echo "set heading 180"; sleep 1) | nc -q 2 192.168.1.118 2323

# Engage
(echo "engage"; sleep 1) | nc -q 2 192.168.1.118 2323

# Wait for settling, then check heading error
sleep 5
(echo "heading"; sleep 1) | nc -q 2 192.168.1.118 2323
```
**Pass Criteria:** Heading error (|target - actual|) < 3° after settling

### AC-04: Rudder Response within 200ms
**FSD Requirement:** Rudder responds to commands within 200ms
**Method:** Command rudder change, measure response time
```bash
# Engage system
(echo "engage"; sleep 1) | nc -q 2 192.168.1.118 2323

# Change heading to force rudder movement
(echo "adjust 10"; sleep 1) | nc -q 2 192.168.1.118 2323

# Check rudder immediately
(echo "rudder"; sleep 1) | nc -q 2 192.168.1.157 2323
```
**Pass Criteria:** Rudder command received and motor responding within 200ms
**Note:** Use automated script with timestamps for precise measurement

### AC-05: Stall Detection within 600ms
**FSD Requirement:** Stall detection triggers within 600ms
**Method:** Block rudder physically, observe fault
```bash
# Engage and command large rudder movement
(echo "engage"; sleep 1) | nc -q 2 192.168.1.118 2323
(echo "set rudder 30"; sleep 1) | nc -q 2 192.168.1.157 2323

# PHYSICALLY BLOCK RUDDER
# Wait and check for fault
sleep 1
(echo "state"; sleep 1) | nc -q 2 192.168.1.157 2323
```
**Pass Criteria:** Rudder enters FAULTED state with code 0x20 (MOTOR_STALL) within 600ms
**Note:** Requires physical intervention

### AC-06: Heartbeat Loss Triggers FAULTED within 500ms
**FSD Requirement:** Heartbeat loss triggers FAULTED within 500ms
**Method:** Power off Master, observe Rudder fault
```bash
# Check rudder state
(echo "state"; sleep 1) | nc -q 2 192.168.1.157 2323

# POWER OFF MASTER NODE
# Wait 600ms, check rudder state
sleep 1
(echo "state"; sleep 1) | nc -q 2 192.168.1.157 2323
```
**Pass Criteria:** Rudder enters FAULTED with code 0x40 (HEARTBEAT_LOST)
**Note:** Requires physical power control

### AC-07: DISENGAGE Stops Motor within 100ms
**FSD Requirement:** DISENGAGE stops motor within 100ms
**Method:** Engage with rudder moving, disengage, verify stop
```bash
# Engage and command movement
(echo "engage"; sleep 1) | nc -q 2 192.168.1.118 2323
(echo "set rudder 30"; sleep 0.5) | nc -q 2 192.168.1.157 2323

# Disengage while moving
(echo "disengage"; sleep 0.2) | nc -q 2 192.168.1.118 2323

# Check motor status
(echo "motor"; sleep 1) | nc -q 2 192.168.1.157 2323
```
**Pass Criteria:** Motor ENABLED=false, RUNNING=false within 100ms

### AC-08: Calibration Saves to NVS
**FSD Requirement:** Calibration persists across reboots
**Method:** Calibrate, reboot, verify
```bash
# Enter calibration
(echo "cal enter"; sleep 1) | nc -q 2 192.168.1.157 2323

# Set center (at current position)
(echo "cal center"; sleep 1) | nc -q 2 192.168.1.157 2323

# Save
(echo "cal save"; sleep 1) | nc -q 2 192.168.1.157 2323

# Exit calibration
(echo "cal exit"; sleep 1) | nc -q 2 192.168.1.157 2323

# Record calibration value
(echo "rudder"; sleep 1) | nc -q 2 192.168.1.157 2323

# Reboot
(echo "reboot"; sleep 1) | nc -q 2 192.168.1.157 2323
sleep 12

# Verify calibration restored
(echo "rudder"; sleep 1) | nc -q 2 192.168.1.157 2323
```
**Pass Criteria:** Calibration status shows "valid" after reboot

### AC-09: BLE Commands within 50ms
**FSD Requirement:** BLE commands processed within 50ms
**Method:** Send BLE command, measure response time
**See Section 9: BLE Interface Tests**

### AC-10: OTA Update Completes
**FSD Requirement:** OTA update successful
**Method:** Upload firmware via HTTP
```bash
# Build and upload
curl -X POST --data-binary @build/testapen.bin \
  -H "Content-Type: application/octet-stream" \
  http://192.168.1.118/update
```
**Pass Criteria:** "OTA Success! Rebooting..." response

### AC-11: Console Connects within 2 seconds
**FSD Requirement:** Network console connects within 2 seconds
**Method:** Time telnet connection
```bash
time (echo "" | nc -q 1 192.168.1.118 2323)
```
**Pass Criteria:** Connection established in < 2 seconds

### AC-12: Low Voltage Warning at 22V
**FSD Requirement:** Low voltage warning at V < 22V
**Method:** Voltage simulation (if available) or monitor
**Note:** Requires voltage control capability or real battery test

### AC-13: GNSS Loss Does Not Trigger FAULTED
**FSD Requirement:** GNSS loss is non-fatal
**Method:** Check system state with no GNSS
```bash
(echo "gnss"; sleep 1) | nc -q 2 192.168.1.118 2323
(echo "state"; sleep 1) | nc -q 2 192.168.1.118 2323
```
**Pass Criteria:** Even with GNSS invalid, state remains IDLE (not FAULTED)

---

## 4. Display Tests

### 4.1 Master OLED Display (SSD1306)

#### DISP-M01: Display Initialization
**Purpose:** Verify OLED initializes and shows content
**Method:** Visual inspection via camera
```bash
# Take photo of Master OLED
# Check for readable text showing:
# - Heading value
# - Target heading
# - System state
# - WiFi status
```
**Pass Criteria:** Display shows readable status information

#### DISP-M02: Heading Display Updates
**Purpose:** Verify heading display updates in real-time
**Method:** Change heading, verify display update
```bash
# Enable heading simulation
(echo "heading sim 90"; sleep 2) | nc -q 2 192.168.1.118 2323
# Take photo - should show ~90 deg

(echo "heading sim 180"; sleep 2) | nc -q 2 192.168.1.118 2323
# Take photo - should show ~180 deg
```
**Pass Criteria:** Display shows updated heading within 1 second

#### DISP-M03: State Display
**Purpose:** Verify state changes shown on display
**Method:** Change state, verify display
```bash
# Engage
(echo "engage"; sleep 2) | nc -q 2 192.168.1.118 2323
# Photo - should show ENGAGED

# Disengage
(echo "disengage"; sleep 2) | nc -q 2 192.168.1.118 2323
# Photo - should show IDLE
```
**Pass Criteria:** Display reflects current state

### 4.2 UI e-Paper Display (SSD1680)

#### DISP-U01: e-Paper Initialization
**Purpose:** Verify e-Paper display initializes
**Method:** Console check + visual
```bash
(echo "display"; sleep 1) | nc -q 2 192.168.1.186 2323
```
**Pass Criteria:** "Initialized: yes" in response

#### DISP-U02: Page 0 - Autopilot Status
**Purpose:** Verify autopilot status page content
**Method:** Set page 0, take photo
```bash
(echo "page 0"; sleep 3) | nc -q 2 192.168.1.186 2323
# Take photo of e-Paper display
```
**Pass Criteria:** Display shows:
- Heading value
- Target heading
- Rudder angle with PORT/STBD indicator
- State (IDLE/ENGAGED)
- Fault code if any

#### DISP-U03: Page 1 - Navigation
**Purpose:** Verify navigation page
**Method:** Set page 1, take photo
```bash
(echo "page 1"; sleep 3) | nc -q 2 192.168.1.186 2323
```
**Pass Criteria:** Display shows:
- Lat/Lon (or "No Fix" if GNSS unavailable)
- Speed
- COG
- Status bar

#### DISP-U04: Page 2 - System Status
**Purpose:** Verify system status page
**Method:** Set page 2, take photo
```bash
(echo "page 2"; sleep 3) | nc -q 2 192.168.1.186 2323
```
**Pass Criteria:** Display shows:
- Master: connected/disconnected
- Rudder: connected/disconnected
- WiFi IP
- Firmware version

#### DISP-U05: Force Refresh
**Purpose:** Verify manual refresh works
**Method:** Command refresh
```bash
(echo "display refresh"; sleep 3) | nc -q 2 192.168.1.186 2323
```
**Pass Criteria:** Display refreshes (may see brief flash)

#### DISP-U06: Status Bar Icons
**Purpose:** Verify status bar shows connectivity
**Method:** Visual inspection
**Pass Criteria:** Status bar shows M:OK R:OK (or -- if disconnected)

---

## 5. Button Tests

### BTN-01: Button State Query
**Purpose:** Verify button state reporting
**Method:** Console query
```bash
(echo "buttons"; sleep 1) | nc -q 2 192.168.1.186 2323
```
**Pass Criteria:** Shows all 6 buttons with states

### BTN-02: DEC-10 Button (-10 degrees)
**Purpose:** Verify -10 degree heading adjustment
**Method:** Press button, check heading change
```bash
# Note current target
(echo "status"; sleep 1) | nc -q 2 192.168.1.186 2323

# PRESS DEC-10 BUTTON
# Wait for ESP-NOW transmission
sleep 1

# Check new target on Master
(echo "heading"; sleep 1) | nc -q 2 192.168.1.118 2323
```
**Pass Criteria:** Target heading decreased by 10 degrees

### BTN-03: DEC-1 Button (-1 degree)
**Purpose:** Verify -1 degree heading adjustment
**Method:** Same as BTN-02, expect -1 change
**Pass Criteria:** Target heading decreased by 1 degree

### BTN-04: INC+1 Button (+1 degree)
**Purpose:** Verify +1 degree heading adjustment
**Pass Criteria:** Target heading increased by 1 degree

### BTN-05: INC+10 Button (+10 degrees)
**Purpose:** Verify +10 degree heading adjustment
**Pass Criteria:** Target heading increased by 10 degrees

### BTN-06: ENGAGE Button
**Purpose:** Verify engage/disengage toggle
**Method:** Press ENGAGE button
```bash
# Check initial state
(echo "state"; sleep 1) | nc -q 2 192.168.1.118 2323

# PRESS ENGAGE BUTTON
sleep 1

# Check state changed
(echo "state"; sleep 1) | nc -q 2 192.168.1.118 2323
```
**Pass Criteria:** State toggles between IDLE and ENGAGED

### BTN-07: MODE Button (Page Cycle)
**Purpose:** Verify page cycling
**Method:** Press MODE button
```bash
# PRESS MODE BUTTON multiple times
# Observe e-Paper display changing pages
# Page cycles: 0 -> 1 -> 2 -> 0
```
**Pass Criteria:** Display pages cycle correctly

### BTN-08: Long Press Repeat
**Purpose:** Verify long press triggers repeat
**Method:** Hold INC+10 for 2 seconds
```bash
# Record current target
(echo "heading"; sleep 1) | nc -q 2 192.168.1.118 2323

# HOLD INC+10 FOR 2 SECONDS
sleep 2

# Check target - should have increased by multiple of 10
(echo "heading"; sleep 1) | nc -q 2 192.168.1.118 2323
```
**Pass Criteria:** Target increased by 20-50 degrees (multiple repeats at 5Hz)

### BTN-09: Button Debounce
**Purpose:** Verify debounce prevents double-triggers
**Method:** Quick press (< 20ms)
**Pass Criteria:** Single press = single action

---

## 6. Communication Tests

### COMM-01: ESP-NOW Statistics
**Purpose:** Verify ESP-NOW counters
**Method:** Console query on all nodes
```bash
# Master
(echo "espnow"; sleep 1) | nc -q 2 192.168.1.118 2323

# Rudder
(echo "espnow"; sleep 1) | nc -q 2 192.168.1.157 2323

# UI
(echo "espnow"; sleep 1) | nc -q 2 192.168.1.186 2323
```
**Pass Criteria:**
- All show Initialized: yes
- TX failed = 0 on all
- RX count > 0 on all

### COMM-02: Master Heartbeat Reception
**Purpose:** Verify Rudder and UI receive Master heartbeats
**Method:** Check RX counters over time
```bash
# Initial count
(echo "espnow"; sleep 1) | nc -q 2 192.168.1.157 2323 | grep "RX count"
sleep 5
# Count after 5 seconds
(echo "espnow"; sleep 1) | nc -q 2 192.168.1.157 2323 | grep "RX count"
```
**Pass Criteria:** RX count increases (Master sends at 10Hz = ~50 messages in 5s)

### COMM-03: Rudder Heartbeat Reception
**Purpose:** Verify Master and UI receive Rudder heartbeats
**Method:** Same as COMM-02 for Rudder messages
**Pass Criteria:** RX count increases (Rudder sends at 50Hz = ~250 messages in 5s)

### COMM-04: UI Command Transmission
**Purpose:** Verify UI sends commands to Master
**Method:** Press button on UI, verify Master receives
```bash
# Watch Master RX count
(echo "espnow"; sleep 1) | nc -q 2 192.168.1.118 2323 | grep "RX count"

# PRESS UI BUTTON
sleep 1

(echo "espnow"; sleep 1) | nc -q 2 192.168.1.118 2323 | grep "RX count"
```
**Pass Criteria:** Master RX count increases after button press

### COMM-05: Peer Discovery
**Purpose:** Verify all peers configured correctly
**Method:** Check peer list
```bash
(echo "espnow"; sleep 1) | nc -q 2 192.168.1.186 2323 | grep -A5 "Configured Peers"
```
**Pass Criteria:**
- Master: 78:42:1C:6C:FA:58
- Rudder: 78:42:1C:6D:28:94
- UI: 78:42:1C:6B:E5:F0

### COMM-06: Channel Match
**Purpose:** Verify all nodes on same WiFi channel
**Method:** Check channel on all nodes
```bash
(echo "espnow"; sleep 1) | nc -q 2 192.168.1.118 2323 | grep "Channel"
(echo "espnow"; sleep 1) | nc -q 2 192.168.1.157 2323 | grep "Channel"
(echo "espnow"; sleep 1) | nc -q 2 192.168.1.186 2323 | grep "Channel"
```
**Pass Criteria:** All nodes report same channel (e.g., Channel: 9)

---

## 7. Motor Control Tests

### MOT-01: Motor Status Query
**Purpose:** Verify motor status reporting
**Method:** Console query
```bash
(echo "motor"; sleep 1) | nc -q 2 192.168.1.157 2323
```
**Pass Criteria:** Shows ENABLED, RUNNING, DIRECTION flags

### MOT-02: Rudder Position Query
**Purpose:** Verify encoder reading
**Method:** Console query
```bash
(echo "rudder"; sleep 1) | nc -q 2 192.168.1.157 2323
```
**Pass Criteria:** Shows actual angle, commanded angle, raw encoder value

### MOT-03: Direct Rudder Command
**Purpose:** Verify direct rudder positioning
**Method:** Command rudder angle
```bash
# Engage locally on rudder
(echo "engage"; sleep 1) | nc -q 2 192.168.1.157 2323

# Command 10 degrees starboard
(echo "set rudder 10"; sleep 2) | nc -q 2 192.168.1.157 2323

# Check position
(echo "rudder"; sleep 1) | nc -q 2 192.168.1.157 2323
```
**Pass Criteria:** Actual angle approaches 10 degrees

### MOT-04: Port Movement
**Purpose:** Verify port (negative) movement
**Method:** Command negative angle
```bash
(echo "set rudder -15"; sleep 2) | nc -q 2 192.168.1.157 2323
(echo "rudder"; sleep 1) | nc -q 2 192.168.1.157 2323
```
**Pass Criteria:** Actual angle approaches -15 degrees

### MOT-05: Starboard Movement
**Purpose:** Verify starboard (positive) movement
**Method:** Command positive angle
```bash
(echo "set rudder 15"; sleep 2) | nc -q 2 192.168.1.157 2323
(echo "rudder"; sleep 1) | nc -q 2 192.168.1.157 2323
```
**Pass Criteria:** Actual angle approaches +15 degrees

### MOT-06: Center Command
**Purpose:** Verify return to center
**Method:** Command zero
```bash
(echo "set rudder 0"; sleep 2) | nc -q 2 192.168.1.157 2323
(echo "rudder"; sleep 1) | nc -q 2 192.168.1.157 2323
```
**Pass Criteria:** Actual angle approaches 0 degrees

### MOT-07: Limit Clamping (+35)
**Purpose:** Verify positive limit enforced
**Method:** Command beyond limit
```bash
(echo "set rudder 50"; sleep 2) | nc -q 2 192.168.1.157 2323
(echo "rudder"; sleep 1) | nc -q 2 192.168.1.157 2323
```
**Pass Criteria:** Actual angle clamped to +35 degrees

### MOT-08: Limit Clamping (-35)
**Purpose:** Verify negative limit enforced
**Method:** Command beyond limit
```bash
(echo "set rudder -50"; sleep 2) | nc -q 2 192.168.1.157 2323
(echo "rudder"; sleep 1) | nc -q 2 192.168.1.157 2323
```
**Pass Criteria:** Actual angle clamped to -35 degrees

### MOT-09: Deadband Entry
**Purpose:** Verify motor stops in deadband
**Method:** Command small error
```bash
(echo "set rudder 0"; sleep 2) | nc -q 2 192.168.1.157 2323
# Motor should stop when within deadband (1.0 deg)
(echo "motor"; sleep 1) | nc -q 2 192.168.1.157 2323
```
**Pass Criteria:** RUNNING = false, IN_DEADBAND = true

### MOT-10: Motor Disable on Disengage
**Purpose:** Verify motor disabled when disengaged
**Method:** Disengage, check motor
```bash
(echo "disengage"; sleep 1) | nc -q 2 192.168.1.157 2323
(echo "motor"; sleep 1) | nc -q 2 192.168.1.157 2323
```
**Pass Criteria:** ENABLED = false

### MOT-11: Servo Parameters
**Purpose:** Verify servo parameter query
**Method:** Console query
```bash
(echo "servo"; sleep 1) | nc -q 2 192.168.1.157 2323
```
**Pass Criteria:** Shows Kp, deadband_enter, deadband_exit

### MOT-12: Motor Parameters
**Purpose:** Verify motor parameter query
**Method:** Console query
```bash
(echo "motor params"; sleep 1) | nc -q 2 192.168.1.157 2323
```
**Pass Criteria:** Shows min_speed, max_speed, slew_rate

---

## 8. Error and Fault Tests

### ERR-01: Fault Code Display
**Purpose:** Verify fault code shown in status
**Method:** Console query
```bash
(echo "state"; sleep 1) | nc -q 2 192.168.1.118 2323
```
**Pass Criteria:** Shows state and fault_code fields

### ERR-02: Fault Clear Command
**Purpose:** Verify fault clearing works
**Method:** Clear fault via console
```bash
# First ensure we're in FAULTED (may need to induce fault)
(echo "fault clear"; sleep 1) | nc -q 2 192.168.1.118 2323
(echo "state"; sleep 1) | nc -q 2 192.168.1.118 2323
```
**Pass Criteria:** State returns to IDLE, fault_code = 0

### ERR-03: Heartbeat Timeout Fault (Master Loss)
**Purpose:** Verify Rudder faults when Master heartbeat lost
**Method:** Requires Master power cycle
**Pass Criteria:** Rudder fault_code = 0x40 (HEARTBEAT_LOST)

### ERR-04: Heartbeat Timeout Fault (Rudder Loss)
**Purpose:** Verify Master detects Rudder heartbeat loss
**Method:** Requires Rudder power cycle
**Pass Criteria:** Master fault_code = 0x40 (HEARTBEAT_LOST)

### ERR-05: Motor Stall Detection
**Purpose:** Verify stall fault triggers
**Method:** Block rudder while commanding movement
**Pass Criteria:** Rudder fault_code = 0x20 (MOTOR_STALL)

### ERR-06: Motor Timeout Detection
**Purpose:** Verify drive timeout triggers
**Method:** Command large movement, observe 5-second timeout
```bash
(echo "engage"; sleep 1) | nc -q 2 192.168.1.157 2323
(echo "set rudder 35"; sleep 6) | nc -q 2 192.168.1.157 2323
(echo "state"; sleep 1) | nc -q 2 192.168.1.157 2323
```
**Pass Criteria:** fault_code = 0x22 (MOTOR_TIMEOUT) after 5 seconds continuous motion

### ERR-07: Calibration Invalid Warning
**Purpose:** Verify invalid calibration detected
**Method:** Check with no calibration
```bash
(echo "param reset"; sleep 1) | nc -q 2 192.168.1.157 2323
(echo "rudder"; sleep 1) | nc -q 2 192.168.1.157 2323
```
**Pass Criteria:** Shows "Calibration: invalid" or similar

### ERR-08: Sensor Init Fault
**Purpose:** Verify sensor fault handling
**Note:** Would require hardware failure simulation

### ERR-09: Error Recovery Sequence
**Purpose:** Verify full fault recovery
**Method:** Induce fault, clear, re-engage
```bash
# Assume faulted
(echo "fault clear"; sleep 1) | nc -q 2 192.168.1.118 2323
(echo "engage"; sleep 1) | nc -q 2 192.168.1.118 2323
(echo "state"; sleep 1) | nc -q 2 192.168.1.118 2323
```
**Pass Criteria:** Successfully transitions FAULTED -> IDLE -> ENGAGED

---

## 9. BLE Interface Tests

### BLE-01: Device Discovery
**Purpose:** Verify BLE device visible
**Method:** Bluetooth scan
```bash
bluetoothctl scan on
# Look for "TestAPEN" device
```
**Pass Criteria:** Device "TestAPEN" appears in scan

### BLE-02: BLE Connect
**Purpose:** Verify BLE connection
**Method:** Connect via bluetoothctl
```bash
bluetoothctl
> connect <MAC_ADDRESS>
```
**Pass Criteria:** Connection established

### BLE-03: Service Discovery
**Purpose:** Verify GATT service available
**Method:** List services after connect
```bash
bluetoothctl
> menu gatt
> list-attributes
```
**Pass Criteria:** Service UUID 12345678-1234-1234-1234-123456789abc visible

### BLE-04: BLE Engage Command
**Purpose:** Verify engage via BLE
**Method:** Write to command characteristic
```bash
# Write 0x01 to command characteristic
bluetoothctl
> select-attribute <command_uuid>
> write 0x01
```
**Pass Criteria:** System engages (verify via telnet)

### BLE-05: BLE Disengage Command
**Purpose:** Verify disengage via BLE
**Method:** Write 0x02 to command
**Pass Criteria:** System disengages

### BLE-06: BLE Heading Adjust
**Purpose:** Verify heading adjustment via BLE
**Method:** Write 0x04 + delta to command
**Pass Criteria:** Target heading changes

### BLE-07: BLE Status Notification
**Purpose:** Verify status notifications received
**Method:** Subscribe to status characteristic
**Pass Criteria:** Notifications received with state/heading data

### BLE-08: BLE Parameter Read
**Purpose:** Verify parameter read via BLE
**Method:** Read parameters characteristic
**Pass Criteria:** Returns current parameter values

### BLE-09: BLE Parameter Write
**Purpose:** Verify parameter write via BLE
**Method:** Write parameter value
**Pass Criteria:** Parameter updated (verify via telnet)

### BLE-10: BLE Fault Clear
**Purpose:** Verify fault clear via BLE
**Method:** Write 0x20 to command
**Pass Criteria:** Fault cleared

---

## 10. Calibration Tests

### CAL-01: Enter Calibration Mode
**Purpose:** Verify calibration mode entry
**Method:** Console command
```bash
(echo "cal enter"; sleep 1) | nc -q 2 192.168.1.157 2323
(echo "state"; sleep 1) | nc -q 2 192.168.1.157 2323
```
**Pass Criteria:** State = CALIBRATION (0x03)

### CAL-02: Center Capture
**Purpose:** Verify center position capture
**Method:** Capture at current position
```bash
(echo "cal center"; sleep 1) | nc -q 2 192.168.1.157 2323
(echo "rudder"; sleep 1) | nc -q 2 192.168.1.157 2323
```
**Pass Criteria:** Center position recorded

### CAL-03: Port Limit Capture
**Purpose:** Verify port limit capture
**Method:** Move to port, capture
```bash
# MANUALLY MOVE RUDDER TO PORT LIMIT
(echo "cal port"; sleep 1) | nc -q 2 192.168.1.157 2323
```
**Pass Criteria:** Port limit recorded

### CAL-04: Starboard Limit Capture
**Purpose:** Verify starboard limit capture
**Method:** Move to starboard, capture
```bash
# MANUALLY MOVE RUDDER TO STARBOARD LIMIT
(echo "cal stbd"; sleep 1) | nc -q 2 192.168.1.157 2323
```
**Pass Criteria:** Starboard limit recorded

### CAL-05: Calibration Save
**Purpose:** Verify calibration persistence
**Method:** Save command
```bash
(echo "cal save"; sleep 1) | nc -q 2 192.168.1.157 2323
```
**Pass Criteria:** "Calibration saved" or similar

### CAL-06: Exit Calibration Mode
**Purpose:** Verify calibration exit
**Method:** Exit command
```bash
(echo "cal exit"; sleep 1) | nc -q 2 192.168.1.157 2323
(echo "state"; sleep 1) | nc -q 2 192.168.1.157 2323
```
**Pass Criteria:** State = IDLE

### CAL-07: Calibration Validation (Range Check)
**Purpose:** Verify range validation
**Method:** Attempt save with insufficient range
```bash
# Only set center, no limits
(echo "cal enter"; sleep 1) | nc -q 2 192.168.1.157 2323
(echo "cal center"; sleep 1) | nc -q 2 192.168.1.157 2323
(echo "cal save"; sleep 1) | nc -q 2 192.168.1.157 2323
```
**Pass Criteria:** Save rejected if range < 5 degrees

### CAL-08: Calibration Timeout
**Purpose:** Verify 5-minute auto-exit
**Method:** Enter calibration, wait 5 minutes
**Pass Criteria:** Auto-exits to IDLE after 5 minutes

### CAL-09: Magnetometer Calibration Start
**Purpose:** Verify magcal procedure start
**Method:** Console command
```bash
(echo "magcal start"; sleep 1) | nc -q 2 192.168.1.118 2323
```
**Pass Criteria:** "Magnetometer calibration started" message

### CAL-10: Magnetometer Calibration Status
**Purpose:** Verify calibration progress tracking
**Method:** Check status during calibration
```bash
(echo "magcal status"; sleep 1) | nc -q 2 192.168.1.118 2323
```
**Pass Criteria:** Shows sample count and coverage

---

## 11. Parameter Tests

### PARAM-01: Parameter List
**Purpose:** Verify parameter listing
**Method:** Console command
```bash
(echo "param list"; sleep 1) | nc -q 2 192.168.1.118 2323
```
**Pass Criteria:** Shows all parameters with names, values, ranges

### PARAM-02: Parameter Get
**Purpose:** Verify individual parameter read
**Method:** Get command
```bash
(echo "param get kp_hdg"; sleep 1) | nc -q 2 192.168.1.118 2323
```
**Pass Criteria:** Shows kp_hdg value (default 0.8)

### PARAM-03: Parameter Set
**Purpose:** Verify parameter modification
**Method:** Set command
```bash
(echo "param set kp_hdg 1.0"; sleep 1) | nc -q 2 192.168.1.118 2323
(echo "param get kp_hdg"; sleep 1) | nc -q 2 192.168.1.118 2323
```
**Pass Criteria:** Value changed to 1.0

### PARAM-04: Parameter Save
**Purpose:** Verify NVS persistence
**Method:** Save and reboot
```bash
(echo "param save"; sleep 1) | nc -q 2 192.168.1.118 2323
(echo "reboot"; sleep 1) | nc -q 2 192.168.1.118 2323
sleep 12
(echo "param get kp_hdg"; sleep 1) | nc -q 2 192.168.1.118 2323
```
**Pass Criteria:** Value persists after reboot

### PARAM-05: Parameter Reset
**Purpose:** Verify default restoration
**Method:** Reset command
```bash
(echo "param reset"; sleep 1) | nc -q 2 192.168.1.118 2323
(echo "param get kp_hdg"; sleep 1) | nc -q 2 192.168.1.118 2323
```
**Pass Criteria:** Value returns to default (0.8)

### PARAM-06: Parameter Range Validation
**Purpose:** Verify out-of-range rejection
**Method:** Set invalid value
```bash
(echo "param set kp_hdg 100.0"; sleep 1) | nc -q 2 192.168.1.118 2323
```
**Pass Criteria:** Rejected with error message

### PARAM-07: PID Parameters
**Purpose:** Verify PID parameter access
**Method:** PID command
```bash
(echo "pid"; sleep 1) | nc -q 2 192.168.1.118 2323
```
**Pass Criteria:** Shows Kp, Ki, Kd values

### PARAM-08: Servo Parameters
**Purpose:** Verify servo parameter access
**Method:** Servo command
```bash
(echo "servo"; sleep 1) | nc -q 2 192.168.1.157 2323
```
**Pass Criteria:** Shows Kp, deadband values

---

## 12. Integration Tests

### INT-01: Full Engage Cycle
**Purpose:** Verify complete engage/disengage cycle
**Method:** Full sequence
```bash
# Setup
(echo "heading sim 180"; sleep 1) | nc -q 2 192.168.1.118 2323
(echo "set heading 180"; sleep 1) | nc -q 2 192.168.1.118 2323

# Engage
(echo "engage"; sleep 1) | nc -q 2 192.168.1.118 2323
sleep 3

# Verify engaged
(echo "state"; sleep 1) | nc -q 2 192.168.1.118 2323

# Change heading
(echo "adjust 20"; sleep 3) | nc -q 2 192.168.1.118 2323

# Verify tracking
(echo "heading"; sleep 1) | nc -q 2 192.168.1.118 2323

# Disengage
(echo "disengage"; sleep 1) | nc -q 2 192.168.1.118 2323

# Verify disengaged
(echo "state"; sleep 1) | nc -q 2 192.168.1.118 2323
```
**Pass Criteria:** Complete cycle without errors

### INT-02: UI-to-Master-to-Rudder Flow
**Purpose:** Verify end-to-end command flow
**Method:** Button press, verify rudder response
```bash
# Engage via Master
(echo "engage"; sleep 1) | nc -q 2 192.168.1.118 2323

# PRESS UI INC+10 BUTTON
sleep 2

# Check Master target changed
(echo "heading"; sleep 1) | nc -q 2 192.168.1.118 2323

# Check Rudder received command
(echo "rudder"; sleep 1) | nc -q 2 192.168.1.157 2323
```
**Pass Criteria:** Heading target increases, rudder responds

### INT-03: Cross-Node State Sync
**Purpose:** Verify state consistent across nodes
**Method:** Change state, check all nodes
```bash
# Engage
(echo "engage"; sleep 1) | nc -q 2 192.168.1.118 2323
sleep 1

# Check Master
(echo "state"; sleep 1) | nc -q 2 192.168.1.118 2323

# Check UI status (should show engaged)
(echo "status"; sleep 1) | nc -q 2 192.168.1.186 2323
```
**Pass Criteria:** All nodes show ENGAGED state

### INT-04: Fault Propagation
**Purpose:** Verify faults propagate across nodes
**Method:** Induce fault, check all nodes
**Pass Criteria:** Fault visible on all node status

### INT-05: Recovery After Fault
**Purpose:** Verify full system recovery
**Method:** Fault, clear, re-engage sequence
**Pass Criteria:** System fully operational after recovery

### INT-06: Continuous Operation (1 hour)
**Purpose:** Verify stability over time
**Method:** Engage, run for 1 hour
```bash
# Engage with heading hold
(echo "heading sim 180"; sleep 1) | nc -q 2 192.168.1.118 2323
(echo "set heading 180"; sleep 1) | nc -q 2 192.168.1.118 2323
(echo "engage"; sleep 1) | nc -q 2 192.168.1.118 2323

# Log status every minute for 1 hour
for i in {1..60}; do
  sleep 60
  echo "=== Minute $i ==="
  (echo "status"; sleep 1) | nc -q 2 192.168.1.118 2323
  (echo "espnow"; sleep 1) | nc -q 2 192.168.1.118 2323 | grep "TX failed"
done
```
**Pass Criteria:** No faults, TX failures remain 0

---

## 13. Automated Test Scripts

The following automated test scripts are provided in `/home/cas/TestAPEN/tests/scripts/`:

- `run_all_tests.sh` - Run complete test suite
- `quick_validation.sh` - Run quick validation tests only
- `acceptance_tests.sh` - Run FSD acceptance criteria tests
- `comm_tests.sh` - Run communication tests
- `motor_tests.sh` - Run motor control tests
- `display_verify.sh` - Display verification with camera
- `ble_tests.sh` - BLE interface tests
- `stress_test.sh` - Long-duration stress test

### Usage:
```bash
cd /home/cas/TestAPEN/tests/scripts
./run_all_tests.sh 2>&1 | tee test_results.log
```

### Test Result Format:
```
[PASS] TEST_ID: Description
[FAIL] TEST_ID: Description - Expected: X, Got: Y
[SKIP] TEST_ID: Description - Requires manual intervention
```

---

## Appendix A: Error Code Reference

| Code | Name | Severity | Cause |
|------|------|----------|-------|
| 0x00 | ERR_NONE | - | No error |
| 0x01 | ERR_ESPNOW_TX_FAIL | Warning | Wireless TX failed |
| 0x02 | ERR_ESPNOW_RX_TIMEOUT | Fault | No heartbeat received |
| 0x10 | ERR_SENSOR_FAULT | Fault | I2C failure |
| 0x20 | ERR_MOTOR_STALL | Fault | Motor blocked |
| 0x22 | ERR_MOTOR_TIMEOUT | Fault | Motor ran > 5s |
| 0x30 | ERR_CAL_INVALID | Warning | Calibration range < 5° |
| 0x40 | ERR_HEARTBEAT_LOST | Fault | Peer heartbeat timeout |

---

## Appendix B: State Reference

| Value | State | Motor |
|-------|-------|-------|
| 0x00 | BOOT | Off |
| 0x01 | IDLE | Off |
| 0x02 | ENGAGED | May be On |
| 0x03 | CALIBRATION | Off |
| 0xFF | FAULTED | Off |

---

**End of Test Suite Document**
