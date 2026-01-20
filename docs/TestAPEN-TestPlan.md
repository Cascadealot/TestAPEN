# TestAPEN Automated Test Plan

## Overview

This document defines automated tests that Claude Code can execute to verify full functionality of the TestAPEN tri-node autopilot system. Tests use:
- **Telnet Console** (port 2323) for command execution and status verification
- **USB Camera** (fswebcam) for visual display verification
- **Button Simulation** (`btn N`) for UI interaction testing

## Node Configuration

| Node | IP Address | Telnet Port | Function |
|------|------------|-------------|----------|
| Master | 192.168.1.118 | 2323 | IMU, heading, autopilot control |
| Rudder | 192.168.1.157 | 2323 | AS5600 encoder, motor control |
| UI | 192.168.1.186 | 2323 | e-Paper display, buttons |

## Test Categories

1. [Network Connectivity](#1-network-connectivity)
2. [ESP-NOW Communication](#2-esp-now-communication)
3. [Master Node Functions](#3-master-node-functions)
4. [Rudder Node Functions](#4-rudder-node-functions)
5. [UI Node Display](#5-ui-node-display)
6. [Button Input](#6-button-input)
7. [Autopilot Engagement](#7-autopilot-engagement)
8. [Heading Adjustment](#8-heading-adjustment)
9. [State Machine](#9-state-machine)
10. [OTA Updates](#10-ota-updates)

---

## 1. Network Connectivity

### Test 1.1: Master Node WiFi
**Command:**
```bash
curl -s --connect-timeout 2 http://192.168.1.118/update -o /dev/null -w "%{http_code}"
```
**Expected:** `405` (indicates OTA endpoint is active)

### Test 1.2: Rudder Node WiFi
**Command:**
```bash
curl -s --connect-timeout 2 http://192.168.1.157/update -o /dev/null -w "%{http_code}"
```
**Expected:** `405`

### Test 1.3: UI Node WiFi
**Command:**
```bash
curl -s --connect-timeout 2 http://192.168.1.186/update -o /dev/null -w "%{http_code}"
```
**Expected:** `405`

### Test 1.4: Master Telnet Console
**Command:**
```bash
(echo "version"; sleep 1) | nc -q 2 192.168.1.118 2323
```
**Expected:** Contains "TestAPEN" and version string

### Test 1.5: Rudder Telnet Console
**Command:**
```bash
(echo "version"; sleep 1) | nc -q 2 192.168.1.157 2323
```
**Expected:** Contains "TestAPEN" and version string

### Test 1.6: UI Telnet Console
**Command:**
```bash
(echo "version"; sleep 1) | nc -q 2 192.168.1.186 2323
```
**Expected:** Contains "TestAPEN" and version string

---

## 2. ESP-NOW Communication

### Test 2.1: Master ESP-NOW Status
**Command:**
```bash
(echo "espnow"; sleep 1) | nc -q 2 192.168.1.118 2323
```
**Expected:** Shows peers, TX/RX counts

### Test 2.2: Rudder ESP-NOW Status
**Command:**
```bash
(echo "espnow"; sleep 1) | nc -q 2 192.168.1.157 2323
```
**Expected:** Shows peers, TX/RX counts

### Test 2.3: UI Receives Master Heartbeat
**Command:**
```bash
(echo "nodes"; sleep 1) | nc -q 2 192.168.1.186 2323
```
**Expected:** Contains "Master: CONNECTED"

### Test 2.4: UI Receives Rudder Heartbeat
**Command:**
```bash
(echo "nodes"; sleep 1) | nc -q 2 192.168.1.186 2323
```
**Expected:** Contains "Rudder: CONNECTED"

---

## 3. Master Node Functions

### Test 3.1: IMU Reading
**Command:**
```bash
(echo "status"; sleep 1) | nc -q 2 192.168.1.118 2323
```
**Expected:** Contains "Heading:" with a numeric value (0-360)

### Test 3.2: IMU Simulation Mode
**Command:**
```bash
(echo "heading sim 180"; sleep 1; echo "status"; sleep 1) | nc -q 2 192.168.1.118 2323
```
**Expected:** Heading shows approximately 180.0 degrees

### Test 3.3: Return to Real IMU
**Command:**
```bash
(echo "heading real"; sleep 1; echo "status"; sleep 1) | nc -q 2 192.168.1.118 2323
```
**Expected:** Heading shows real IMU reading, "Simulation: OFF"

### Test 3.4: State Machine Initial State
**Command:**
```bash
(echo "state"; sleep 1) | nc -q 2 192.168.1.118 2323
```
**Expected:** Contains "IDLE"

---

## 4. Rudder Node Functions

### Test 4.1: AS5600 Encoder Reading
**Command:**
```bash
(echo "status"; sleep 1) | nc -q 2 192.168.1.157 2323
```
**Expected:** Contains "Raw Angle:" with numeric value (0-4095)

### Test 4.2: Rudder Angle Calculation
**Command:**
```bash
(echo "status"; sleep 1) | nc -q 2 192.168.1.157 2323
```
**Expected:** Contains "Rudder:" with angle in degrees (-35 to +35)

### Test 4.3: Motor Status
**Command:**
```bash
(echo "status"; sleep 1) | nc -q 2 192.168.1.157 2323
```
**Expected:** Contains "Motor: ENABLED"

### Test 4.4: Calibration Status
**Command:**
```bash
(echo "status"; sleep 1) | nc -q 2 192.168.1.157 2323
```
**Expected:** Contains "Calibration: VALID"

---

## 5. UI Node Display

### Test 5.1: Display Initialization
**Command:**
```bash
(echo "display"; sleep 1) | nc -q 2 192.168.1.186 2323
```
**Expected:** Contains "Initialized: YES"

### Test 5.2: Visual Display Verification (Camera)
**Command:**
```bash
fswebcam -d /dev/video0 -r 1280x720 --no-banner /tmp/test_display.jpg
```
**Verification:** View image - should show e-Paper with text content

### Test 5.3: Display Refresh
**Command:**
```bash
(echo "display refresh"; sleep 5) | nc -q 6 192.168.1.186 2323
```
**Expected:** Contains "Display refresh triggered"

### Test 5.4: Page 0 (Autopilot)
**Command:**
```bash
(echo "page 0"; sleep 3) | nc -q 4 192.168.1.186 2323 && fswebcam -d /dev/video0 -r 1280x720 --no-banner /tmp/test_page0.jpg
```
**Verification:** Image shows HDG, TGT, RUD values

### Test 5.5: Page 1 (Navigation)
**Command:**
```bash
(echo "page 1"; sleep 3) | nc -q 4 192.168.1.186 2323 && fswebcam -d /dev/video0 -r 1280x720 --no-banner /tmp/test_page1.jpg
```
**Verification:** Image shows "NAVIGATION" title

### Test 5.6: Page 2 (System)
**Command:**
```bash
(echo "page 2"; sleep 3) | nc -q 4 192.168.1.186 2323 && fswebcam -d /dev/video0 -r 1280x720 --no-banner /tmp/test_page2.jpg
```
**Verification:** Image shows "SYSTEM STATUS", IP address

---

## 6. Button Input

### Test 6.1: Button List
**Command:**
```bash
(echo "btn list"; sleep 1) | nc -q 2 192.168.1.186 2323
```
**Expected:** Shows button ID mapping (0-5)

### Test 6.2: Button States
**Command:**
```bash
(echo "buttons"; sleep 1) | nc -q 2 192.168.1.186 2323
```
**Expected:** Shows all 6 buttons with GPIO numbers and states

### Test 6.3: Mode Button Simulation (Page Cycle)
**Command:**
```bash
(echo "page 0"; sleep 2; echo "btn 3"; sleep 3; echo "status"; sleep 1) | nc -q 8 192.168.1.186 2323
```
**Expected:** Page cycles from 0 to 1

---

## 7. Autopilot Engagement

### Test 7.1: Pre-Engagement State
**Command:**
```bash
(echo "status"; sleep 1) | nc -q 2 192.168.1.118 2323
```
**Expected:** State = IDLE

### Test 7.2: Engage via Master Console
**Command:**
```bash
(echo "engage"; sleep 2; echo "status"; sleep 1) | nc -q 4 192.168.1.118 2323
```
**Expected:** State changes to ENGAGED

### Test 7.3: Verify UI Shows Engaged State
**Command:**
```bash
(echo "status"; sleep 1) | nc -q 2 192.168.1.186 2323
```
**Expected:** Contains "state=ENGAGED"

### Test 7.4: Disengage via Master Console
**Command:**
```bash
(echo "disengage"; sleep 2; echo "status"; sleep 1) | nc -q 4 192.168.1.118 2323
```
**Expected:** State returns to IDLE

### Test 7.5: Engage via UI Button Simulation
**Command:**
```bash
(echo "disengage"; sleep 1) | nc -q 2 192.168.1.118 2323
(echo "btn 2"; sleep 2; echo "status"; sleep 1) | nc -q 4 192.168.1.186 2323
```
**Expected:** State changes to ENGAGED

### Test 7.6: Disengage via UI Button Simulation
**Command:**
```bash
(echo "btn 2"; sleep 2; echo "status"; sleep 1) | nc -q 4 192.168.1.186 2323
```
**Expected:** State changes to IDLE

---

## 8. Heading Adjustment

### Test 8.1: Set Target Heading (Master)
**Command:**
```bash
(echo "set heading 90"; sleep 1; echo "status"; sleep 1) | nc -q 3 192.168.1.118 2323
```
**Expected:** Target shows 90.0 degrees

### Test 8.2: Adjust Heading +10 (UI Button)
**Command:**
```bash
(echo "set heading 100"; sleep 1) | nc -q 2 192.168.1.118 2323
(echo "btn 5"; sleep 2; echo "status"; sleep 1) | nc -q 4 192.168.1.186 2323
```
**Expected:** Target increases by 10 degrees (110.0)

### Test 8.3: Adjust Heading -10 (UI Button)
**Command:**
```bash
(echo "btn 0"; sleep 2; echo "status"; sleep 1) | nc -q 4 192.168.1.186 2323
```
**Expected:** Target decreases by 10 degrees (100.0)

### Test 8.4: Adjust Heading +1 (UI Button)
**Command:**
```bash
(echo "btn 4"; sleep 2; echo "status"; sleep 1) | nc -q 4 192.168.1.186 2323
```
**Expected:** Target increases by 1 degree (101.0)

### Test 8.5: Adjust Heading -1 (UI Button)
**Command:**
```bash
(echo "btn 1"; sleep 2; echo "status"; sleep 1) | nc -q 4 192.168.1.186 2323
```
**Expected:** Target decreases by 1 degree (100.0)

---

## 9. State Machine

### Test 9.1: Verify State Transitions
**Command:**
```bash
(echo "disengage"; sleep 1; echo "state"; sleep 1) | nc -q 3 192.168.1.118 2323
```
**Expected:** Shows "IDLE"

### Test 9.2: Calibration Mode Entry
**Command:**
```bash
(echo "cal enter"; sleep 1; echo "state"; sleep 1) | nc -q 3 192.168.1.118 2323
```
**Expected:** Shows "CALIBRATION"

### Test 9.3: Calibration Mode Exit
**Command:**
```bash
(echo "cal exit"; sleep 1; echo "state"; sleep 1) | nc -q 3 192.168.1.118 2323
```
**Expected:** Shows "IDLE"

### Test 9.4: Fault Clear
**Command:**
```bash
(echo "fault clear"; sleep 1; echo "state"; sleep 1) | nc -q 3 192.168.1.118 2323
```
**Expected:** Fault code = 0

---

## 10. OTA Updates

### Test 10.1: Master OTA Endpoint
**Command:**
```bash
curl -s -o /dev/null -w "%{http_code}" http://192.168.1.118/update
```
**Expected:** `405` (GET not allowed, POST required)

### Test 10.2: Rudder OTA Endpoint
**Command:**
```bash
curl -s -o /dev/null -w "%{http_code}" http://192.168.1.157/update
```
**Expected:** `405`

### Test 10.3: UI OTA Endpoint
**Command:**
```bash
curl -s -o /dev/null -w "%{http_code}" http://192.168.1.186/update
```
**Expected:** `405`

---

## Automated Test Runner

To run all tests automatically, use this script:

```bash
#!/bin/bash
# TestAPEN Automated Test Runner

MASTER_IP="192.168.1.118"
RUDDER_IP="192.168.1.157"
UI_IP="192.168.1.186"
PASS=0
FAIL=0

test_result() {
    if [ $1 -eq 0 ]; then
        echo "[PASS] $2"
        ((PASS++))
    else
        echo "[FAIL] $2"
        ((FAIL++))
    fi
}

echo "=== TestAPEN Automated Test Suite ==="
echo "Date: $(date)"
echo ""

# Test 1.1: Master WiFi
result=$(curl -s --connect-timeout 2 http://$MASTER_IP/update -o /dev/null -w "%{http_code}" 2>/dev/null)
[ "$result" = "405" ]
test_result $? "Master WiFi connectivity"

# Test 1.2: Rudder WiFi
result=$(curl -s --connect-timeout 2 http://$RUDDER_IP/update -o /dev/null -w "%{http_code}" 2>/dev/null)
[ "$result" = "405" ]
test_result $? "Rudder WiFi connectivity"

# Test 1.3: UI WiFi
result=$(curl -s --connect-timeout 2 http://$UI_IP/update -o /dev/null -w "%{http_code}" 2>/dev/null)
[ "$result" = "405" ]
test_result $? "UI WiFi connectivity"

# Test 2.3: UI receives Master heartbeat
result=$(echo "nodes" | nc -q 2 $UI_IP 2323 2>/dev/null)
echo "$result" | grep -qi "Master.*CONNECTED"
test_result $? "UI receives Master heartbeat"

# Test 2.4: UI receives Rudder heartbeat
echo "$result" | grep -qi "Rudder.*CONNECTED"
test_result $? "UI receives Rudder heartbeat"

# Test 3.1: Master IMU reading
result=$(echo "status" | nc -q 2 $MASTER_IP 2323 2>/dev/null)
echo "$result" | grep -q "Heading:"
test_result $? "Master IMU reading"

# Test 4.1: Rudder encoder reading
result=$(echo "status" | nc -q 2 $RUDDER_IP 2323 2>/dev/null)
echo "$result" | grep -q "Raw Angle:"
test_result $? "Rudder encoder reading"

# Test 5.1: Display initialization
result=$(echo "display" | nc -q 2 $UI_IP 2323 2>/dev/null)
echo "$result" | grep -q "Initialized: YES"
test_result $? "Display initialization"

echo ""
echo "=== Test Summary ==="
echo "Passed: $PASS"
echo "Failed: $FAIL"
echo "Total:  $((PASS + FAIL))"
```

---

## Camera Verification Commands

For visual verification, capture and view images:

```bash
# Capture display image
fswebcam -d /dev/video0 -r 1280x720 --no-banner /tmp/display_check.jpg

# View with Claude Code Read tool
# (Use the Read tool on the image path to view it)
```

---

## Troubleshooting

### Node Not Responding
1. Check WiFi connection: `ping <ip>`
2. Check OTA endpoint: `curl -v http://<ip>/update`
3. Reboot node via telnet: `(echo "reboot"; sleep 1) | nc <ip> 2323`

### ESP-NOW Communication Issues
1. Check ESP-NOW status: `(echo "espnow"; sleep 1) | nc <ip> 2323`
2. Verify MAC addresses match Kconfig settings
3. Check WiFi channel consistency

### Display Issues
1. Check display status: `(echo "display"; sleep 1) | nc 192.168.1.186 2323`
2. Force refresh: `(echo "display refresh"; sleep 5) | nc 192.168.1.186 2323`
3. Capture camera image to verify physical display

---

## Version History

| Version | Date | Description |
|---------|------|-------------|
| 1.0.0 | 2026-01-20 | Initial test plan |
