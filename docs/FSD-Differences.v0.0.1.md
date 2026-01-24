# TestAPEN FSD Differences Document

## Document Information

| Field | Value |
|-------|-------|
| Document Version | v0.0.1 |
| FSD Version Compared | v1.1.0 |
| Codebase Commit | ef2c9af (2026-01-24) |
| Firmware Version | 1.1.0-espnow |
| Created | 2026-01-24 |
| Author | Claude Code (automated analysis) |

---

## Purpose

This document identifies differences between the TestAPEN FSD v1.1.0 and the current codebase implementation. Changes listed here should be incorporated into the next FSD revision to ensure the FSD accurately reflects the implemented system.

---

## 1. Hardware Configuration Differences

### 1.1 Motor GPIO Pin Assignment (CRITICAL)

**FSD Section 4.2 & 4.5 - Rudder Node GPIO:**

| GPIO | FSD Specification | Actual Code | Reason |
|------|-------------------|-------------|--------|
| 25 | MOTOR_PWM | MOTOR_DIR | Hardware rebuilt with swapped pins |
| 26 | MOTOR_DIR | MOTOR_PWM | Hardware rebuilt with swapped pins |

**Required FSD Change:**
Update Section 4.2 and 4.5 to:
```
├── PWM+GPIO (PWM=26, DIR=25)
│   └── Cytron MD13S motor driver
```

And GPIO table:
| GPIO | Function |
|------|----------|
| 25 | MOTOR_DIR |
| 26 | MOTOR_PWM |

**Code Location:** `main/Kconfig.projbuild` lines 101-110

---

### 1.2 E-Paper GPIO Pins

**FSD Section 4.3 lists:**
```
├── RST:  GPIO 16
├── DC:   GPIO 17
```

**Actual Kconfig defaults:**
```
├── RST:  GPIO 27
├── DC:   GPIO 19
├── CS:   GPIO 17
```

**Required FSD Change:** Update Section 4.3 E-Paper pin assignments to match Kconfig:

| GPIO | Function |
|------|----------|
| 4 | EPAPER_BUSY |
| 17 | EPAPER_CS |
| 18 | EPAPER_CLK |
| 19 | EPAPER_DC |
| 23 | EPAPER_DIN |
| 27 | EPAPER_RST |

**Code Location:** `main/Kconfig.projbuild` lines 123-153

---

## 2. Task Configuration Differences

### 2.1 Task_Network Added to All Nodes

**FSD Sections 6.2-6.4** do not document Task_Network, but it exists on all nodes for telnet console handling.

**Required FSD Change:** Add to each node's task table:

| Task | Pri | Rate | Stack | Purpose |
|------|-----|------|-------|---------|
| Task_Network | 1 | 10Hz | 4096 | Telnet console handler |

**Code Locations:**
- `main/master_node.c` line 879
- `main/rudder_node.c` line 919
- `main/ui_node.c` line 878

---

### 2.2 Task_BLE Not Implemented as Separate Task

**FSD Section 6.2** lists Task_BLE as a separate task.

**Actual Implementation:** BLE runs within the NimBLE host task (managed by ESP-IDF), not as a custom application task.

**Required FSD Change:** Remove Task_BLE from Master Node task table or note that BLE is handled by NimBLE host stack, not a custom task.

---

### 2.3 Rudder Node Has Task_Display

**FSD Section 6.3** does not list Task_Display for Rudder node.

**Actual Implementation:** Rudder node has SSD1306 OLED display and Task_Display.

**Required FSD Change:** Add to Rudder Node tasks:

| Task | Pri | Rate | Stack |
|------|-----|------|-------|
| Task_Display | 2 | 5Hz | 4096 |

**Code Location:** `main/rudder_node.c` line 922

---

## 3. Resilient Initialization (NEW FEATURE)

### 3.1 Rudder Node Initialization Order

**Not documented in FSD.** The Rudder node now uses resilient initialization to ensure OTA and telnet are always available even when sensors fail.

**Required FSD Addition:** Add new section or update Section 6.3:

**Rudder Node Initialization Order:**
1. I2C + Display (for boot status)
2. State Machine
3. Parameter Store
4. **Network/WiFi + Console** (critical for OTA/debug)
5. **ESP-NOW** (before sensors)
6. **Motor Driver** (before sensors)
7. **AS5600 Encoder** (last - failures don't block debug)
8. Summary Screen
9. Create Tasks

**Key Behavior:**
- Sensor failures set FAULTED state but do NOT prevent network initialization
- OTA and telnet always available for recovery
- Motor disabled when sensor unavailable

**Code Location:** `main/rudder_node.c` lines 785-931

---

## 4. Console Commands Differences

### 4.1 Master Node Additional Commands

**FSD Section 14.2** lists basic commands. The following are implemented but not documented:

| Command | Description |
|---------|-------------|
| `set heading N` | Set target heading directly |
| `adjust N` | Adjust heading by ±N degrees |
| `magcal start/stop/status/save/get/set` | Magnetometer calibration suite |
| `accelcal` | Accelerometer level calibration |
| `gyrocal` | Gyroscope bias calibration |
| `fusion on/off/beta N` | Madgwick filter control |
| `gpsoffset apply/reset` | GPS heading offset |
| `gnss` | GNSS status display |

**Code Location:** `components/cmd_console/cmd_console.c`

---

### 4.2 Rudder Node Additional Commands

| Command | Description |
|---------|-------------|
| `rudder` | Rudder angle and status |
| `servo [Kp db_in db_out]` | Servo parameters |
| `set rudder N` | Manual rudder command |

---

### 4.3 UI Node Commands (NEW)

**FSD Section 14.2** does not document UI node commands:

| Command | Description |
|---------|-------------|
| `status` | Node status |
| `nodes` | Connected node status |
| `buttons` | Button state |
| `display` | Display status |
| `display refresh` | Force display refresh |
| `page [0-2]` | Set display page |
| `btn list` | List button mappings |
| `btn N` | Simulate button press |
| `btn N long` | Simulate long press |

---

## 5. BLE Interface Differences

### 5.1 Magnetometer Calibration Commands

**FSD Section 11.2** does not document magnetometer calibration BLE commands.

**Required FSD Addition:**

| Code | Command | Description |
|------|---------|-------------|
| 0x60 | MAGCAL_GET | Read current calibration |
| 0x61 | MAGCAL_SET | Set calibration values |
| 0x62 | MAGCAL_SAVE | Save calibration to NVS |
| 0x63 | MAGCAL_RESET | Reset to defaults |

**Data format:** 49 bytes (1 cmd + 12 hard-iron floats + 36 soft-iron matrix bytes)

**Code Location:** `components/ble_manager/include/ble_manager.h` lines 60-66

---

### 5.2 Mag Calibration Characteristic

**FSD Section 11.1** lists characteristic UUID ...9006 but description is minimal.

**Required FSD Addition:** Expand Section 11.1:

| Characteristic | UUID suffix | Properties | Data Format |
|----------------|-------------|------------|-------------|
| Mag Calibration | ...9006 | Read, Write, Notify | 49 bytes: cmd(1) + hard_iron(12) + soft_iron(36) |

---

## 6. Parameter Store Differences

### 6.1 Additional Parameters Not in FSD

**FSD Appendix C** lists 9 control parameters. The following are implemented but not documented:

**Magnetometer Calibration (12 params):**
| ID | Name | Default |
|----|------|---------|
| PARAM_MAG_HARD_X | Hard iron X offset | 0.0 |
| PARAM_MAG_HARD_Y | Hard iron Y offset | 0.0 |
| PARAM_MAG_HARD_Z | Hard iron Z offset | 0.0 |
| PARAM_MAG_SOFT_00-22 | Soft iron 3x3 matrix | Identity |

**Rudder Calibration (1 param):**
| ID | Name | Default |
|----|------|---------|
| PARAM_CAL_RAW_CENTER | Raw encoder center position | -1.0 (uncalibrated) |

**Code Location:** `components/param_store/param_store.c`

---

## 7. Error Codes Differences

### 7.1 ESP-NOW Specific Error Code

**FSD Section 5.6** should include:

| Code | Name | Severity | Description |
|------|------|----------|-------------|
| 0x03 | ERR_ESPNOW_INIT_FAIL | Critical | ESP-NOW initialization failed |

**Code Location:** `components/espnow_protocol/include/espnow_protocol.h` line 76

---

## 8. Display Content Differences

### 8.1 UI Node Page 0 Format

**FSD Section 13.2** shows:
```
State: IDLE Err:+0.1
```

**Actual Implementation:**
```
<STATE> <MODE> Err:<ERROR>
```
Where MODE is "HDG" or "COG" based on FLAG_COG_MODE.

---

## 9. Minor Documentation Updates

### 9.1 OTA Command in Appendix B

**FSD Appendix B** shows:
```bash
curl -X POST --data-binary @build/testap2.bin \
```

**Should be:**
```bash
curl -X POST --data-binary @build/testapen.bin \
```

---

### 9.2 Version Reference

**FSD Header** shows version 1.1.0 but code references may vary.

All node firmware versions: `1.1.0-espnow`

---

## Summary of Required FSD Updates

| Priority | Section | Change |
|----------|---------|--------|
| HIGH | 4.2, 4.5 | Swap Motor PWM/DIR GPIO pins |
| HIGH | 4.3 | Update E-Paper GPIO pins |
| MEDIUM | 6.2-6.4 | Add Task_Network to all nodes |
| MEDIUM | 6.3 | Add Task_Display to Rudder node |
| MEDIUM | NEW | Add Resilient Initialization section |
| LOW | 14.2 | Document additional console commands |
| LOW | 11.1, 11.2 | Document magnetometer BLE commands |
| LOW | Appendix C | Add magnetometer calibration params |
| LOW | Appendix B | Fix OTA binary filename |

---

## Verification

This document was generated by automated comparison of:
- FSD: `/home/cas/TestAPEN/docs/TestAPEN.FSD.v1.1.0.md`
- Codebase: `/home/cas/TestAPEN/` at commit ef2c9af

To regenerate, run the Code.Compare.To.FSD instructions with Claude Code.

---

**End of Differences Document v0.0.1**
