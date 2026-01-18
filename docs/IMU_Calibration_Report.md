# IMU Calibration & Configuration Report for Canoe Autopilot

## Executive Summary

This report analyzes five potential IMU improvements for the TestAP2 canoe autopilot, prioritized by impact on performance and user experience. The recommendations focus on practical, implementable solutions given our ESP32 platform and ICM-20948 sensor.

**Priority Ranking for Canoe Autopilot:**
1. **Improved Mag Cal Workflow** (Highest - User Experience + Accuracy)
2. **Accelerometer Calibration** (High - Tilt Compensation Accuracy)
3. **DMP Sensor Fusion** (Medium-High - Heading Stability)
4. **Gyroscope Calibration** (Medium - Drift Reduction)
5. **Auto-Calibration** (Lower Priority - Requires GPS Heading Reference)

---

## 1. Improved Magnetometer Calibration Workflow

### Current State
Our current implementation supports:
- Hard-iron offset calibration (X, Y, Z in µT)
- Soft-iron 3x3 correction matrix
- Manual entry via console (`magcal set X Y Z`) or BLE
- NVS persistence

**Problem:** Users must calculate calibration values externally and manually enter them. There's no guided calibration procedure.

### Recommended Improvements

#### 1.1 Guided Figure-8 Calibration Procedure
**Implementation Effort:** Medium (2-3 days)

Add an interactive calibration mode that:
1. User enters calibration mode via console (`magcal start`) or BLE command
2. System collects min/max magnetometer values while user rotates device
3. User performs figure-8 motion for 30-60 seconds
4. System automatically computes hard-iron offsets: `offset = (max + min) / 2`
5. System computes soft-iron scale factors: `scale = avg_radius / (max - min) / 2`
6. Display calibration quality metric (sphericity)
7. User confirms and saves

**Code Changes Required:**
```c
// New API functions to add to icm20948.h
esp_err_t icm20948_magcal_start(void);      // Start collecting samples
esp_err_t icm20948_magcal_stop(void);       // Stop and compute calibration
float icm20948_magcal_get_quality(void);    // 0.0-1.0 quality metric
bool icm20948_magcal_is_running(void);      // Check if calibration active
```

**Console Commands:**
```
magcal start    - Begin calibration (rotate device in figure-8)
magcal stop     - Finish and compute calibration
magcal status   - Show current min/max/quality during calibration
magcal save     - Persist calibration to NVS
```

#### 1.2 Visual Feedback During Calibration
**Implementation Effort:** Low (1 day)

For Android app MagCalibrationActivity:
- Real-time 2D plot showing magnetometer X-Y, X-Z, Y-Z
- Visual indication of coverage (which orientations have been sampled)
- Target sphere vs actual ellipsoid visualization
- Quality percentage during calibration

#### 1.3 Calibration Quality Metrics
Add computation of:
- **Sphericity**: How close the calibrated data is to a sphere (ideal = 1.0)
- **Coverage**: What percentage of orientations have been sampled
- **Residual Error**: RMS error after calibration applied

### Expected Impact
- **Accuracy Improvement:** 5-15° heading error reduction (typical)
- **User Experience:** Dramatically improved - no external tools needed
- **Calibration Time:** 30-60 seconds vs hours of manual trial/error

---

## 2. Accelerometer Calibration

### Current State
Our ICM-20948 driver uses factory calibration only:
```c
#define ACCEL_SCALE_2G  (1.0f / 16384.0f)  // ±2g
```

No offset correction is applied. This affects tilt compensation accuracy.

### Why It Matters for Canoe Autopilot
The accelerometer determines roll and pitch for tilt-compensated heading:
```c
float roll = atan2f(ay, az);
float pitch = atanf(-ax / sqrtf(ay*ay + az*az + 1e-9f));
```

If accelerometer has bias, the computed roll/pitch will be wrong, causing heading errors especially when the canoe tilts.

### Recommended Implementation

#### 2.1 Six-Position Calibration
**Implementation Effort:** Medium (2 days)

Standard aerospace calibration method:
1. Place device flat (Z up) - measure gravity on Z axis
2. Place device flat (Z down) - measure gravity on -Z axis
3. Repeat for X and Y axes (4 more positions)
4. Compute scale and offset for each axis

**Simplified Two-Position Method for Canoe:**
Since canoe is always roughly level:
1. Place device level, right-side up
2. Measure for 5 seconds, compute mean accel values
3. Expected: ax≈0, ay≈0, az≈1.0g
4. Offset = measured - expected

**Code Changes:**
```c
// Add to icm20948.h
typedef struct {
    float offset_x, offset_y, offset_z;  // Offset in g
    float scale_x, scale_y, scale_z;     // Scale factors
} icm20948_accel_cal_t;

void icm20948_set_accel_cal(const icm20948_accel_cal_t *cal);
void icm20948_get_accel_cal(icm20948_accel_cal_t *cal);
esp_err_t icm20948_accel_cal_level(void);  // Calibrate assuming level
```

**Console Command:**
```
accelcal level   - Calibrate with device level (simple)
accelcal full    - Full 6-position calibration (advanced)
accelcal show    - Show current calibration values
accelcal save    - Save to NVS
```

#### 2.2 Apply Calibration in Update Loop
Modify `icm20948_update()`:
```c
// Apply accel calibration
float ax = (raw[0] * ACCEL_SCALE_2G - g_accel_offset_x) * g_accel_scale_x;
float ay = (raw[1] * ACCEL_SCALE_2G - g_accel_offset_y) * g_accel_scale_y;
float az = (raw[2] * ACCEL_SCALE_2G - g_accel_offset_z) * g_accel_scale_z;
```

### Expected Impact
- **Tilt Compensation:** 2-5° improvement in heading when tilted
- **Roll/Pitch Accuracy:** ±0.5° vs ±2-3° uncalibrated
- **Critical for Canoe:** Canoes tilt significantly, making this important

---

## 3. DMP (Digital Motion Processor) Sensor Fusion

### Current State
We implement tilt-compensated heading in software:
```c
static float compute_tilt_compensated_heading(
    float ax, float ay, float az,
    float mx, float my, float mz)
```

This is a basic algorithm without sensor fusion.

### What DMP Provides
The ICM-20948's onboard DMP offers:
- **9-axis sensor fusion** combining accel, gyro, and mag
- **Quaternion output** for orientation
- **Run-time calibration** of all sensors
- **Reduced CPU load** on ESP32
- **14KB firmware** loaded to ICM-20948's FPGA

### Available DMP Outputs
| Output | Description |
|--------|-------------|
| Quaternion (6-axis) | Orientation without magnetometer (no drift correction) |
| Quaternion (9-axis) | Full orientation with magnetic north reference |
| Game Rotation Vector | For gaming/AR without absolute reference |
| Geomagnetic Rotation Vector | Mag-only orientation |
| Euler Angles | Roll, Pitch, Yaw derived from quaternions |
| Linear Acceleration | Gravity removed |
| Gravity Vector | Direction of gravity |
| Step Counter | Pedometer (not relevant for canoe) |

### Implementation Options

#### Option A: Use Existing ESP-IDF DMP Library
**Implementation Effort:** Medium-High (3-5 days)

Use [cybergear-robotics/icm20948](https://github.com/cybergear-robotics/icm20948):
- Pre-ported SparkFun library for ESP-IDF
- Includes DMP support via menuconfig
- Would require replacing our current driver

**Pros:**
- Proven implementation
- Full DMP support
- Active maintenance

**Cons:**
- Significant code change
- Different API than our current driver
- 14KB additional flash for DMP firmware

#### Option B: Add DMP to Existing Driver
**Implementation Effort:** High (1-2 weeks)

Port DMP functionality to our existing driver:
- Load DMP firmware blob
- Configure FIFO for DMP output
- Parse quaternion/euler data

**Pros:**
- Maintains our existing API
- Can enable/disable DMP at runtime

**Cons:**
- Significant development effort
- Risk of bugs in port

#### Option C: Implement Software Sensor Fusion
**Implementation Effort:** Medium (3-4 days)

Add complementary or Madgwick filter to our existing driver:
```c
// Madgwick AHRS filter
void MadgwickAHRSupdate(float gx, float gy, float gz,
                         float ax, float ay, float az,
                         float mx, float my, float mz);
float getMadgwickYaw(void);
```

**Pros:**
- No hardware dependency
- Well-understood algorithms
- Gyro integration prevents magnetometer noise
- Configurable filter parameters

**Cons:**
- Uses ESP32 CPU cycles
- Must tune filter parameters

### Recommendation for Canoe Autopilot
**Option C (Software Sensor Fusion)** is recommended because:
1. Minimal code change required
2. Madgwick/Complementary filters are proven for marine applications
3. Gyro integration smooths heading during magnetic interference
4. Can tune filter responsiveness for canoe dynamics

### Expected Impact
- **Heading Stability:** Significantly improved during maneuvers
- **Noise Rejection:** Gyro integration filters magnetometer noise
- **Lag Reduction:** Faster response than pure magnetometer
- **Magnetic Interference:** Better handling of temporary disturbances

---

## 4. Gyroscope Calibration

### Current State
We use factory calibration only:
```c
#define GYRO_SCALE_250DPS  (1.0f / 131.0f)  // ±250°/s
```

No bias compensation applied. Gyro bias causes:
- Yaw rate offset when stationary
- Drift in sensor fusion algorithms

### Why It Matters
Gyroscope bias is the single biggest source of heading drift in sensor fusion. Even a 0.1°/s bias causes 6° drift per minute.

### Recommended Implementation

#### 4.1 Boot-time Gyro Calibration
**Implementation Effort:** Low (0.5 days)

At startup, while device is stationary:
1. Collect gyro samples for 2-3 seconds
2. Compute mean (this is the bias)
3. Subtract bias from all future readings

**Code Changes:**
```c
// Add to icm20948_init() or new function
esp_err_t icm20948_calibrate_gyro(void) {
    // Collect 100 samples over 1 second
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    for (int i = 0; i < 100; i++) {
        icm20948_update();
        sum_gx += g_data.gyro_x;
        sum_gy += g_data.gyro_y;
        sum_gz += g_data.gyro_z;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    g_gyro_bias_x = sum_gx / 100.0f;
    g_gyro_bias_y = sum_gy / 100.0f;
    g_gyro_bias_z = sum_gz / 100.0f;
    ESP_LOGI(TAG, "Gyro bias: [%.3f, %.3f, %.3f] deg/s",
             g_gyro_bias_x, g_gyro_bias_y, g_gyro_bias_z);
    return ESP_OK;
}
```

#### 4.2 Runtime Bias Estimation
More advanced: continuously estimate bias when canoe is stable (low acceleration variance).

### Expected Impact
- **Yaw Rate Accuracy:** ±0.01°/s vs ±0.5°/s uncalibrated
- **Sensor Fusion:** Essential for drift-free heading
- **PID Control:** More accurate derivative term (uses yaw rate)

---

## 5. Auto-Calibration

### Current State
No automatic calibration. User must manually calibrate magnetometer.

### Approaches Researched

#### 5.1 GPS-Aided Magnetometer Calibration
**Implementation Effort:** High (1-2 weeks)

Use GNSS COG (Course Over Ground) as reference heading when:
- Speed > 3 knots (COG is reliable)
- Heading error vs COG exceeds threshold
- Adaptively update mag calibration

Based on research: "Two different online compass calibration methods... based on the parameter adaptation algorithm and... functional learning algorithm... using GPS heading measurements as reference signals."

**Pros:**
- Continuous improvement of calibration
- Adapts to environmental changes
- No user intervention needed

**Cons:**
- Requires GPS fix and movement
- Complex algorithm
- Risk of incorrect calibration if GPS heading is wrong

#### 5.2 Ellipsoid Fitting Auto-Calibration
Run ellipsoid fitting algorithm in background:
- Collect magnetometer samples during normal operation
- When sufficient coverage achieved, compute new calibration
- Apply gradually to avoid sudden heading jumps

#### 5.3 DMP Background Calibration
The ICM-20948 DMP includes:
"ultra-low power run-time and background calibration of the accelerometer, gyroscope, and compass"

This happens automatically when DMP is enabled.

### Recommendation for Canoe Autopilot
**Lower Priority** because:
1. Canoe use case is short trips (hours, not days)
2. Manual calibration before trip is acceptable
3. GPS-aided calibration requires reliable COG (need speed)
4. Risk of auto-calibration going wrong during use

**Consider Later:** Once DMP is implemented, the background calibration comes "for free."

---

## Implementation Roadmap

### Phase 1: Quick Wins (1-2 days)
1. **Gyroscope calibration at boot** - Low effort, immediate benefit
2. **Simple accelerometer level calibration** - Low effort, improves tilt compensation

### Phase 2: User Experience (3-5 days)
3. **Guided magnetometer calibration workflow** - Medium effort, major UX improvement
4. **Android app calibration UI enhancements** - Builds on #3

### Phase 3: Heading Quality (1 week)
5. **Madgwick/Complementary sensor fusion filter** - Medium effort, significant heading improvement
6. **Calibration quality metrics and feedback** - Helps users achieve good calibration

### Phase 4: Advanced (Future)
7. **DMP integration** - High effort, full sensor fusion
8. **GPS-aided auto-calibration** - High effort, hands-off operation

---

## Specific Code Changes Summary

| Feature | Files to Modify | New Parameters | Effort |
|---------|-----------------|----------------|--------|
| Gyro cal at boot | icm20948.c | 3 floats (bias) | 0.5 days |
| Accel cal | icm20948.c/h, param_store | 6 floats (offset+scale) | 2 days |
| Guided mag cal | icm20948.c/h, cmd_console.c | None (computed) | 3 days |
| Madgwick filter | New file: madgwick.c/h | Filter beta | 3 days |
| Android mag cal UI | MagCalibrationActivity.kt | None | 2 days |
| DMP integration | Major refactor or new lib | Many | 1-2 weeks |

---

## Conclusion

For the TestAP2 canoe autopilot, the highest-impact improvements are:

1. **Guided mag calibration workflow** - Users currently struggle with calibration
2. **Accelerometer calibration** - Critical for accurate tilt compensation in a tilting canoe
3. **Software sensor fusion** - Significantly improves heading stability

These three improvements can be implemented in approximately 1-2 weeks and will dramatically improve both accuracy and user experience.

The DMP and auto-calibration features are valuable but lower priority due to implementation complexity and the short-trip nature of canoe use.

---

## References

- [SparkFun ICM-20948 DMP Documentation](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/blob/main/DMP.md)
- [ESP-IDF ICM-20948 Library with DMP](https://github.com/cybergear-robotics/icm20948)
- [ICM-20948 Calibration Discussion](https://wolles-elektronikkiste.de/en/icm-20948-9-axis-sensor-part-i)
- [Real-Time Magnetometer Calibration (MDPI)](https://www.mdpi.com/1424-8220/20/2/535)
- [Adaptive Compass Calibration (ResearchGate)](https://www.researchgate.net/publication/245336114_Adaptive_and_learning_calibration_of_magnetic_compass)
- [PX4 Compass Calibration Guide](https://docs.px4.io/main/en/config/compass.html)
- [ArduPilot Mounting Guide](https://ardupilot.org/copter/docs/common-mounting-the-flight-controller.html)
- [IMU Vibration Best Practices (Hexagon)](https://hexagondownloads.blob.core.windows.net/public/Novatel/assets/Documents/Papers/APN-112-Inertial-Navigation-Systems-and-Vibration/APN-112-Inertial-Navigation-Systems-and-Vibration.pdf)
