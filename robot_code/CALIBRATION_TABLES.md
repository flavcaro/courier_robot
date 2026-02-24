# 🤖 Courier Robot - Calibration Tables & Parameters

## 📋 Table of Contents
- [Motor & Track Calibration](#motor--track-calibration)
- [Robot Velocity Calibration](#robot-velocity-calibration)
- [Rotation Calibration](#rotation-calibration)
- [IMU Sensor Calibration](#imu-sensor-calibration)
- [Ultrasonic Sensor Calibration](#ultrasonic-sensor-calibration)
- [Calibration Procedures](#calibration-procedures)

---

## 🚗 Motor & Track Calibration

### Linear Movement Compensation Factors
Compensates for track asymmetry and motor power differences during straight-line movement.

| Parameter | Value | Unit | Description |
|-----------|-------|------|-------------|
| **Left Motor Factor** | 1.34 | multiplier | Left track speed multiplier (compensates for weaker left track) |
| **Right Motor Factor** | 1.00 | multiplier | Right track speed multiplier (baseline) |
| **Base Linear Speed** | 0.60 | ratio (0-1) | Default forward/backward speed (60% max power) |
| **Base Turn Speed** | 0.75 | ratio (0-1) | Default rotation speed (75% max power) |

**📊 Effect**: With these factors, the robot moves straight instead of veering left/right.

### Rotation Compensation Factors
Compensates for differential track behavior during rotation (one track forward, one reverse).

| Parameter | Value | Unit | Description |
|-----------|-------|------|-------------|
| **Rotation Left Factor** | 1.50 | multiplier | Base power for left track during rotation |
| **Rotation Right Factor** | 1.00 | multiplier | Base power for right track during rotation |
| **Forward Boost** | 1.00 | multiplier | Boost for track moving forward during rotation |
| **Reverse Boost** | 1.30 | multiplier | Boost for track moving backward (compensates for higher friction) |

**📊 Effect**: Accounts for asymmetric friction - reverse track experiences more resistance.

### Motor Power Levels

| Movement Type | Speed Setting | Effective Power (Left/Right) | PWM Approximate |
|---------------|---------------|-------------------------------|-----------------|
| **Linear Forward (ECO)** | 60% base | 80% / 60% | ~204 / ~153 |
| **Linear Full** | 100% base | 134% / 100% | ~342 / ~255 |
| **Rotation (base)** | 75% turn | 112% / 75% | ~286 / ~191 |
| **Rotation (boosted)** | 75% turn | 97% / 97% | ~248 / ~248 |

**Note**: PWM values are calculated as `255 × speed × factor`

---

## 🏃 Robot Velocity Calibration

### Forward Movement Speed
| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| **Forward Speed** | 0.044 | m/s | ~4.4 cm/s at 60% power (ECO mode) |
| **Lateral Speed** | 0.044 | m/s | ~4.4 cm/s for sideways movement |
| **Time per Meter** | 22.7 | seconds | At default 60% power |

**Calibration Method**: Measure time to travel exactly 1 meter, then calculate: `speed = distance / time`

### Speed vs Power Table (Estimated)

| Power Level | Speed (m/s) | Speed (cm/s) | Time for 1m | Notes |
|-------------|-------------|--------------|-------------|-------|
| 30% | ~0.022 | ~2.2 | ~45s | Very slow, high precision |
| 40% | ~0.029 | ~2.9 | ~34s | Slow, good control |
| 50% | ~0.037 | ~3.7 | ~27s | Moderate |
| **60% (ECO)** | **0.044** | **4.4** | **23s** | **Default - Battery optimized** |
| 75% | ~0.055 | ~5.5 | ~18s | Fast |
| 100% | ~0.073 | ~7.3 | ~14s | Maximum speed |

**⚡ Battery Impact**: 60% power provides best balance of speed vs battery life.

---

## 🔄 Rotation Calibration

### 90° Rotation Times
| Rotation Direction | Time (s) | Power | Accuracy | Notes |
|--------------------|----------|-------|----------|-------|
| **Right** | 2.10 | 75% | ±2° | Calibrated - perfect 90° |
| **Left** | 1.99 | 75% | ±2° | Slightly faster (was overshooting to 95°) |
| **180° Right** | 4.20 | 75% | ±3° | 2× 90° rotation |
| **180° Left** | 3.98 | 75% | ±3° | 2× 90° rotation |

### Rotation Speed vs Power

| Power Level | Degrees/Second | 90° Time (s) | 180° Time (s) | 360° Time (s) |
|-------------|----------------|--------------|---------------|---------------|
| 50% | ~30 | 3.0 | 6.0 | 12.0 |
| 60% | ~36 | 2.5 | 5.0 | 10.0 |
| **75% (Default)** | **~43** | **2.1** | **4.2** | **8.4** |
| 100% | ~57 | 1.6 | 3.2 | 6.4 |

**📐 Calibration Status**:
- ✅ Right rotation: Accurate at 2.10s
- ✅ Left rotation: Accurate at 1.99s (reduced from 2.10s due to overshoot)

---

## 🧭 IMU Sensor Calibration (MPU6050)

### Static Calibration Offsets
Measured with robot stationary on flat surface (300 samples).

| Sensor | Axis | Offset Value | Unit | Status |
|--------|------|--------------|------|--------|
| **Gyroscope** | Z (Yaw) | TBD | °/s | ⚠️ Calibrate on startup |
| **Accelerometer** | X | TBD | g | ⚠️ Calibrate on startup |
| **Accelerometer** | Y | TBD | g | ⚠️ Calibrate on startup |

**🔧 How to Calibrate**: Run `test_imu_calibration.py` with robot stationary and level.

### Gyroscope Drift Characteristics

| Duration | Expected Drift | Acceptable Range | Status |
|----------|----------------|------------------|--------|
| 10 seconds | < 2° | 0-5° | ✅ Acceptable |
| 30 seconds | < 5° | 0-10° | ✅ Acceptable |
| 60 seconds | < 10° | 0-20° | ⚠️ Monitor |

**💡 Drift Compensation**: IMU heading is continuously corrected during movement using complementary filter.

### IMU Integration Parameters

| Parameter | Value | Unit | Description |
|-----------|-------|------|-------------|
| **Alpha Filter** | 0.98 | ratio | Complementary filter weight (gyro vs accel) |
| **Heading Correction Gain** | 0.25 | multiplier | Aggressiveness of heading correction during movement |
| **Update Rate** | 20 | Hz | IMU reading frequency during movement (every 50ms) |
| **Odometry Scale Factor** | 0.60 | multiplier | Correction for IMU odometry (accelerometer overestimates ~40%) |

### IMU Feature Flags

| Feature | Enabled | Status | Description |
|---------|---------|--------|-------------|
| **Use IMU Rotation** | ❌ No | Testing | Closed-loop rotation control with IMU feedback |
| **Use IMU Odometry** | ✅ Yes | Active | Distance measurement via accelerometer integration |
| **Use Heading Correction** | ✅ Yes | Active | Continuous drift correction during movement |
| **Heading Debug** | ✅ Yes | Active | Prints correction messages |

---

## 📡 Ultrasonic Sensor Calibration

### Distance Measurement Characteristics

| Range | Accuracy | Reliability | Use Case |
|-------|----------|-------------|----------|
| 0-10 cm | ±3 cm | Low | Too close - avoid |
| 10-30 cm | ±2 cm | High | Obstacle detection during movement |
| 30-100 cm | ±3 cm | High | Direction scanning |
| 100-200 cm | ±5 cm | Medium | Long-range detection |
| 200-400 cm | ±10 cm | Low | Maximum range, less reliable |

### Threshold Configuration

| Threshold Type | Distance (cm) | Purpose | Context |
|----------------|---------------|---------|---------|
| **Obstacle Threshold (Dynamic)** | 15 | Emergency stop during movement | Continuous monitoring while moving |
| **Obstacle Threshold (Static)** | 30 | Safe distance for stopped robot | Scanning while stationary |
| **Direction Clearance** | 40 | Minimum space to consider direction "free" | Path planning decisions |
| **Lateral Clearance** | 35 | Side clearance for turning | Verifying turn space |
| **Bypass Verification** | 35 | Confirmation after obstacle avoidance | Post-maneuver check |

**⚠️ Important**: Dynamic threshold (15cm) is lower because:
1. Robot checks every 100ms while moving
2. Movement inertia requires safety margin
3. Prevents collision during deceleration

### Detection Zone Table

| Zone | Distance Range | Alert Level | Action |
|------|----------------|-------------|--------|
| **Critical** | 0-15 cm | 🔴 Stop | Immediate motor stop |
| **Warning** | 15-30 cm | 🟡 Caution | Prepare to stop or turn |
| **Safe** | 30-50 cm | 🟢 OK | Continue with monitoring |
| **Clear** | 50-100 cm | ✅ Free | Normal operation |
| **Far** | 100+ cm | ⚪ Ignore | No immediate concern |

### Sensor Response Time

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| **Measurement Time** | ~30 | ms | Single ultrasonic ping |
| **Update Rate (Moving)** | 10 | Hz | Check every 100ms during movement |
| **Update Rate (Scanning)** | 5 | Hz | Check every 200ms during direction scan |
| **Settling Time** | 50 | ms | Time to stabilize after rotation |

---

## 🔧 Calibration Procedures

### 1️⃣ Motor & Track Calibration

**Test File**: `test_calibration.py`

**Procedure**:
1. Place robot on smooth, flat surface
2. Align with straight reference line (tape on floor)
3. Run test with current compensation factors
4. Observe drift direction:
   - Drifts LEFT → Increase `left_factor`
   - Drifts RIGHT → Increase `right_factor`
5. Adjust in increments of 0.05
6. Repeat until robot travels straight

**Parameter Location**: `bt/simple_state.py`
```python
self.left_factor = 1.34   # Adjust this
self.right_factor = 1.00  # Keep as baseline
```

---

### 2️⃣ Velocity Calibration

**Test File**: `test_imu_odometry.py` or manual measurement

**Procedure**:
1. Measure and mark exactly 1.0 meter on floor
2. Position robot at start line
3. Run forward movement for the measured distance
4. Time how long it takes
5. Calculate: `speed = 1.0 / time_in_seconds`
6. Update calibration

**Parameter Location**: `bt/simple_state.py`
```python
self.meters_per_second_forward = 0.044  # Update with measured value
```

**Example**:
- If robot takes 25 seconds for 1 meter: `speed = 1.0 / 25 = 0.04 m/s`

---

### 3️⃣ Rotation Calibration

**Test File**: `test_rotation_calibrate.py`

**Procedure**:
1. Place alignment mark on floor (straight line)
2. Align robot with mark
3. Run rotation test with incremental times
4. Visually measure rotation angle
5. Find time that gives exactly 90°
6. Update both left and right rotation times separately

**Parameter Location**: `bt/simple_state.py`
```python
self.rotation_90_time_left = 1.99   # Left rotation time
self.rotation_90_time_right = 2.10  # Right rotation time
```

**Verification**:
- Full 360° rotation should take `4 × rotation_90_time` seconds
- Use protractor or marked floor angles for accuracy

---

### 4️⃣ IMU Calibration

**Test Files**: 
- `test_imu_calibration.py` - Basic calibration and drift test
- `test_imu_rotation.py` - Rotation accuracy test
- `test_imu_odometry.py` - Distance measurement test

**Procedure**:

**Phase 1 - Static Calibration** (300 samples, ~3 seconds):
1. Place robot on flat, level surface
2. Ensure robot is completely stationary
3. Run calibration to measure sensor offsets
4. Record gyro and accelerometer offsets

**Phase 2 - Drift Test** (30 seconds):
1. Keep robot stationary
2. Monitor heading drift
3. Acceptable: < 5° drift in 30 seconds
4. If drift > 10°: Recalibrate or check sensor mounting

**Phase 3 - Rotation Test**:
1. Manually rotate robot exactly 90° (use protractor)
2. Compare IMU reading vs actual angle
3. Error should be < 5°

**Phase 4 - Movement Test**:
1. Move robot 1 meter
2. Compare IMU distance vs actual distance
3. Adjust `imu_odometry_scale` if needed

**Parameter Location**: `bt/simple_state.py`
```python
self.imu_odometry_scale = 0.60  # Adjust based on test results
self.imu_heading_correction_gain = 0.25  # Adjust for drift correction aggressiveness
```

---

### 5️⃣ Ultrasonic Sensor Calibration

**Test Method**: Manual measurement with ruler/tape measure

**Procedure**:
1. Place flat surface (wall, board) at known distances
2. Read sensor value at each distance
3. Compare sensor reading vs actual distance
4. Create distance correction table if needed

**Accuracy Test Distances**:
- 10 cm
- 20 cm
- 30 cm
- 50 cm
- 100 cm

**Threshold Tuning**:
1. Test obstacle detection while moving at default speed
2. Measure typical stopping distance
3. Set `obstacle_threshold` to: stopping_distance + 5cm safety margin
4. For scanning, use larger threshold (30-40cm) for comfortable clearance

**Parameter Location**: `bt/simple_state.py`
```python
self.obstacle_threshold = 15.0  # Dynamic (while moving)
self.obstacle_threshold_static = 30.0  # Static (scanning)
self.direction_clearance_threshold = 40.0  # Path planning
```

---

## 📊 Calibration Checklist

Use this checklist before running missions:

- [ ] **Motor Balance**: Robot travels straight for 3+ meters
- [ ] **Rotation Left**: 360° rotation completes in ~8 seconds (4×90°)
- [ ] **Rotation Right**: 360° rotation completes in ~8.4 seconds (4×90°)
- [ ] **IMU Drift**: < 5° drift in 30 seconds when stationary
- [ ] **IMU Rotation**: 90° rotation error < 5° (if IMU rotation enabled)
- [ ] **Velocity**: Measured speed matches expected (±10%)
- [ ] **Ultrasonic**: Accurate readings from 10-100cm (±5cm)
- [ ] **Obstacle Detection**: Robot stops before hitting wall at full speed

---

## 💾 Quick Reference - Current Calibration Values

```python
# MOVEMENT
left_factor = 1.34
right_factor = 1.00
meters_per_second_forward = 0.044  # m/s
base_speed_linear = 0.60  # 60%
base_speed_turn = 0.75    # 75%

# ROTATION
rotation_90_time_left = 1.99   # seconds
rotation_90_time_right = 2.10  # seconds
rotation_left_factor = 1.50
rotation_right_factor = 1.00
rotation_reverse_boost = 1.30
rotation_forward_boost = 1.00

# ULTRASONIC
obstacle_threshold = 15.0           # cm (moving)
obstacle_threshold_static = 30.0    # cm (stationary)
direction_clearance_threshold = 40.0 # cm (path planning)

# IMU
use_imu_odometry = True
use_imu_heading_correction = True
imu_heading_correction_gain = 0.25
imu_odometry_scale = 0.60
alpha_filter = 0.98
```

---

## 📝 Notes

- All calibration values are stored in `bt/simple_state.py`
- Motor power values are in `bt/actions.py`
- Test scripts are in `robot_code/` directory
- Recalibrate after any hardware changes (battery level, track wear, weight changes)
- Surface type affects calibration (smooth floor vs carpet)

**Last Updated**: February 24, 2026
