# 🤖 CALIBRATION QUICK REFERENCE CARD

## 📍 Current Values (Quick Copy-Paste)

```python
# bt/simple_state.py - MOVEMENT
self.meters_per_second_forward = 0.044
self.rotation_90_time_left = 1.99
self.rotation_90_time_right = 2.10

# bt/simple_state.py - MOTOR COMPENSATION
self.left_factor = 1.34
self.right_factor = 1.00

# bt/simple_state.py - ROTATION
self.rotation_left_factor = 1.50
self.rotation_right_factor = 1.00
self.rotation_reverse_boost = 1.30
self.rotation_forward_boost = 1.00

# bt/simple_state.py - ULTRASONIC
self.obstacle_threshold = 15.0
self.obstacle_threshold_static = 30.0
self.direction_clearance_threshold = 40.0

# bt/simple_state.py - IMU
self.use_imu_odometry = True
self.use_imu_heading_correction = True
self.imu_heading_correction_gain = 0.25
self.imu_odometry_scale = 0.60

# bt/actions.py - SPEEDS
DEFAULT_SPEED_LINEAR = 0.60  # 60%
DEFAULT_SPEED_TURN = 0.75    # 75%
```

---

## 🔧 Calibration Commands (Quick Start)

### 1️⃣ Motor Balance Test
```bash
python3 test_calibration.py
```
**What to look for**: Robot should go straight for 3+ meters
**If drifts LEFT**: Increase `left_factor` by 0.05
**If drifts RIGHT**: Increase `right_factor` by 0.05 (or decrease `left_factor`)

### 2️⃣ Rotation Timing Test
```bash
python3 test_rotation_calibrate.py
```
**Goal**: Find exact time for 90° rotation
**Current values**: Left=1.99s, Right=2.10s at 75% power
**Adjust**: Increment by 0.1s until exactly 90°

### 3️⃣ IMU Calibration
```bash
python3 test_imu_calibration.py
```
**Requirements**: Robot STATIONARY on FLAT surface
**Phases**: Static cal → Drift test (30s) → Manual rotation test
**Acceptable drift**: < 5° in 30 seconds

### 4️⃣ Velocity Measurement
```bash
python3 test_imu_odometry.py
# OR manually: measure time to travel 1 meter
```
**Formula**: `velocity = distance / time = 1.0 / seconds`
**Example**: 23 seconds for 1m → 0.044 m/s

### 5️⃣ Ultrasonic Test
```bash
# Manual test with ruler
python3 -c "from bt.actions import rover; import time; 
while True: print(f'{rover.getUltrasonicSensor():.1f} cm'); time.sleep(0.5)"
```
**Test at**: 10cm, 20cm, 30cm, 50cm, 100cm
**Adjust thresholds** if readings consistently off

---

## 🎯 Troubleshooting Guide

| Problem | Likely Cause | Solution |
|---------|-------------|----------|
| Robot drifts left | Left track weaker | Increase `left_factor` |
| Robot drifts right | Right track weaker | Increase `right_factor` |
| Rotation < 90° | Time too short | Increase `rotation_90_time` |
| Rotation > 90° | Time too long | Decrease `rotation_90_time` |
| Left rotation ≠ Right | Track asymmetry | Calibrate separately |
| IMU drift > 10° | Bad calibration | Recalibrate IMU stationary |
| Stops too early | Threshold too high | Decrease `obstacle_threshold` |
| Hits obstacles | Threshold too low | Increase `obstacle_threshold` |
| IMU odometry off | Scale factor wrong | Adjust `imu_odometry_scale` |

---

## 📊 Calibration Checklist

Before mission:
- [ ] Robot goes straight (3m test)
- [ ] 360° rotation = ~8 seconds (4×90°)
- [ ] IMU drift < 5° in 30s
- [ ] Ultrasonic accurate ±5cm (10-100cm)
- [ ] Stops before hitting wall

After hardware change:
- [ ] Recalibrate motor balance
- [ ] Verify rotation timing
- [ ] Check IMU if moved

After battery change:
- [ ] Quick motor balance check
- [ ] Rotation timing may change slightly

---

## 🔢 Formulas

**Velocity from time**:
```
velocity (m/s) = distance (m) / time (s)
Example: 1.0m / 23s = 0.044 m/s
```

**Rotation time from degrees**:
```
time = (degrees / 90) × rotation_90_time
Example: 180° = (180/90) × 2.1s = 4.2s
```

**Motor power calculation**:
```
effective_power = base_speed × factor
Example: left = 0.60 × 1.34 = 0.804 (80.4%)
```

**PWM value**:
```
PWM = 255 × speed × factor
Example: 255 × 0.60 × 1.34 = 205
```

---

## 🎮 Test Scenarios

### Scenario 1: Initial Setup (New Robot)
1. Run `test_motors_diagnostic.py` → verify wiring
2. Run `test_calibration.py` → find left/right factors
3. Run `test_rotation_calibrate.py` → find rotation times
4. Run `test_imu_calibration.py` → calibrate IMU
5. Run `simple_mission.py` → verify complete system

### Scenario 2: Drift Problem
1. Run `test_calibration.py` → observe drift direction
2. Adjust factors in small increments (±0.05)
3. Retest until straight
4. Update `simple_state.py`

### Scenario 3: IMU Acting Strange
1. Run `test_imu_calibration.py` → check drift
2. If drift > 10°: Check wiring, sensor mounting
3. If drift acceptable but rotation off: Adjust `imu_odometry_scale`
4. If persistent issues: Disable IMU (`use_imu_odometry = False`)

### Scenario 4: Collision Issues
1. Test ultrasonic: `rover.getUltrasonicSensor()`
2. If readings good: Increase `obstacle_threshold`
3. If readings bad: Check sensor alignment
4. If movement inertia high: Lower speed or increase threshold

---

## 📁 File Locations

| File | Purpose |
|------|---------|
| `bt/simple_state.py` | Main calibration storage |
| `bt/actions.py` | Motor speed defaults |
| `calibration_config.json` | Backup/export values |
| `CALIBRATION_TABLES.md` | Full documentation |
| `calibration_manager.py` | Helper tool |

---

## 💡 Pro Tips

1. **Battery matters**: Calibrate at 100% battery, values may drift at <20%
2. **Surface matters**: Smooth floor ≠ carpet, recalibrate if surface changes
3. **Weight matters**: Adding payload changes dynamics, retest
4. **Small increments**: Adjust by 0.05 at a time for motor factors
5. **Test loop**: Always test 3+ times to verify consistency
6. **Document**: Note surface type, battery level, payload in comments
7. **Backup**: Save working calibration before experimenting
8. **IMU warmup**: Let robot sit 30s before IMU calibration

---

## 🚀 Quick Mission Pre-Flight Check (30 seconds)

```bash
# 1. Verify connection
python3 -c "from bt.actions import rover; print(f'✅ Connected: {rover.port}')"

# 2. Quick ultrasonic test
python3 -c "from bt.actions import rover; print(f'📡 Distance: {rover.getUltrasonicSensor():.1f}cm')"

# 3. IMU check (if available)
python3 -c "from bt.simple_state import simple_state; print(f'🧭 IMU: {'✅' if simple_state.imu.is_available() else '❌'}')"

# 4. Launch mission
python3 simple_mission.py
```

---

**Last Updated**: 2026-02-24  
**Maintained by**: Robot Calibration Team  
**Version**: 1.0
