# 📚 Calibration Documentation - README

This folder contains comprehensive calibration tables and tools for the Courier Robot.

## 📄 Files Overview

### Documentation
- **`CALIBRATION_TABLES.md`** - Complete calibration reference with detailed tables for all sensors and motors
- **`CALIBRATION_QUICK_REFERENCE.md`** - Quick reference card for fast lookups during calibration
- **`calibration_history.csv`** - Historical log of calibration values over time

### Configuration
- **`calibration_config.json`** - Machine-readable calibration configuration (backup/export)
- **`calibration_manager.py`** - Interactive tool to view/edit calibration values

## 🚀 Quick Start

### View Current Calibration
```bash
python3 calibration_manager.py
# Select option 1 to view report
```

### Update Calibration Values
```bash
python3 calibration_manager.py
# Select option 2 for interactive update
```

### Run Calibration Tests
```bash
# Motor balance
python3 test_calibration.py

# Rotation timing
python3 test_rotation_calibrate.py

# IMU calibration
python3 test_imu_calibration.py

# Velocity measurement
python3 test_imu_odometry.py
```

## 📊 Calibration Workflow

```
┌─────────────────────────────────────────────────────────────┐
│ 1. RUN TEST SCRIPTS                                         │
│    • test_calibration.py                                    │
│    • test_rotation_calibrate.py                             │
│    • test_imu_calibration.py                                │
└────────────────┬────────────────────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────────────────────┐
│ 2. COLLECT MEASUREMENTS                                     │
│    • Motor drift direction                                  │
│    • Rotation timing for 90°                                │
│    • IMU drift rate                                         │
│    • Velocity (time for 1 meter)                            │
└────────────────┬────────────────────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────────────────────┐
│ 3. UPDATE VALUES                                            │
│    Option A: Edit bt/simple_state.py directly               │
│    Option B: Use calibration_manager.py → export code       │
│    Option C: Edit calibration_config.json                   │
└────────────────┬────────────────────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────────────────────┐
│ 4. LOG CHANGES                                              │
│    • Add entry to calibration_history.csv                   │
│    • Note date, surface, battery level, changes             │
└────────────────┬────────────────────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────────────────────┐
│ 5. VERIFY                                                   │
│    • Run test again to verify improvement                   │
│    • Test full mission: simple_mission.py                   │
└─────────────────────────────────────────────────────────────┘
```

## 🎯 What to Calibrate When

### 🆕 Brand New Robot / First Time Setup
1. **Motor wiring check**: `test_motors_diagnostic.py`
2. **Motor balance**: `test_calibration.py` → adjust `left_factor`/`right_factor`
3. **Rotation timing**: `test_rotation_calibrate.py` → adjust `rotation_90_time_left`/`right`
4. **IMU**: `test_imu_calibration.py` → run calibration routine
5. **Velocity**: `test_imu_odometry.py` OR measure manually → adjust `meters_per_second_forward`
6. **Full mission test**: `simple_mission.py` → verify all systems

### 🔋 Battery Level Changed Significantly
- Quick motor balance check (may drift slightly with low battery)
- Rotation timing may be slower at <30% battery

### 🏗️ Hardware Changed (track replacement, weight added, etc.)
- Full recalibration recommended (all 5 steps above)

### 🌍 Surface Type Changed (smooth floor → carpet)
- Motor balance test
- Rotation timing test (may need adjustment due to friction)
- Velocity recalibration

### 🔧 IMU Sensor Moved/Replaced
- Full IMU recalibration (`test_imu_calibration.py`)
- Check all 3 test modes: static, drift, rotation

### 📡 Robot Colliding with Obstacles
- Check ultrasonic readings: `rover.getUltrasonicSensor()`
- Adjust `obstacle_threshold` (increase if stopping too late)
- Test at current speed before changing speed settings

### 🎯 Robot Drifting During Movement
- If drifts sideways: Adjust `left_factor`/`right_factor` (motor balance)
- If drifts in heading: Enable/tune `imu_heading_correction_gain`
- If happens only during long movements: IMU drift compensation

## 📋 Calibration Tables Summary

### Motor & Track
| Parameter | Current | Min | Max | Step |
|-----------|---------|-----|-----|------|
| left_factor | 1.34 | 0.5 | 2.0 | 0.05 |
| right_factor | 1.00 | 0.5 | 2.0 | 0.05 |

### Rotation
| Parameter | Current | Min | Max | Step |
|-----------|---------|-----|-----|------|
| rotation_90_time_left | 1.99s | 1.0s | 10.0s | 0.1s |
| rotation_90_time_right | 2.10s | 1.0s | 10.0s | 0.1s |

### Velocity
| Parameter | Current | Unit | Measurement |
|-----------|---------|------|-------------|
| forward_velocity | 0.044 | m/s | Time 1 meter at 60% power |

### Ultrasonic
| Threshold | Current | Purpose |
|-----------|---------|---------|
| Dynamic | 15 cm | Stop while moving |
| Static | 30 cm | Scanning stationary |
| Clearance | 40 cm | Path planning |

### IMU
| Parameter | Current | Range | Purpose |
|-----------|---------|-------|---------|
| odometry_scale | 0.60 | 0.1-2.0 | Distance correction |
| heading_gain | 0.25 | 0.0-1.0 | Drift correction rate |

## 🛠️ Tools Reference

### calibration_manager.py
Interactive Python tool to:
- View formatted calibration report
- Update values through menu
- Export to Python code for bt/simple_state.py
- Manage calibration_config.json

### Test Scripts
| Script | Purpose | Duration |
|--------|---------|----------|
| `test_calibration.py` | Motor balance (drift test) | ~20s per test |
| `test_rotation_calibrate.py` | Find 90° rotation time | ~5-10min |
| `test_imu_calibration.py` | IMU calibration & drift | ~2min |
| `test_imu_rotation.py` | IMU rotation accuracy | ~3min |
| `test_imu_odometry.py` | IMU distance accuracy | ~2min |
| `test_motors_diagnostic.py` | Verify motor wiring | ~2min |

## 💾 Backup & Version Control

### Before Making Changes
```bash
# Backup current calibration
cp bt/simple_state.py bt/simple_state.py.backup
cp calibration_config.json calibration_config.json.backup

# Or use git
git add bt/simple_state.py calibration_config.json
git commit -m "Calibration backup before tuning"
```

### After Successful Calibration
```bash
# Update history log
echo "2026-02-24,smooth_floor,80%,1.34,1.00,1.99,2.10,0.044,15.0,30.0,0.60,0.25,Tuned for carpet" >> calibration_history.csv

# Commit changes
git add bt/simple_state.py calibration_*.json calibration_history.csv
git commit -m "Calibration update: improved motor balance"
```

## 📖 Additional Resources

### Detailed Documentation
See `CALIBRATION_TABLES.md` for:
- Complete parameter tables with descriptions
- Measurement characteristics for all sensors
- Calibration procedures with step-by-step instructions
- Troubleshooting guides

### Quick Reference
See `CALIBRATION_QUICK_REFERENCE.md` for:
- One-page cheat sheet for fast lookups
- Quick commands and formulas
- Troubleshooting table
- Pre-flight checklist

## 🔍 Troubleshooting

### Calibration Not Helping?
1. **Check hardware first**:
   - Are tracks tight/loose?
   - Is battery level stable (>50%)?
   - Are motors making unusual sounds?
   - Is IMU firmly mounted?

2. **Verify surface**:
   - Is floor level?
   - Is surface consistent (not half carpet/half floor)?
   - Are there obstacles interfering?

3. **Check test procedure**:
   - Is robot actually stationary during IMU calibration?
   - Are you measuring from same starting point?
   - Is battery at similar level across tests?

### Values Seem Wrong?
- **Compare with defaults**: See current values in tables
- **Check units**: m/s vs cm/s, seconds vs milliseconds
- **Verify calculation**: Use formulas in quick reference
- **Test smaller range**: If factor is 1.5, try 1.4-1.6 in 0.05 steps

### Can't Find Right Value?
- **Broader range**: Try wider range of values
- **Smaller steps**: Use 0.01 steps instead of 0.05
- **Multiple tests**: Average 3-5 test runs
- **Different conditions**: Test with different battery levels/surfaces

## 📞 Support

For questions or issues:
1. Check `CALIBRATION_TABLES.md` troubleshooting section
2. Review test script output for error messages
3. Check IMU availability: `simple_state.imu.is_available()`
4. Verify serial connection: `rover.port`

## 📝 Contributing

When you find improved calibration values:
1. Test thoroughly (3+ successful runs)
2. Document test conditions (surface, battery, payload)
3. Update `calibration_history.csv`
4. Share findings with team

---

**Last Updated**: February 24, 2026  
**Version**: 1.0  
**Maintainer**: Robot Calibration Team
