# Quick Start: Battery Voltage Calibration

## ✅ What's Done

Your robot now has **automatic battery voltage compensation**! The system:

1. ✅ Monitors battery voltage continuously
2. ✅ Automatically increases motor speeds as battery drains
3. ✅ Warns when battery is low
4. ✅ Maintains consistent robot performance throughout mission

## 🚀 How to Use

### Option 1: Use Existing Code (Already Configured!)

Your existing code in [bt/actions.py](rpi_bt_integration/bt/actions.py) now automatically uses voltage compensation:

```bash
cd rpi_bt_integration
python3 main.py              # Basic BT with compensation
python3 main_mission.py      # Delivery mission with compensation
```

**No changes needed** - compensation is active by default! 🎉

### Option 2: Test the Calibration System

```bash
cd rpi_bt_integration
python3 test_battery_calibration.py
```

Choose test:
- **Test 1**: Full monitoring with continuous voltage display
- **Test 2**: Compare with/without compensation

### Option 3: Custom Python Script

```python
from rover_API import RoverApi

# Initialize with compensation (default)
rover = RoverApi('/dev/ttyUSB0')

# Check battery
status = rover.getBatteryStatus()
print(f"Battery: {status['voltage']:.2f}V ({status['percentage']:.0f}%)")

# Move - speed automatically compensated!
rover.moveTo('Forward', 0.7)
```

## 📊 What You'll See

### Normal Operation:
```
✅ Connesso ad Arduino su /dev/ttyUSB0
🔋 Tensione batteria: 7.20V (compensazione attiva)
Sto andando avanti...
```

### Battery Draining (Auto-Compensation):
```
🔋 Compensazione batteria: 0.70 → 0.76 (6.85V)
Sto andando avanti...
```

### Low Battery Warning:
```
⚠️ Batteria in esaurimento: 6.65V
🔋 Compensazione batteria: 0.70 → 0.78 (6.65V)
```

## ⚙️ Configuration (Optional)

Edit [bt/actions.py](rpi_bt_integration/bt/actions.py) to customize:

```python
rover = RoverApi(
    port='/dev/ttyUSB0',
    enable_voltage_compensation=True,   # Enable/disable
    reference_voltage=7.4,              # Your battery fully charged voltage
    min_voltage=6.4                     # Minimum safe voltage
)
```

### Battery Types:
- **LiPo 2S (2 cells)**: `reference_voltage=7.4`, `min_voltage=6.4`
- **LiPo 3S (3 cells)**: `reference_voltage=11.1`, `min_voltage=9.0`

## 🔧 Arduino Hardware Check

Ensure voltage divider is connected to pin A0:
```
Battery(+) → R1(10kΩ) → A0 → R2(10kΩ) → GND
```

Test voltage reading:
```python
rover = RoverApi('/dev/ttyUSB0')
voltage = rover.getBatteryVoltage()
print(f"Voltage: {voltage}V")
```

Expected: ~7.4V for full LiPo 2S (if 0.0V, check hardware)

## 📖 Full Documentation

See [BATTERY_CALIBRATION.md](BATTERY_CALIBRATION.md) for:
- How compensation works
- Advanced configuration
- Battery status monitoring
- Troubleshooting
- Integration examples

## 🎯 Benefits

- ✅ **Consistent speed** at any battery level
- ✅ **Longer mission times** - reliable down to 6.4V
- ✅ **No manual tuning** needed
- ✅ **Automatic warnings** prevent damage
- ✅ **Works with existing code** - no changes required!

## 📝 Files Modified

1. [rover_API.py](rpi_bt_integration/rover_API.py) - Added compensation system
2. [bt/actions.py](rpi_bt_integration/bt/actions.py) - Enabled by default
3. [test_battery_calibration.py](rpi_bt_integration/test_battery_calibration.py) - Test suite

---

**Ready to test!** Just run your existing missions - compensation is already active! 🚀
