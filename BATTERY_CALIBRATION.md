# Battery Voltage Calibration System

## Overview

The robot now includes **automatic voltage compensation** to maintain consistent motor performance as the battery drains. This prevents the robot from slowing down when battery voltage drops.

## Problem

As batteries discharge:
- **Voltage drops** (e.g., from 8.4V → 6.5V for LiPo 2S)
- Motors receive less power at the same PWM values
- Robot moves slower and less predictably
- Missions become unreliable with low battery

## Solution

The system automatically **scales motor speeds** based on current voltage:

```
Compensated Speed = Requested Speed × (Reference Voltage / Current Voltage)
```

**Example:**
- Reference voltage: 7.4V (fully charged)
- Current voltage: 6.8V (partially drained)
- Requested speed: 70%
- Compensation factor: 7.4 / 6.8 = 1.088
- **Actual speed sent**: 70 × 1.088 = **76%** ✅

## Features

### 1. **Automatic Speed Compensation**
- Continuously monitors battery voltage
- Adjusts motor speeds to maintain performance
- Maximum compensation: 30% (1.3× speed increase)
- Updates voltage every 10 seconds

### 2. **Battery Status Monitoring**
- Real-time voltage reading
- Battery percentage estimation
- Status indicators: FULL, GOOD, LOW, CRITICAL
- Low battery warnings

### 3. **Configurable Parameters**
```python
rover = RoverApi(
    port='/dev/ttyUSB0',
    enable_voltage_compensation=True,  # Enable/disable compensation
    reference_voltage=7.4,             # Voltage when fully charged
    min_voltage=6.4                    # Minimum safe voltage
)
```

## Usage

### Basic Usage (Default - Compensation Enabled)

```python
from rover_API import RoverApi

# Initialize with default compensation
rover = RoverApi('/dev/ttyUSB0')

# Move forward - speed automatically compensated
rover.moveTo('Forward', 0.7)
```

### Advanced Configuration

```python
from rover_API import RoverApi

# Custom configuration for different battery types
rover = RoverApi(
    port='/dev/ttyUSB0',
    enable_voltage_compensation=True,
    reference_voltage=7.4,    # Adjust for your battery (7.4V for LiPo 2S, 11.1V for 3S)
    min_voltage=6.4           # Safe cutoff voltage
)

# Check battery status
status = rover.getBatteryStatus()
print(f"Voltage: {status['voltage']:.2f}V")
print(f"Battery: {status['percentage']:.1f}%")
print(f"Status: {status['status']}")
print(f"Compensation: {status['compensation_factor']:.2f}x")
```

### Runtime Control

```python
# Disable compensation temporarily
rover.setVoltageCompensation(False)
rover.moveTo('Forward', 0.7)  # Raw speed, no compensation

# Re-enable compensation
rover.setVoltageCompensation(True)
rover.moveTo('Forward', 0.7)  # Compensated speed
```

### Battery Monitoring

```python
# Get detailed battery information
status = rover.getBatteryStatus()

if status['status'] == 'CRITICAL':
    print("⚠️ Battery critical - return to base!")
    # Emergency landing/return logic
elif status['status'] == 'LOW':
    print("⚠️ Battery low - finish mission soon")
```

## Arduino Configuration

The Arduino code ([RoverAPI.ino](../RoverAPI.ino)) already includes battery voltage reading via analog pin A0:

```cpp
// Battery command returns voltage
else if (cmd == "battery"){
    int rawValue = analogRead(A0);
    float voltage = (rawValue * 5.0 / 1023.0) * 2.0;
    Serial.println(voltage);
}
```

### ⚠️ Hardware Setup Required

For accurate readings, ensure:
1. **Voltage divider** connected to A0 (battery → R1(10kΩ) → A0 → R2(10kΩ) → GND)
2. Max voltage to A0: **5V** (Arduino limitation)
3. Adjust multiplier in Arduino code based on your voltage divider ratio

## Testing

Run the test suite to verify calibration:

```bash
cd rpi_bt_integration
python3 test_battery_calibration.py
```

### Test Options:
1. **Full monitoring test** - Continuous voltage tracking with movement
2. **No compensation test** - Compare behavior without compensation

## Battery Status Reference

| Status | Voltage Range (2S LiPo) | Description |
|--------|-------------------------|-------------|
| **FULL** | > 7.9V | Battery fully charged |
| **GOOD** | 6.9V - 7.9V | Normal operation |
| **LOW** | 6.4V - 6.9V | Should recharge soon |
| **CRITICAL** | < 6.4V | Stop mission immediately |

## Console Output Examples

### With Compensation Active:
```
✅ Connesso ad Arduino su /dev/ttyUSB0
🔋 Tensione batteria: 6.85V (compensazione attiva)
🔋 Compensazione batteria: 0.70 → 0.76 (6.85V)
Sto andando avanti...
```

### Low Battery Warning:
```
⚠️ Batteria in esaurimento: 6.65V
🔋 Compensazione batteria: 0.70 → 0.78 (6.65V)
```

### Critical Battery:
```
⚠️ BATTERIA BASSA: 6.35V - Ricaricare!
🔋 Compensazione batteria: 0.70 → 0.81 (6.35V)
```

## Calibration Tips

### 1. **Determine Your Reference Voltage**
- Measure battery voltage when fully charged
- Use this as `reference_voltage` parameter
- Examples:
  - LiPo 2S (2 cells): 7.4V nominal, 8.4V full
  - LiPo 3S (3 cells): 11.1V nominal, 12.6V full

### 2. **Set Minimum Voltage**
- Never discharge LiPo below **3.0V per cell**
- Safe minimums:
  - 2S: 6.4V (3.2V × 2)
  - 3S: 9.0V (3.0V × 3)

### 3. **Test at Different Voltage Levels**
1. Start with full battery - note robot speed
2. Run until voltage drops to ~80% (e.g., 7.0V for 2S)
3. Verify robot maintains similar speed
4. Continue testing at 60%, 40% remaining

### 4. **Fine-Tune if Needed**
- If robot still slows down: Lower `reference_voltage` slightly
- If robot speeds up too much: Raise `reference_voltage` slightly
- Maximum compensation can be adjusted in `_get_voltage_compensation_factor()` method

## Integration with Behavior Trees

The compensation works automatically with behavior tree actions:

```python
# In bt/actions.py - already configured!
rover = RoverApi(
    port='/dev/ttyUSB0',
    enable_voltage_compensation=True,
    reference_voltage=7.4,
    min_voltage=6.4
)

# All movement functions automatically benefit from compensation
def move_forward(duration=1.0):
    rover.moveTo('Forward', duration)  # Speed is auto-compensated!
```

## Troubleshooting

### Problem: Compensation not working
**Solution:** Check battery voltage reading:
```python
voltage = rover.getBatteryVoltage()
print(f"Voltage: {voltage}V")
```
- If voltage = 0.0: Check Arduino connection and voltage divider hardware

### Problem: Robot moves too fast with low battery
**Solution:** Compensation may be too aggressive. Check:
```python
status = rover.getBatteryStatus()
print(f"Compensation factor: {status['compensation_factor']}")
```
- If > 1.25: Battery very low OR reference_voltage set too high

### Problem: Battery readings fluctuate
**Solution:** 
- Add capacitor (100µF) across voltage divider
- Increase `voltage_check_interval` to 30 seconds
- Average multiple readings in Arduino code

## Benefits

✅ **Consistent Performance** - Robot maintains speed regardless of battery level  
✅ **Extended Mission Time** - Can operate reliably at lower voltages  
✅ **Predictable Behavior** - Movement durations remain consistent  
✅ **Battery Protection** - Warnings prevent over-discharge  
✅ **No Manual Tuning** - Automatically adapts to voltage changes  

## Limitations

⚠️ **Maximum compensation is 30%** - Below minimum voltage, robot will still slow down  
⚠️ **Requires voltage divider** - Hardware modification needed if not present  
⚠️ **Not suitable for damaged batteries** - Batteries with high internal resistance may not benefit  

## Next Steps

1. **Test with your battery** - Run calibration tests
2. **Adjust parameters** - Set correct reference and minimum voltages
3. **Monitor during missions** - Check compensation logs
4. **Add battery alerts** - Integrate with mission planning to return-to-base at low battery

---

**Updated:** February 12, 2026  
**Author:** GitHub Copilot  
**Version:** 1.0
