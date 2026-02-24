# Courier Robot - Maze Navigation System

**Intelligent Systems & Robotics Laboratory - UNIVAQ**  
**Group Mecha Minds**

---

## Table of Contents
- [Project Overview](#project-overview)
- [Physical Requirements](#physical-requirements)
- [Software Requirements](#software-requirements)
- [Installation Guide](#installation-guide)
- [Hardware Setup](#hardware-setup)
- [Calibration Procedures](#calibration-procedures)
- [Usage Instructions](#usage-instructions)
- [Project Structure](#project-structure)
- [Features & Capabilities](#features--capabilities)
- [Troubleshooting](#troubleshooting)
- [Technical Details](#technical-details)

---

## Project Overview

This project implements an **autonomous courier robot** capable of navigating through maze environments using **wall-following algorithms** and **behavior tree architecture**. The robot features:

- **Maze Navigation**: Autonomous navigation using left-hand/right-hand wall-following rules
- **Object Manipulation**: Robotic arm with gripper for picking up and transporting objects
- **Sensor Fusion**: IMU-based odometry with ultrasonic obstacle detection
- **Intelligent Behavior**: py_trees behavior tree framework for decision-making
- **Battery Optimization**: Efficient movement strategies to maximize operational time

### Mission Scenario
The robot must navigate through a maze, avoid obstacles, reach a target location, grab an object, and return to the starting point - all while maintaining precise positioning and efficient battery usage.

---

## Physical Requirements

### Robot Platform
- **Model**: Makeblock Ultimate 2.0
- **Controller**: Makeblock MegaPi (Arduino Mega 2560 compatible)
- **Base**: Tracked/tank drive robot chassis (differential drive)
- **Motors Configuration**:
  - **Physical**: 1 DC motor per track (left + right)
  - **Arduino Code**: 2 motors per track for compatibility with reference implementation
    - Motors 1-2 (PORT1A/B): Left track (both controlled together)
    - Motors 3-4 (PORT2A/B): Right track (both controlled together)
  - Motors 5-6 (PORT3A/B): Robotic arm
  - Motor Hand (PORT4B): Gripper/hand

**Note**: The firmware uses dual-motor commands per track based on the [RoverAPI reference implementation](https://github.com/AAAI-DISIM-UnivAQ/RoverAPI), though only one physical motor per track is present on the Ultimate 2.0.

### Sensors
- **Ultrasonic Sensor** (MeUltrasonicSensor on PORT_7)
  - Range: 2-400 cm
  - Used for obstacle detection and distance measurement
  
- **IMU MPU6050** (I2C connection)
  - 3-axis gyroscope
  - 3-axis accelerometer
  - I2C Address: 0x68 (default)
  - Used for heading tracking and movement correction

### Additional Components
- **Robotic Arm**: 2-DOF arm with gripper
- **Power Supply**: Battery pack suitable for MegaPi + motors
- **I2C Connection**: Wired from Raspberry Pi to MPU6050
- **WiFi USB Adapter**: TP-Link TL-WN725N
  - 2.4GHz wireless connectivity (Raspberry Pi 2 doesn't support 5GHz)
  - Required for SSH remote access
  - Driver installation needed (see Installation Guide)

### Computing Platform
- **Raspberry Pi** (any model with I2C support)
  - Runs Python control software
  - Communicates with MegaPi via USB serial
  - Connects to IMU via I2C
  - Requires WiFi adapter for remote access

---

## Software Requirements

### Operating System
- **Raspberry Pi OS Lite (32-bit) Legacy** - Based on Debian Buster
- **Python 3.7+** (tested on Python 3.9+)

**Note**: The project uses the legacy 32-bit version for compatibility with older Raspberry Pi models and stability with I2C/serial interfaces.

### Python Libraries

#### Core Dependencies
```bash
# Behavior Tree Framework
py-trees>=2.1.0

# Serial Communication
pyserial>=3.5

# IMU Sensor Library
mpu6050-raspberrypi>=1.2.0
smbus2>=0.4.0
```

#### System Libraries (pre-installed on Raspberry Pi OS)
- `time`, `math`, `sys`, `os`, `glob`

### Arduino IDE
- **Version**: 1.8.x or 2.x
- **Board Manager**: Makeblock MegaPi board support
- **Libraries Required**:
  - `MeMegaPi.h` (Makeblock official library)

### Reference Implementation
This project is based on the [RoverAPI](https://github.com/AAAI-DISIM-UnivAQ/RoverAPI) framework developed by AAAI-DISIM-UnivAQ, adapted for autonomous maze navigation with behavior trees.

---

## Installation Guide

### Step 1: Raspberry Pi Setup

#### 1.1 Install WiFi USB Adapter Drivers

The TP-Link TL-WN725N requires driver installation on Raspberry Pi OS Lite:

```bash
# Update system first
sudo apt-get update
sudo apt-get upgrade

# Install WiFi adapter drivers
cd ~
git clone https://github.com/lwfinger/rtl8188eu.git
cd rtl8188eu
make all
sudo make install
sudo modprobe 8188eu
```

Configure WiFi connection:
```bash
sudo raspi-config
```
Navigate to: `System Options > Wireless LAN` and enter your network credentials.

**Note**: The adapter only supports 2.4GHz networks. Ensure your router has 2.4GHz enabled.

Reboot:
```bash
sudo reboot
```

Verify WiFi connection:
```bash
ifconfig wlan0
ping -c 4 google.com
```

#### 1.2 Enable I2C Interface
```bash
sudo raspi-config
```
Navigate to: `Interface Options > I2C > Enable`

Reboot:
```bash
sudo reboot
```

#### 1.3 Verify I2C Connection
After connecting the MPU6050 to the Raspberry Pi:
```bash
sudo i2cdetect -y 1
```
You should see device at address `0x68`.

#### 1.4 Install Python Dependencies
```bash
# Install Python packages
pip3 install py-trees pyserial smbus2 mpu6050-raspberrypi

# Alternative: using requirements file
pip3 install -r requirements.txt
```

### Step 2: Arduino Firmware Setup

#### 2.1 Install Arduino IDE
Download from: https://www.arduino.cc/en/software

#### 2.2 Install Makeblock Library
1. Open Arduino IDE
2. Go to `Sketch > Include Library > Manage Libraries`
3. Search for "Makeblock" or "MegaPi"
4. Install `Makeblock` library

#### 2.3 Upload Firmware
1. Connect MegaPi to computer via USB
2. Open `robot_code/firmware_con_rotazione_differenziale.ino`
3. Select board: `Tools > Board > Arduino Mega 2560`
4. Select port: `Tools > Port > [Your COM port]`
5. Click `Upload` button

#### 2.4 Verify Firmware
Open Serial Monitor (115200 baud) and send test commands:
- `Forward:100` - Should move forward
- `Stop` - Should stop
- `GetDistance` - Should return distance reading

### Step 3: Clone/Transfer Project Files

```bash
# On Raspberry Pi
cd ~/
git clone [your-repository-url] courier_robot

# Or transfer files manually via SCP
scp -r ./courier_robot pi@[raspberry-ip]:~/
```

### Step 4: Configure Serial Port

#### Option 1: Environment Variable (Recommended)
```bash
export ROVER_PORT=/dev/ttyACM0
# Add to ~/.bashrc for persistence
echo 'export ROVER_PORT=/dev/ttyACM0' >> ~/.bashrc
```

#### Option 2: Auto-detection
The `rover_API.py` automatically detects the Arduino port if `ROVER_PORT` is not set.

### Step 5: Test Connection

```bash
cd ~/courier_robot/robot_code
python3 rover_API.py
```

Expected output:
```
Connessione rover (autodetect)...
Rover connesso su /dev/ttyACM0
```

---

## Hardware Setup

### Wiring Diagram

#### MegaPi Connections
```
PORT1A + PORT1B: Left Track Motor (motor1, motor2 - controlled together)
PORT2A + PORT2B: Right Track Motor (motor3, motor4 - controlled together)
PORT3A + PORT3B: Arm Motors (motor5, motor6)
PORT4B: Gripper Motor (motorHand)
PORT_7: Ultrasonic Sensor (Trigger/Echo)
```

**Note**: Although the Makeblock Ultimate 2.0 has only one motor per track, the firmware uses dual-motor commands (PORT1A+B for left, PORT2A+B for right) for compatibility with the RoverAPI reference implementation. Both motors in each pair receive identical commands.

#### I2C Connection (Raspberry Pi to MPU6050)
```
RPi Pin 3 (GPIO2) → MPU6050 SDA
RPi Pin 5 (GPIO3) → MPU6050 SCL
RPi Pin 6 (GND)   → MPU6050 GND
RPi Pin 1 (3.3V)  → MPU6050 VCC
```

**WARNING**: 
- MPU6050 operates at **3.3V** - do NOT connect to 5V!
- Ensure I2C pull-up resistors are present (usually on-board)
- Keep I2C wires short (<30cm) to reduce noise

#### Serial Connection
- MegaPi USB → Raspberry Pi USB port (auto-detected as /dev/ttyACM0 or /dev/ttyUSB0)

### Physical Assembly Checklist

- [ ] Left track motor connected to PORT1A (PORT1B receives same signal)
- [ ] Right track motor connected to PORT2A (PORT2B receives same signal)
- [ ] Arm motors connected to PORT3A/B
- [ ] Gripper motor connected to PORT4B
- [ ] Ultrasonic sensor mounted at front, connected to PORT_7
- [ ] IMU MPU6050 connected via I2C to Raspberry Pi
- [ ] WiFi USB adapter (TL-WN725N) plugged into Raspberry Pi USB port
- [ ] WiFi adapter drivers installed and connected to 2.4GHz network
- [ ] MegaPi powered with adequate battery
- [ ] Raspberry Pi powered and communicating with MegaPi via USB
- [ ] All grounds connected (RPi and MegaPi share common ground)

---

## Calibration Procedures

Calibration is **critical** for accurate navigation. Follow these procedures in order:

### 1. IMU Calibration (Automatic)

The IMU auto-calibrates during mission startup, but you can test it separately:

```bash
cd robot_code
python3 test_imu_calibration.py
```

**Procedure**:
1. Place robot on **flat, level surface**
2. Robot must be **completely stationary**
3. Script will collect 300 samples (~5 seconds)
4. Gyroscope and accelerometer offsets calculated automatically

**Expected Output**:
```
Calibrazione completata:
   Gyro Z offset: 0.234 °/s
   Accel X offset: -0.012 g
   Accel Y offset: 0.008 g
```

### 2. Motor Compensation Calibration

Motors may have different power outputs due to mechanical differences:

```bash
python3 test_calibration.py
```

**Procedure**:
1. Place robot on smooth, flat surface
2. Align robot with a straight reference line (tape on floor)
3. Run test and observe drift direction
4. Adjust values in `bt/simple_state.py`:
   ```python
   self.left_factor = 1.34   # Adjust if robot drifts right
   self.right_factor = 1.00  # Keep as reference
   ```

**Interpretation**:
- Robot drifts **LEFT** → Increase `left_factor` (e.g., 1.34 → 1.40)
- Robot drifts **RIGHT** → Decrease `left_factor` (e.g., 1.34 → 1.25)

### 3. Rotation Calibration

Calibrate 90° rotation timing for precise turns:

```bash
python3 test_rotation_calibrate.py
```

**Procedure**:
1. Mark robot's initial orientation on floor
2. Script tests increasing time intervals
3. Measure actual rotation angle
4. When robot rotates exactly 90°, note the time

**Update** in `bt/simple_state.py`:
```python
self.rotation_90_time_left = 1.99   # Time for 90° left turn
self.rotation_90_time_right = 2.1   # Time for 90° right turn
```

### 4. Speed Calibration

Measure actual robot speed for accurate odometry:

**Procedure**:
1. Mark a 1-meter distance on floor
2. Time how long robot takes to traverse it
3. Calculate: `speed = distance / time`

**Update** in `bt/simple_state.py`:
```python
self.meters_per_second_forward = 0.044  # Your measured speed
```

### 5. Arm/Gripper Test

Test arm mechanisms:

```bash
python3 test_arm.py
```

Tests opening, closing, raising, and lowering sequences.

---

## Usage Instructions

### Basic Maze Navigation Mission

```bash
cd robot_code
python3 simple_mission.py
```

**Interactive Setup**:
1. **Target Distance**: Enter desired distance from start (default: 3.0m)
2. **Calibration**: IMU auto-calibrates (keep robot stationary)
3. **Initial Scan**: Ultrasonic sensor checks for obstacles
4. **Mission Start**: Press ENTER to begin

**During Mission**:
- Robot displays status every 10 ticks
- Heading corrections applied automatically
- Obstacle detection stops robot before collision
- Wall-following algorithm guides navigation

**Example Output**:
```
MAZE NAVIGATION - WALL-FOLLOWING MISSION
Velocità: 60% lineare, 75% rotazione
Strategia: Wall-Following (Left-Hand Rule)
Imposta parametri missione:
   Distanza target dal punto iniziale in metri [default 3.0]: 2.5
   Target: 2.5m dal punto iniziale
```

### Test Scripts

#### Test Individual Components

**Arm Control**:
```bash
python3 test_arm.py
```

**IMU Readings**:
```bash
python3 test_imu_rotation.py
```

**Motor Diagnostics**:
```bash
python3 test_motors_diagnostic.py
```

**Obstacle Avoidance**:
```bash
python3 test_obstacle_avoid_simple.py
```

---

## Project Structure

```
courier_robot/
│
├── robot_code/
│   ├── firmware_con_rotazione_differenziale.ino  # Arduino firmware
│   ├── rover_API.py                              # Low-level robot API
│   ├── simple_mission.py                         # Main mission script
│   │
│   ├── bt/                                       # Behavior Tree modules
│   │   ├── __init__.py
│   │   ├── actions.py                            # Basic movement actions
│   │   ├── simple_actions.py                     # Complex movement actions
│   │   ├── simple_behaviours.py                  # Behavior tree nodes
│   │   ├── simple_state.py                       # Robot state management
│   │   └── imu_sensor.py                         # IMU driver
│   │
│   ├── test_*.py                                 # Calibration/test scripts
│   ├── calibration_*.py/json/csv                 # Calibration utilities
│   │
│   └── CALIBRATION_*.md                          # Calibration documentation
│
├── bt.drawio                                     # Behavior tree diagram
└── README.md                                     # This file
```

---

## Features & Capabilities

### Navigation

#### Wall-Following Algorithm
- **Left-Hand Rule**: Robot keeps "left hand" on wall (default)
- **Right-Hand Rule**: Alternative strategy (configurable)
- **Priority System**: Forward > Left/Right > Back
- **Dynamic Replanning**: Adapts to encountered obstacles

#### Movement Control
- **Differential Drive**: Independent track control for precise turning
- **Compensated Movement**: Automatic drift correction for straight-line motion
- **IMU Odometry**: Real-time position tracking using sensor fusion
- **Obstacle Detection**: Continuous ultrasonic scanning during movement

### Behavior Tree Architecture

The robot uses **py_trees** for hierarchical decision-making:

```
Mission Root
├── Sequence: Navigate to Target
│   ├── MazeNavigator
│   │   ├── Check Distance from Start
│   │   ├── Move Forward (0.5m steps)
│   │   ├── Detect Obstacle
│   │   └── Choose Direction (Wall-Following)
│   └── GrabObject
│       ├── Open Gripper
│       ├── Lower Arm
│       ├── Close Gripper
│       └── Raise Arm
└── Return to Start (optional)
```

### Sensor Fusion

#### IMU Integration
- **Gyroscope**: Tracks rotational velocity for heading estimation
- **Accelerometer**: Provides velocity/position (with integration)
- **Complementary Filter**: Combines gyro (98%) + accel (2%) for robust heading
- **Continuous Correction**: Heading adjustments during movement (no stop-correct-move)

#### Ultrasonic Sensor
- **Obstacle Threshold**: 30-50 cm (configurable)
- **Continuous Monitoring**: Checks during movement, not just at waypoints
- **Smart Scanning**: Only scans necessary directions (battery optimization)

### Battery Optimization

- **Reduced Speed**: 60% linear, 75% rotation (vs. 100% max)
- **Efficient Scanning**: Stops at first free direction (not full 360°)
- **Wall-Following**: Follows maze walls instead of random exploration
- **Movement Smoothing**: Fewer stop-start cycles

---

## Calibration Procedures

### Quick Calibration Checklist

For optimal performance, calibrate in this order:

1. **IMU Calibration** (~1 minute)
   - Automatic during mission startup
   - Robot must be stationary on flat surface

2. **Movement Speed** (~5 minutes)
   - Measure distance robot travels in known time
   - Update `meters_per_second_forward` in `simple_state.py`

3. **Drift Compensation** (~10 minutes)
   - Observe straight-line drift direction
   - Adjust `left_factor` / `right_factor` in `simple_state.py`

4. **Rotation Timing** (~15 minutes)
   - Find exact time for 90° rotation
   - Update `rotation_90_time_left/right` in `simple_state.py`

### Detailed Calibration

See separate documentation:
- [CALIBRATION_README.md](robot_code/CALIBRATION_README.md) - Full calibration guide
- [CALIBRATION_QUICK_REFERENCE.md](robot_code/CALIBRATION_QUICK_REFERENCE.md) - Quick reference
- [CALIBRATION_TABLES.md](robot_code/CALIBRATION_TABLES.md) - Calibration data tables

### Configuration Files

- `calibration_config.json`: Stores calibration parameters
- `calibration_history.csv`: Tracks calibration sessions
- `bt/simple_state.py`: Main configuration parameters

**Key Parameters** in `simple_state.py`:
```python
# Movement speeds
self.meters_per_second_forward = 0.044  # m/s
self.rotation_90_time_left = 1.99       # seconds
self.rotation_90_time_right = 2.1       # seconds

# Drift compensation
self.left_factor = 1.34   # Left track multiplier
self.right_factor = 1.00  # Right track multiplier (reference)

# Rotation compensation
self.rotation_reverse_boost = 1.30  # Extra power for reverse track
```

---

## Usage Instructions

### Running the Main Mission

1. **Power up the robot**
   - Ensure MegaPi is powered
   - Boot Raspberry Pi

2. **Connect via SSH** (if running headless)
   
   **Note**: SSH requires the TP-Link TL-WN725N WiFi adapter to be installed and connected to your 2.4GHz network.
   
   ```bash
   ssh pi@raspberrypi.local
   # Or use IP address if .local doesn't resolve:
   ssh pi@192.168.1.XXX
   ```

3. **Navigate to project directory**
   ```bash
   cd ~/courier_robot/robot_code
   ```

4. **Run the mission**
   ```bash
   python3 simple_mission.py
   ```

5. **Configure mission parameters**
   - Target distance: Distance from start point (meters)
   - Default: 3.0m

6. **Calibration phase**
   - Keep robot stationary for IMU calibration
   - Wait for confirmation beep

7. **Start navigation**
   - Press ENTER to begin
   - Robot will navigate autonomously
   - Press Ctrl+C to emergency stop

### Understanding the Output

```
STATUS (tick 0):
   Position: (0.0, 0.0) | Heading: N
   Distance: 0.00m / 3.00m
   Ultrasonic: 85.3cm
```

- **Position**: Current (x, y) coordinates in meters
- **Heading**: Current direction (N/E/S/W)
- **Distance**: Distance traveled from start
- **Ultrasonic**: Current obstacle distance

### Mission Behavior

1. **Navigation Phase**:
   - Robot moves forward in 0.5m increments
   - Continuously monitors for obstacles (<30cm)
   - When blocked, scans available directions
   - Chooses direction based on wall-following rule
   - Corrects heading drift automatically with IMU

2. **Target Reached**:
   - When distance from start ≥ target distance
   - Executes object grab sequence
   - Mission completes

3. **Object Grab Sequence**:
   - Open gripper (1000ms PWM)
   - Lower arm (1.5s)
   - Close gripper (1750ms PWM)
   - Raise arm (3.5s)

### Emergency Stop

**Method 1**: Press `Ctrl+C` in terminal  
**Method 2**: Kill Python process  
**Method 3**: Power off MegaPi (emergency only)

The robot will stop all motors safely.

---

## Testing & Validation

### Pre-Mission Checklist

- [ ] WiFi adapter (TL-WN725N) installed and connected to 2.4GHz network
- [ ] SSH connection working
- [ ] IMU connected and detected (`sudo i2cdetect -y 1`)
- [ ] Arduino firmware uploaded and responsive
- [ ] Serial connection established
- [ ] IMU calibration completed
- [ ] Movement compensation calibrated
- [ ] Test arena prepared (flat surface, clear obstacles)
- [ ] Battery fully charged

### Component Tests

#### 1. Test IMU Sensor
```bash
python3 test_imu_calibration.py
```
Validates gyroscope, accelerometer, heading tracking.

#### 2. Test Rotations
```bash
python3 test_imu_rotation.py
```
Compares time-based vs. IMU-feedback rotations.

#### 3. Test Motor Response
```bash
python3 test_motors_diagnostic.py
```
Verifies motor wiring and compensation parameters.

#### 4. Test Arm Mechanism
```bash
python3 test_arm.py
```
Tests full grab sequence: open → lower → close → raise.

#### 5. Test Obstacle Avoidance
```bash
python3 test_obstacle_avoid_simple.py
```
Tests forward movement with obstacle detection and avoidance.

---

## Troubleshooting

### IMU Issues

**Problem**: `IMU non disponibile!`

**Solutions**:
1. Check I2C enabled:
   ```bash
   sudo raspi-config  # Interface > I2C > Enable
   ```

2. Verify connection:
   ```bash
   sudo i2cdetect -y 1
   # Should show 0x68
   ```

3. Check library installation:
   ```bash
   pip3 install --upgrade mpu6050-raspberrypi smbus2
   ```

4. Check wiring:
   - VCC → 3.3V (NOT 5V!)
   - GND → GND
   - SDA → GPIO2 (Pin 3)
   - SCL → GPIO3 (Pin 5)

### Serial Communication Issues

**Problem**: `Nessuna porta seriale trovata`

**Solutions**:
1. Check USB connection
2. Verify port permissions:
   ```bash
   sudo usermod -a -G dialout $USER
   # Logout and login again
   ```

3. List available ports:
   ```bash
   ls /dev/ttyACM* /dev/ttyUSB*
   ```

4. Manually specify port:
   ```bash
   export ROVER_PORT=/dev/ttyACM0
   ```

### WiFi Connection Issues

**Problem**: WiFi adapter not recognized or not connecting

**Solutions**:
1. Verify adapter is detected:
   ```bash
   lsusb
   # Should show: Realtek Semiconductor Corp. RTL8188EUS
   ```

2. Check if driver module is loaded:
   ```bash
   lsmod | grep 8188eu
   ```

3. If driver not loaded, reinstall:
   ```bash
   cd ~/rtl8188eu
   make clean
   make all
   sudo make install
   sudo modprobe 8188eu
   ```

4. Verify WiFi interface:
   ```bash
   ifconfig wlan0
   # or
   ip addr show wlan0
   ```

5. Check network configuration:
   ```bash
   sudo nano /etc/wpa_supplicant/wpa_supplicant.conf
   ```
   Should contain:
   ```
   network={
       ssid="YourNetwork"
       psk="YourPassword"
   }
   ```

6. Restart networking:
   ```bash
   sudo systemctl restart dhcpcd
   sudo wpa_cli -i wlan0 reconfigure
   ```

**Problem**: Can't connect via SSH

**Solutions**:
1. Ensure WiFi adapter is connected to 2.4GHz network (not 5GHz)
2. Find Raspberry Pi IP address (connect monitor temporarily):
   ```bash
   hostname -I
   ```
3. Verify SSH is enabled:
   ```bash
   sudo raspi-config  # Interface Options > SSH > Enable
   ```
4. Test connection from computer:
   ```bash
   ping raspberrypi.local
   # Or use IP directly
   ping 192.168.1.XXX
   ```

### Movement Issues

**Problem**: Robot drifts left/right during straight movement

**Solution**: Recalibrate motor compensation
- Edit `bt/simple_state.py`:
  ```python
  self.left_factor = X.XX   # Adjust this value
  self.right_factor = 1.00  # Keep as reference
  ```
- Run `test_calibration.py` to find optimal value

**Problem**: Rotations overshoot/undershoot 90°

**Solution**: Adjust rotation timing
- Edit `bt/simple_state.py`:
  ```python
  self.rotation_90_time_left = X.XX
  self.rotation_90_time_right = X.XX
  ```
- Run `test_rotation_calibrate.py` to find optimal values

**Problem**: IMU heading drifts during movement

**Solution**: Recalibrate IMU or adjust complementary filter
- Ensure robot was stationary during calibration
- Check for magnetic interference near IMU
- Adjust `self.alpha` in `imu_sensor.py` (default: 0.98)

### Ultrasonic Sensor Issues

**Problem**: Inconsistent or noisy distance readings

**Solutions**:
1. Check sensor mounting (must face forward, unobstructed)
2. Verify wiring to PORT_7
3. Test sensor directly:
   ```python
   from bt.actions import rover
   print(rover.getUltrasonicSensor())  # Should return distance in cm
   ```
4. Increase detection threshold in `simple_state.py`:
   ```python
   self.obstacle_threshold = 35  # Increase from 30
   ```

### Behavior Tree Issues

**Problem**: Mission gets stuck or doesn't progress

**Solution**: Enable debug visualization
- Uncomment debug prints in `simple_behaviours.py`
- Check behavior tree status:
  ```python
  print(py_trees.display.unicode_tree(tree.root, show_status=True))
  ```

---

## Technical Details

### Control Architecture

```
┌─────────────────────────────────────────────┐
│         Raspberry Pi (Python)               │
│  ┌────────────────────────────────────┐    │
│  │     Behavior Tree (py_trees)       │    │
│  │  - MazeNavigator                   │    │
│  │  - GrabObject                      │    │
│  │  - Decision Logic                  │    │
│  └────────────────────────────────────┘    │
│              │          │                    │
│     ┌────────┘          └───────┐           │
│     ▼                           ▼            │
│  [rover_API]                [imu_sensor]     │
│     │                           │            │
└─────┼───────────────────────────┼────────────┘
      │ (USB Serial)              │ (I2C)
      ▼                           ▼
┌─────────────┐           ┌─────────────┐
│   MegaPi    │           │  MPU6050    │
│  (Arduino)  │           │    (IMU)    │
│             │           └─────────────┘
│  ┌────────┐ │
│  │Motors  │ │
│  │Servo   │ │
│  │Sensors │ │
│  └────────┘ │
└─────────────┘
```

### Coordinate System

- **Origin**: Robot's starting position (0, 0)
- **Axes**:
  - X-axis: East (positive) / West (negative)
  - Y-axis: North (positive) / South (negative)
- **Headings**:
  - N = 0° (North)
  - E = 90° (East)
  - S = 180° (South)
  - W = 270° (West)

### Movement Model

#### Linear Movement
- **Base Speed**: 60% PWM (battery-optimized)
- **Actual Speed**: ~4.4 cm/s (calibrated)
- **Compensation**: Left track 1.34x, Right track 1.0x
- **Step Size**: 0.5m increments with continuous obstacle checking

#### Rotational Movement
- **Differential Drive**: One track forward, one track reverse
- **Rotation Speed**: 75% PWM
- **Time for 90°**: ~2 seconds (left: 1.99s, right: 2.1s)
- **Compensation**: Reverse track gets 1.30x boost (higher friction)

### Algorithm Details

#### Wall-Following Strategy

**Left-Hand Rule** (default):
```
While not at target:
  1. Try move Forward
  2. If blocked: Try turn Left
  3. If blocked: Try turn Right  
  4. If blocked: Turn Back (180°)
  5. Update position
```

**Priority**: Forward > Left > Right > Back

This ensures the robot follows the left wall of the maze until reaching the target distance.

#### Behavior Tree Execution

- **Tick Rate**: 10 Hz (every 0.1 seconds)
- **Status Updates**: Every 10 ticks (1 second)
- **Obstacle Check**: Every movement cycle
- **Heading Correction**: Continuous during movement (PID-like control)

### IMU Sensor Fusion

**Complementary Filter**:
```python
heading(k+1) = α × (heading(k) + gyro_z × Δt) + (1-α) × accel_heading
```
Where α = 0.98 (heavily favors gyroscope for short-term accuracy)

**Benefits**:
- Reduces gyroscope drift over time
- Compensates for accelerometer noise
- Provides stable heading estimate

---

## Advanced Configuration

### Tuning Wall-Following Rule

Edit in `bt/simple_state.py`:
```python
self.wall_following_rule = 'left'  # or 'right'
```

### Adjusting Movement Parameters

```python
# In bt/simple_state.py:

# Step size for maze navigation
self.maze_step_size = 0.4  # meters (smaller = more precise, slower)

# Obstacle detection threshold
self.obstacle_threshold = 30  # cm (larger = more cautious)

# Micro-rotation for scanning
self.micro_turn_degrees = 30  # degrees per scan step
self.scan_threshold = 50  # cm for free path detection
```

### Speed Profiles

Edit in `bt/actions.py`:
```python
DEFAULT_SPEED_LINEAR = 0.60  # 60% for battery saving
DEFAULT_SPEED_TURN = 0.75    # 75% for precise rotations
```

**Trade-offs**:
- **Higher speed** (0.80-1.0): Faster mission, more battery drain, less precise
- **Lower speed** (0.40-0.60): Better precision, longer mission, battery efficient

---

## Development Notes

### Design Decisions

1. **Why Behavior Trees?**
   - Modular, reusable behaviors
   - Clear mission structure
   - Easy to debug and extend
   - Industry standard for robotic systems

2. **Why Wall-Following?**
   - Guarantees maze exit (connected mazes)
   - Efficient battery usage
   - Simple, robust algorithm
   - No need for mapping/SLAM

3. **Why IMU Integration?**
   - Eliminates time-based rotation errors
   - Continuous heading correction
   - Better odometry than time-only
   - Professional-grade navigation

4. **Why Differential Drive Compensation?**
   - Real-world motors are never perfectly matched
   - Track friction varies
   - Mechanical differences accumulate
   - Calibration essential for straight movement

### Known Limitations

- **IMU Drift**: Gyroscope integrates error over time (mitigated by complementary filter)
- **Odometry Error**: Wheel slip on smooth surfaces introduces position error
- **Ultrasonic Noise**: Soft materials or angled surfaces give poor readings
- **Battery Voltage**: Motor speed varies with battery voltage (not compensated)

### Future Enhancements

- [ ] Encoder-based odometry for ground-truth distance
- [ ] Kalman filter for improved sensor fusion
- [ ] SLAM (Simultaneous Localization and Mapping)
- [ ] Vision-based object recognition
- [ ] Path optimization (A* algorithm)
- [ ] Multi-robot coordination

---

## References

### Libraries
- **py_trees**: https://py-trees.readthedocs.io/
- **MPU6050**: https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/
- **Makeblock**: https://www.makeblock.com/
- **RoverAPI Reference**: https://github.com/AAAI-DISIM-UnivAQ/RoverAPI

### Algorithms
- **Wall-Following**: Classic maze-solving algorithm
- **Complementary Filter**: Sensor fusion technique
- **Behavior Trees**: AI decision-making architecture

### Academic Context
- **Course**: Intelligent Systems & Robotics Laboratory
- **Institution**: UNIVAQ (Università degli Studi dell'Aquila)
- **Topics Covered**:
  - Autonomous navigation
  - Sensor fusion
  - Behavior-based robotics
  - Real-time control systems

---

## Author & License

**Project**: Autonomous Courier Robot - Maze Navigation System  
**Course**: Intelligent Systems & Robotics Laboratory  
**Institution**: UNIVAQ  
**Year**: 2025-2026

---

## Support & Contact

For technical issues:
1. Check [Troubleshooting](#troubleshooting) section
2. Review calibration documentation
3. Test individual components with provided test scripts
4. Verify hardware connections and power supply

---

## Exam Demonstration Checklist

Before your exam demonstration:

- [ ] All calibrations completed and verified
- [ ] WiFi adapter connected to 2.4GHz network and SSH working
- [ ] Test runs successful in practice arena
- [ ] Battery fully charged (>50% minimum)
- [ ] Backup battery available
- [ ] All connections secure and verified
- [ ] Code documented and clean
- [ ] Behavior tree diagram prepared (`bt.drawio`)
- [ ] Understand wall-following algorithm
- [ ] Can explain IMU sensor fusion
- [ ] Can demonstrate calibration procedure
- [ ] Prepared for questions on:
  - Behavior tree design choices
  - Sensor fusion mathematics
  - Motor control compensation
  - Algorithm complexity analysis

### Expected Performance

- **Navigation Accuracy**: ±5cm position error per meter
- **Rotation Accuracy**: ±2° per 90° turn
- **Obstacle Detection**: 100% stop rate before collision
- **Mission Success Rate**: >90% in standard maze
- **Battery Life**: ~30-45 minutes continuous operation

---