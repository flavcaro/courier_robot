Courier Robot – Design Document (Real Robot)

1. General Project Description
The project involves designing and deploying an autonomous mobile robot in a
structured indoor environment. The robot acts as a courier, capable of navigating
from a starting position (A) to a target position (B), picking up an object, and
returning to base. Navigation is performed without a discrete grid: the robot moves
along continuous metric distances and uses a Behavior Tree (BT) with a wall-following
algorithm (Left-Hand Rule or Right-Hand Rule) to avoid obstacles encountered along
the path. The robot detects obstacles in real time using an ultrasonic sensor, corrects
its heading using an IMU (MPU6050), and is controlled via a Raspberry Pi that
communicates with an Arduino Mega through a serial (USB) interface. All software is
written in Python 3 and runs natively on the Raspberry Pi — no ROS 2 or simulation
framework is required.

2. Real Robot Description
2.1 Body and Locomotion
• Tracked mobile base (tank-style differential drive)
• Four DC motors (two per side) controlled by the MeGaPi board
• Compact indoor frame suitable for maze-like environments
• Cell-free, distance-based navigation (no grid)

2.2 Sensor Set
• Ultrasonic sensor (MeUltrasonicSensor on PORT_7): front-facing obstacle
  detection with range up to ~400 cm
• IMU MPU6050 (connected to Raspberry Pi via I2C): provides gyroscope (yaw
  rate) and accelerometer (X, Y) data for heading estimation and odometry

2.3 Actuator Set
• Four DC motors for tracked locomotion (motors 1–4 on MeGaPi)
• Two DC motors for robotic arm lift (motors 5–6 on MeGaPi)
• One DC motor for the gripper/hand (motorHand on PORT4B)
• Buzzer for audio feedback (beep on error)

2.4 Body Shape & Movable Parts
• Body: rectangular tracked chassis
• Movable parts:
  o Left and right track groups (4 motors total)
  o Robot arm (2 motors, vertical lift)
  o Gripper/claw (1 motor, open/close)

3. Execution Environment
• Hardware: Raspberry Pi (running Python 3) + Arduino Mega (MeGaPi board)
• Communication: USB serial at 115200 baud (auto-detected port)
• Python libraries: py_trees (Behavior Tree), mpu6050-raspberrypi, smbus2
• No ROS 2, no Gazebo — runs entirely as standalone Python scripts
• Entry point: simple_mission.py (maze navigation) or test_*.py scripts for
  diagnostics

The Raspberry Pi acts as the high-level controller: it runs the Behavior Tree logic,
reads the IMU over I2C, and sends string commands to the Arduino. The Arduino
interprets commands and directly drives all motors and sensors via the MeGaPi
firmware. This architecture decouples high-level navigation logic (Python) from
low-level actuator control (C++ firmware).

4. Robot Goals
4.1 Main Goal
Enable the robot to autonomously navigate from a starting position (A) to a target
position (B), pick up an object at the target, and return to the starting point to
perform object delivery, maintaining safety (no collisions).

4.2 Sub-Goals
• Navigate in continuous metric space using wall-following (Left/Right-Hand Rule)
• Detect and stop before obstacles using the ultrasonic sensor (threshold ≤ 15 cm
  during motion, ≤ 30 cm during static scan)
• Correct heading drift continuously during movement using IMU (MPU6050)
• Rotate precisely by 90° using either timed rotation (fallback) or IMU feedback
• Pick up objects via preset arm sequences (open → down → close → up)
• Return safely to the base and release the object

5. Robot Agent Architecture
5.1 Hierarchical Layers
1. Sensing Layer – reads IMU (via I2C) and ultrasonic sensor (via Arduino serial).
2. Control Layer – manages robot state, executes the wall-following Behavior Tree,
   handles heading correction, and supervises mission phases.
3. Actuation Layer – sends serial commands to the Arduino which drives motors,
   arm, and gripper.

5.2 Internal Description
• The sensing layer provides real-time data on heading (MPU6050 gyro integration)
  and obstacle distance (ultrasonic). The IMU is polled at ~50 Hz during rotation
  and ~10 Hz during motion for heading correction.
• The control layer instantiates the py_trees Behavior Tree, applies the
  wall-following strategy, and tracks the robot's metric position (x, y) and heading
  (N/E/S/W) in the SimpleRobotState object.
• The actuation layer translates high-level commands (move forward, rotate, arm up,
  grab) into serial ASCII strings (e.g., "ForwardComp:80:75\n") sent to the Arduino.

6. Software Architecture
6.1 File Structure and Module Roles

Entry Points:
• simple_mission.py: Main mission script — calibrates IMU, asks for target distance,
  creates and ticks the Behavior Tree until SUCCESS or FAILURE.
• test_obstacle_avoid_simple.py: Standalone obstacle avoidance loop test.
• rover_API.py: Standalone serial API test (movement + lidar monitoring).

bt/ Module:
• bt/simple_state.py: Global singleton SimpleRobotState — stores metric position
  (x, y), heading (N/E/S/W), calibrated motion timings, IMU handle, and all
  tunable parameters (thresholds, factors, speeds).
• bt/simple_actions.py: Low-level motion primitives:
  - rotate_to_heading(): rotate to a cardinal direction (N/E/S/W) by shortest path;
    uses IMU feedback (rotate_90_with_imu) if available, timed fallback otherwise.
  - move_forward_meters(): time-based forward motion with obstacle check.
  - move_forward_meters_with_imu(): IMU-odometry-based forward motion with
    continuous heading correction (_move_segment_imu).
  - scan_fan_one_side(): fan scan on one lateral side to find a clear direction.
  - rotate_small() / rotate_to_micro_angle(): small timed micro-rotations.
• bt/simple_behaviours.py: py_trees Behaviour classes:
  - MazeNavigator: wall-following navigation loop (see §6.3).
  - GrabObject: timed arm pickup sequence.
  - create_main_mission_tree(): factory that assembles Sequence(MazeNavigator, GrabObject).
• bt/actions.py: Hardware instantiation — creates the global rover (RoverApi) object,
  defines DEFAULT_SPEED_LINEAR (0.60) and DEFAULT_SPEED_TURN (0.75), and provides
  arm_up(), arm_down(), open_hand(), close_hand() wrappers.
• bt/sensors.py: Legacy RobotState class (from grid era), kept for compatibility.
  Contains battery compensation logic and grid/world coordinate converters.
• bt/imu_sensor.py: IMUSensor class wrapping the MPU6050 library:
  - calibrate(): static offset calibration for gyro Z and accel X/Y.
  - update_heading(): integrates gyro Z over time to track heading (0–360°).
  - get_accel_xy(): returns calibrated X,Y accelerations for odometry.

Hardware Interface:
• rover_API.py: RoverApi class — manages a pySerial connection to the Arduino,
  auto-detects the port (/dev/ttyACM* or /dev/ttyUSB*), and exposes methods:
  moveTo(), stop(), rotate_differential(), rotate_differential_compensated(),
  getUltrasonicSensor(), armUP(), armDown(), openHand(), closeHand(),
  getBatteryVoltage(), beep().
• firmware_con_rotazione_differenziale.ino: Arduino firmware — receives ASCII
  commands over serial and drives all motors via MeGaPi. Key commands:
  Forward/Back/Left/Right/Stop, ForwardComp/BackComp (per-side PWM),
  RotateRight/RotateLeft, RotateRightComp/RotateLeftComp (compensated),
  ultrasonic, armUP, armDown, openHand, closeHand, getBattery, beep.

6.2 Modular Design — Separation of Concerns

Logic modules (platform-independent):
• bt/simple_state.py: robot state and configuration;
• bt/simple_actions.py: navigation primitives (rotation, movement, scanning);
• bt/simple_behaviours.py: mission logic as Behavior Tree;
• bt/imu_sensor.py: IMU abstraction.

Driver interface (platform-dependent):
• rover_API.py: serial communication with Arduino;
• firmware_con_rotazione_differenziale.ino: motor control firmware.

The Python layers only depend on rover_API.py for hardware access; swapping the
hardware layer (e.g., replacing the Arduino) would not require changes to the BT logic.

6.3 Wall-Following Navigation (MazeNavigator Behaviour)
The MazeNavigator behaviour implements an efficient wall-following strategy using
relative directional priorities:

Algorithm per tick:
1. Read ultrasonic sensor in the current heading direction.
2. EARLY OBJECT DETECTION: if remaining distance < 0.5 m and obstacle < 60 cm
   → declare SUCCESS (object detected at target).
3. If path is clear (> obstacle_threshold) → move forward one step (0.5 m) with
   continuous obstacle monitoring → return RUNNING.
4. If obstacle detected: enter wall-following scan.
   - Build priority list relative to current heading:
     * LEFT-HAND RULE: Forward > Left > Right > Back
     * RIGHT-HAND RULE: Forward > Right > Left > Back
   - For each candidate direction: rotate to it, read sensor (clearance ≥ 40 cm).
   - Re-check higher-priority directions before trying lower ones (dynamic priority).
   - Choose first free direction → move forward one step → return RUNNING.
5. If all directions blocked (stuck_count ≥ 3) → return FAILURE.

The robot tracks its 2D position (x, y) in metres and computes the Euclidean
distance from the start; the mission ends when this distance reaches target_distance.

6.4 IMU Integration (MPU6050)
The IMU serves two roles:

A. Heading estimation (gyroscope integration):
   heading_deg += gyro_z_calibrated × dt
   Used to close the loop on 90° rotations (rotate_90_with_imu): rotation stops
   when |angle_rotated − 90°| ≤ 2°, with a 5 s safety timeout.

B. Odometry (accelerometer integration):
   velocity_x = α·velocity_x + (1−α)·(ax·9.81·dt)  [α = 0.95]
   distance += |velocity| · dt
   IMU-estimated distance is scaled by imu_odometry_scale = 0.60 (IMU overstimates
   ~40% due to vibration noise).

C. Continuous heading correction during movement:
   Every 100 ms, a proportional correction is computed:
     error = current_heading − initial_heading  (normalised to ±180°)
     correction = clamp(gain · error, −0.25, 0.25)
     speed_left  = base_left  × (1 + correction)
     speed_right = base_right × (1 − correction)
   This keeps the robot on a straight line without stopping.

7. Main Functional Requirements
• Continuous obstacle detection and stop (ultrasonic, threshold 15 cm during motion).
• Wall-following navigation to a metric target distance using Left or Right-Hand Rule.
• IMU-based heading correction during forward movement (continuous, no stop).
• 90° rotations with IMU feedback (or timed fallback if IMU unavailable).
• Object pick-up via preset arm sequence (open → lower → close → raise).
• Battery voltage monitoring via Arduino (getBattery command).
• Centralized logging to stdout with emoji indicators for human-readable debugging.

8. Non-Functional Requirements
• Safety: immediate stop when obstacle within 15 cm during motion, within
  25 cm minimum for any rotation clearance;
• Modularity: clear separation between BT logic, motion primitives, IMU, and
  serial API;
• Portability: Python code runs on any Linux host with available serial port and
  I2C bus; no OS-level dependencies beyond pySerial and smbus2;
• Robustness: fallback to time-based navigation if IMU is unavailable;
• Calibration: all timing and speed parameters centralised in SimpleRobotState
  and tunable without code changes;
• Diagnostics: individual test scripts for motors, IMU, calibration, and obstacle
  avoidance.

9. Use Cases
UC1 – System Startup and Calibration
• Primary Actor: Operator
• Goal: Start the robot and calibrate the IMU
• Procedure:
  o Power on Raspberry Pi and Arduino (connected via USB)
  o Run: python3 simple_mission.py
  o Place robot on flat surface, press ENTER to start IMU calibration (300 samples)
  o Orient robot to North, press ENTER to set reference heading
  o Enter target distance in metres
• Post-condition: IMU calibrated, BT created, robot ready to start mission

UC2 – Navigation to Target (Wall-Following)
• Primary Actor: Robot (MazeNavigator)
• Goal: Reach the target metric distance from start
• Procedure:
  o MazeNavigator ticks in loop; checks ultrasonic sensor each tick
  o If clear: moves forward 0.5 m with continuous IMU heading correction
  o If obstacle: rotates to first free direction (priority relative to heading)
  o Repeats until Euclidean distance from start ≥ target_distance
• Post-condition: Robot near target object, ready for pickup

UC3 – Object Pick-Up
• Primary Actor: Robot (GrabObject)
• Goal: Grasp object in front of the robot
• Procedure:
  o Check ultrasonic: if < 40 cm → object present, else beep (FAILURE)
  o Open gripper (openHand, 2000 ms PWM)
  o Lower arm (armDown, 1500 ms)
  o Close gripper (closeHand, 1750 ms PWM)
  o Raise arm (armUP, 3000 ms)
• Post-condition: Object held, mission SUCCESS

UC4 – Error / Stuck Handling
• Primary Actor: Robot (MazeNavigator)
• Goal: Recover from a completely blocked state
• Procedure:
  o After 3 consecutive failed direction scans: attempt small backward movement
  o If still blocked: return FAILURE
• Post-condition: Mission aborted or recovered

10. Motor Compensation and Calibration
The real robot's tracks exhibit mechanical asymmetries corrected in software:

10.1 Forward Motion Compensation
Simple state parameters:
  left_factor  = 1.34  (left track needs more power to go straight)
  right_factor = 1.00  (reference)
Commands sent as: "ForwardComp:pwm_left:pwm_right\n"

10.2 Rotation Compensation
Separate calibrated times for left and right 90° rotations:
  rotation_90_time_left  = 1.99 s
  rotation_90_time_right = 2.10 s
During rotation, the track moving in reverse receives a boost factor
(rotation_reverse_boost = 1.30) to overcome static friction:
  speed_forward_track = DEFAULT_SPEED_TURN × rotation_forward_factor
  speed_reverse_track = DEFAULT_SPEED_TURN × rotation_reverse_boost
Commands: "RotateRightComp:pwm_left:pwm_right\n"

10.3 Speed Profile
• Linear movement: 60% PWM (DEFAULT_SPEED_LINEAR = 0.60)
  → ~4.4 cm/s (meters_per_second_forward = 0.044)
• Rotations: 75% PWM (DEFAULT_SPEED_TURN = 0.75)

11. Position Tracking
Without a grid, the robot tracks a continuous 2D position:

11.1 State Variables (SimpleRobotState)
  position_x, position_y: metric coordinates (metres), origin at start
  heading: N/E/S/W (cardinal direction)
  total_distance: total path length travelled

11.2 Position Update (update_position)
After each motion step of d metres in direction heading:
  if heading == 'N': position_y += d
  if heading == 'S': position_y -= d
  if heading == 'E': position_x += d
  if heading == 'W': position_x -= d

11.3 Target Condition
  distance_from_start = sqrt(position_x² + position_y²)
  Mission SUCCESS when distance_from_start ≥ target_distance

12. UML Diagrams
• Class diagram for robot modules (RoverApi, IMUSensor, SimpleRobotState,
  MazeNavigator, GrabObject)
• Activity diagram for mission flow:
  calibrate → BT tick → [obstacle?] → wall-follow → [at target?] → grab → done
• Sequence diagram: Raspberry Pi ↔ Arduino serial protocol
