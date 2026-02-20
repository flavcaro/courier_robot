Courier Robot – Design Document
1. General Project Description
The projpct involves designing and simulating an autonomous mobile robot in a
structured indoor environment. The robot acts as a courier, capable of transporting
items such as medicines or packages while navigating on a static 2D grid (0 = free cell, 1
= occupied cell). Pathfinding is performed using a Breadth-First Search (BFS) algorithm,
which guarantees the shortest viable route on a grid and ensures predictable, cell-by-
cell navigation. The robot identifies targets via AprilTags using the OpenCV library for
the detection, executes pick-up using preset arm poses, and returns to base, all
implemented in ROS 2 Jazzy and tested in Gazebo 11 LTS.
2. Choice of the Simulated Robot Model
2.1 Body and Locomotion
•Mobile base with differential drive or tracks
•Compact frame suitable for indoor environments
•Stability adequate for cell-by-cell movement
2.2 Sensors Set
•Camera for AprilTag detection
•Encoders for odometry
•Distance sensor (LIDAR) for immediate obstacle detection
2.3 Actuators Set
•Differential drive motors
•Robotic arm with 2–3 DOF•
Gripper or claw for object pick-up
2.4 Body Shape & Movable Parts
•Body: rectangular compact chassis
•Movable parts:
o Arm joints (2–3 DOF)
o Gripper/claw
o Differential drive wheels/tracks
3. Simulation Environment
•Gazebo Harmonic + ROS 2 Jazzy
•Official plugins for sensors, motors, and AprilTags
•Ready-made models for mobile manipulators
•Realistic simulation transferable to real robot
ROS 2 Jazzy was chosen because it is a stable LTS release that offers reliable, modular,
and real-time-capable communication between sensing, control, and actuation nodes,
making the system easily transferable from simulation to the real robot. Gazebo
Harmonic complements it by providing a realistic physics environment, accurate
sensor simulation (camera, encoders), and native ROS 2 integration through official
plugins. Together, they allow developing, testing, and validating navigation, perception,
and manipulation using the exact same architecture used on the physical robot.
4. Robot Goals
4.1 Main Goal•
Enable the robot to autonomously navigate from a starting cell (A) to a target cell
(B), pickup object at target cell, return to starting cell again to perform object
delivery, maintaining accuracy and safety.
4.2 Sub-Goals
•Navigate on a static 2D grid using BFS path planning
•Detect and realign using AprilTags and onboard sensors
•Pick up objects via preset arm poses:
•Return safely to the base and release the object
We perform the arm’s pick-up and delivery via simple log simulation: robot waits 4
seconds on target cell to pick up object, and another 4 seconds on the start cell to
simulate delivery once it has returned.
5. Robot Agent Architecture
5.1 Hierarchical Layers
1. Sensing Layer – reads sensors (camera, encoders, LIDAR).
2. Control Layer – manages grid, executes BFS path planning, error correction, and
mission supervision.
3. Actuation Layer – controls motors, arm, and gripper.
5.2 Internal Description
•
The sensing layer provides real-time data on robot position, orientation, and
obstacle detection.
•
The control layer decides movements, triggers error correction, and supervises
mission phases.•
The actuation layer executes locomotion and arm movements according to
commands from control.
6. Software Architecture
6.1 ROS 2 Jazzy Structure
Sensing Layer
•Odometry sensor: Tracks robot poses in 2D space (position + orientation) .
•LIDAR sensor: Detects obstacles and provides distance measurements for
collision avoidance .
•
AprilTag detector: Provides visual localization corrections to reduce odometry
drift.
Control Layer•Grid-based world model: 5×5 discrete grid with obstacle mapping.
•BFS path planning: Computes optimal paths avoiding known obstacles.
•Behavior tree controller: Hierarchical mission supervision with 7-phase
execution.
•
Error correction systems: AprilTag-based localization fusion, dynamic obstacle
detection and replanning, battery management with automatic charging, cell-
centering using LIDAR lateral distance.
Actuation Layer
•
Differential drive control: Publishes velocity commands (/cmd_vel) for
navigation.
•
Gripper operations: Simulated pickup/delivery sequences (4-second
animations, ready for hardware integration).
Communication Infrastructure
•
ROS 2 Topics: Primary communication method Publishers: /cmd_vel,
/path_markers Subscribers: /odom, /scan, /apriltag_pose.
•
Centralized logging: ROS 2 logger for real-time monitoring and post-mission
analysis.
•
ROS 2 Graph visualization: Available via rqt_graph for system debugging.
6.2 Modular Design
Separation of Concerns
Logic modules (platform-independent):
•behaviors/navigation.py: Cell-based navigation primitives;
•behaviors/mission.py: Path planning and mission phases;
•behaviors/battery.py: Energy management and charging;
•behaviors/obstacle.py: Dynamic obstacle handling;
•behaviors/conditions.py: State checking and validation;
Driver interface (platform-dependent):•
Uses only standard ROS 2 message types (Twist, Odometry,
LaserScan,PoseWithCovarianceStamped);
•
Interfaces with Gazebo simulator or real robot hardware.
6.3 Integration of OpenCV in the Courier Robot Project
In the Courier Robot project, OpenCV is used as the main library for image processing
and for supporting the robot’s visual perception.
Role of OpenCV in the system
1. AprilTag detection and pose estimation
• handling camera frames
• converting images to grayscale
• feature extraction
• use of intrinsic camera parameters
• estimation of the 3D pose of the tag with respect to the camera
2. Support for localization
• correction of odometry drift• fusion with sensor data (odometry + AprilTag)
• precise realignment during pick-up and delivery phases
3. Camera calibration
• representation of the camera_matrix (ndarray)
• handling distortion coefficients
• image distortion correction
4. Vision pipeline
• acquisition of images from the simulator camera
• image pre-processing
• sending the results to the control node for navigation
OpenCV is therefore integrated into the Sensing layer, working together with ROS 2 and
the control algorithms to provide the robot with accurate visual perception that can be
used for navigation and manipulation.
7. Main Functional Requirements
•Cell-by-cell navigation using BFS.
•Position error handling and realignment using AprilTags, odometry, and LIDAR.
•Pick-up via robotic arm simulation.
•Return to base and final recognition via AprilTag.
•Centralized logging and operator error communication.
8. Non-Functional Requirements
•Position accuracy <10% of cell size;
•Error correction and realignment <200 ms;
•Modularity: clear separation between logic and drivers;
•Portability: same logic for simulation and real robot;
•Safety: immediate stop on unexpected obstacles.
•Implementation Requirements:o Dockerized multi-process deployment;
o Message broker system for node communication;
o Centralized logging system;
o GUI (RViz) for sensor and robot state visualization in real-time or
simulated time;
o Shared software repository (MS Teams code folder);
o ReadMe.md instructions for installation and testing;
9. Use Cases
UC1 – Mission Start
•Primary Actor: Operator
•Goal: Start robot and initialize sensors and actuators
•Procedure:
o Power on robot or simulation
o Load static 2D grid
o Initialize sensors and arm/gripper in POSE_HOME
o Perform self-diagnostic•
Post-condition: Robot ready to start mission
UC2 – Navigation to Target
•Primary Actor: Robot
•Goal: Reach target cell
•Procedure:
o Read BFS path
o Move cell-by-cell
o After each cell, check position relative to cell center and correct errors
o Detect AprilTag for final alignment
•
Post-condition: Robot aligned above target
UC3 – Object Pick-up
•Primary Actor: Robot (arm)
•Goal: Grasp object using preset poses
•Procedure:
o Execute preset sequence: POSE_ABOVE_OBJECT → POSE_GRIP →
POSE_CARRY
o Gripper closes on object
o Sensor confirmation of secure grasp
•
Post-condition: Object held, ready for transport
UC4 – Return to Base
•Primary Actor: Robot
•Goal: Return to starting cell
•Procedure:
o Compute reverse BFS path (B → A)
o Move cell-by-cell, correcting position errors
o Align with base AprilTag
o Execute POSE_RELEASE and open gripper to release object
•
Post-condition: Object delivered, robot at starting position
10. Position Error Handling and Cell Centering
In the behavior code (navigation.py), centering and position-error handling are
implemented in two complementary stages: coarse navigation and fine centering.
10.1 MoveToTarget: Coarse Navigation and Safety Checks
MoveToTarget computes the nominal odometry error to the target waypoint in the world
frame as:
ex_odom = target_world_x − node.robot_x
ey_odom = target_world_y − node.robot_y
An optional AprilTag-based lateral correction is applied using
get_apriltag_lateral_correction. This correction:
•
•Uses only very recent detections
Ignores lateral offsets smaller than 0.03 m or larger than 0.15 m
•Converts the lateral offset into a bounded angular correction using a 0.8 m
lookahead distance
•
Limits the maximum angular correction to approximately 0.10 rad
Forward motion is rejected if an obstacle is detected or if excessive heading drift is
observed. Heading drift is computed as:
angle_error = normalize_angle(target_yaw − robot_yaw)
The angle error is compared against a dynamic drift_tolerance:
•
•
0.10 rad during the first second
0.18 rad after the first second
If the tolerance is exceeded, the behavior triggers recovery (e.g., rotation or replanning)
instead of continuing forward motion.
10.2 CenterOnCell: Fine Position Centering
After reaching the target vicinity, CenterOnCell performs precise centering
using odometry-based position estimation. The world-frame position error is:
ex = target_x - robot_x (from odometry)
ey = target_y - robot_y (from odometry)
The error is transformed into the robot frame as:
x_r = cos(yaw) · ex + sin(yaw) · ey (forward/backward)
y_r = -sin(yaw) · ex + cos(yaw) · ey (left/right)
A proportional controller generates velocity commands based on these errors:
- Linear speed: proportional to x_r (forward error)
- Angular speed: proportional to atan2(y_r, x_r) (heading correction)
A centering tolerance of 0.04 m (4 cm) is used as the success condition.
A timeout of 4 seconds prevents indefinite blocking if centering cannot
converge due to odometry drift.10.3 Centering Control Law
During centering, a small, bounded proportional controller is used. The desired heading
is computed as:
desired_heading = atan2(y_r, x_r)
The velocity commands are:
linear_speed = clamp(k_linear · x_r, −0.08, 0.08)
angular_speed = clamp(k_angular · desired_heading, −0.6, 0.6)
If the error is primarily lateral (i.e., the desired heading is large), the behavior prioritizes
rotation with minimal forward motion until the lateral error is reduced. Otherwise, the
robot drives forward with gentle angular correction.
When both |x_r| and |y_r| fall below the centering tolerance, the controller stops the
robot and reports success.
10.4 Final Alignment Using AprilTags
For high-precision tasks such as object pick-up or drop-off, AprilTag detection provides
the exact pose relative to the target. The robot performs fine-tuning of its position using
the tag, ensuring alignment within millimeter-level accuracy.
Overall Behavior
The navigation strategy combines multiple layers:
•
Coarse odometry-based navigation with conservative heading-drift checks
during approach (MoveToTarget)
•Use of recent AprilTag corrections and short-range LIDAR lateral corrections
•A bounded proportional controller in CenterOnCell to achieve tight centering
accuracy of approximately 4 cm, with a safety timeout to ensure robustness
11. UML Diagrams
•Class diagrams for robot modules (Sensing, Control, Actuation)
•Activity diagram for mission flow (navigate → pick-up → return)