# Courier Robot - Project Flow & Structure Analysis

## PROJECT FLOW

1. START POINT
   └─ start_mission.sh (or Docker container)
      │
      ├─ source /opt/ros/jazzy/setup.bash
      ├─ colcon build --symlink-install
      │
      └─► Step 1-6 Below

2. GAZEBO SIMULATION
   └─ ros2 launch courier_description sim.launch.py
      │
      ├─ Launches Gazebo (empty.sdf world)
      ├─ Starts ROS-Gazebo Bridge (ros_gz_bridge)
      │  └─ Bridges: /cmd_vel, /odom, /scan, /camera, /camera_info, /tf, /clock
      └─► Ready for spawning

3. WORLD SPAWNING 
   └─ ros2 run courier_nav spawner
      │
      ├─ world_spawner.py (BehaviorTreeController not involved)
      ├─ Spawns 80+ objects in parallel (12 max workers):
      │  ├─ Floor tiles (5x5 = 25)
      │  ├─ Grid lines (horizontal + vertical = 12)
      │  ├─ Boundary walls (4)
      │  ├─ Obstacles (4)
      │  ├─ AprilTag markers (28 = 12 walls + 16 obstacle-mounted)
      │  └─ Start/Goal markers (2)
      └─► Grid visible in Gazebo

4. ROBOT SPAWNING
   └─ gz service call (in start_mission.sh)
      │
      └─ Spawns /home/ubuntu/ros2_ws/robot.sdf
         └─ Position: (0.5, 0.5, 0.15) = cell (0,0) center
         └─ Equipped with:
            ├─ GPU LIDAR (360°, 10 Hz, range 0.15-6.0m)
            ├─ Camera (for AprilTag detection)
            └─ Wheels (commanded via /cmd_vel)

5. STATIC TRANSFORMS
   └─ ros2 run tf2_ros static_transform_publisher
      │
      ├─ map → odom (identity)
      ├─ base_link → base_footprint (0.1m up)
      ├─ base_link → lidar_link (0.2m up)
      └─ base_link → camera_link (0.15m forward, 0.15m up)

6. APRITAG LOCALIZER
   └─ ros2 run courier_nav apriltag_localizer
      │
      ├─ Listens to /camera images
      ├─ Detects AprilTags (28 known positions)
      ├─ Estimates robot pose using PnP
      ├─ Publishes /apriltag_pose corrections
      └─ Fuses with odometry (30% tag, 70% odometry)

7. MISSION CONTROLLER (BEHAVIOR TREE)
   └─ ros2 run courier_nav mission_controller
      │
      ├─ Starts BehaviorTreeController (controller.py)
      ├─ Creates behavior tree (py_trees)
      ├─ Ticks at ~10 Hz
      │
      └─► BEHAVIOR TREE LOGIC:
         │
         ├─ PlanPath (BFS from (0,0) to (4,2))
         │  └─ Uses static obstacles from world_spawner
         │
         ├─ For each waypoint:
         │  ├─ GetNextWaypoint (pop from queue)
         │  ├─ RotateToTarget (align to waypoint direction)
         │  ├─ MoveToTarget (move straight)
         │  └─ Check LIDAR for dynamic obstacles
         │     └─ If obstacle: HandleObstacle → back up → replan
         │
         ├─ Reach goal (4,2):
         │  ├─ CollectObject (4-sec animation)
         │  └─ Set "object_collected = true"
         │
         ├─ PlanReturnPath (BFS from (4,2) to (0,0))
         │  └─ Same static obstacles
         │
         ├─ Navigate home using same waypoint logic
         │
         └─ Reach home (0,0):
            ├─ DeliverObject (4-sec animation)
            └─ Mission complete! ✅

```
---

## STRUCTURE


```
courier_robot/
├── README.md (updated)
├── ARCHITETTURA.md (updated - document actual BFS+BT architecture)
├── courier_behavior_tree.mmd
├── Dockerfile
├── .gitignore
├── .git/
│
└── ros2_ws/
    ├── robot.sdf 
    ├── requirements.txt 
    ├── cleanup.sh 
    ├── start_mission.sh (ONLY startup script)
    │
    └── src/
        ├── courier_description/
        │   ├── package.xml
        │   ├── CMakeLists.txt
        │   ├── urdf/courier.urdf.xacro
        │   ├── config/gui.config
        │   └── launch/sim.launch.py
        │
        └── courier_nav/
            ├── package.xml
            ├── setup.py
            ├── setup.cfg
            ├── courier_nav/
            │   ├── __init__.py
            │   ├── mission_controller.py
            │   ├── controller.py 
            │   ├── world_spawner.py 
            │   ├── apriltag_localizer.py
            │   ├── generate_apriltags.py 
            │   ├── apriltag_images/ (28 PNG)
            │   └── behaviors/ 
            ├── launch/
            │   └── cell_navigation.launch.py (optional, secondary)
            └── test/ (optional - boilerplate)
```

---

## EXECUTION FLOW

```
User runs: start_mission.sh
    ↓
1. Build & source ROS2
2. Launch Gazebo + Bridge (sim.launch.py)
3. Spawn world objects (world_spawner.py)
4. Spawn robot
5. Publish static TF transforms
6. Start AprilTag localizer
7. Start mission controller (BehaviorTreeController)
    ↓
Behavior Tree ticks at 10 Hz:
    ├─ Plan BFS path (0,0) → (4,2)
    ├─ Navigate cell-by-cell
    │  ├─ Rotate to target
    │  ├─ Move straight
    │  └─ Check LIDAR
    ├─ Collect object (4 sec)
    ├─ Plan return path (4,2) → (0,0)
    ├─ Navigate home
    ├─ Deliver object (4 sec)
    └─ Mission complete
```