# Courier Robot Architecture Diagram

## File Structure
```
courier_nav/
├── courier_nav/
│   ├── behaviors/                    Behavior Tree Behaviors Package
│   │   ├── __init__.py              Exports all behaviors
│   │   ├── navigation.py            Movement primitives
│   │   │   ├── RotateToTarget       Rotate to target yaw
│   │   │   ├── MoveToTarget         Move straight with LIDAR checks
│   │   │   ├── GetNextWaypoint      Process path queue
│   │   │   └── CenterOnCell         Center robot in cell
│   │   │
│   │   ├── mission.py               High-level actions
│   │   │   ├── CollectObject        Gripper collection animation
│   │   │   ├── DeliverObject        Gripper delivery animation
│   │   │   ├── PlanPath             Initial BFS path planning
│   │   │   ├── PlanReturnPath       Return BFS path planning
│   │   │   └── AlignWithAprilTag    Align with specific AprilTag
│   │   │
│   │   ├── conditions.py            State checks
│   │   │   └── IsPathComplete       Check if path empty
│   │   │
│   │   ├── obstacle.py              Obstacle handling
│   │   │   └── HandleObstacle       Backup and replan
│   │   │
│   │   └── battery.py               Battery management
│   │       ├── CheckBattery         Check battery level
│   │       └── ChargeBattery        Charge battery
│   │
│   ├── controller.py                Main controller
│   │   └── BehaviorTreeController   ROS2 node + BT orchestration
│   │       ├── create_behavior_tree()
│   │       ├── create_navigate_one_cell()
│   │       ├── start_mission()
│   │       ├── tick_tree()
│   │       ├── bfs_path()
│   │       ├── odom_callback()
│   │       ├── scan_callback()
│   │       ├── apriltag_callback()
│   │       └── utility methods
│   │
│   ├── apriltag_localizer.py        AprilTag detection
│   │   └── AprilTagLocalizer        Detect and estimate AprilTag poses
│   │
│   ├── world_spawner.py             World setup
│   │   └── WorldSpawner             Spawn grid, obstacles, robot
│   │
│   ├── mission_controller.py        Mission logic
│   │   └── MissionController        High-level mission control
│   │
│   └── generate_apriltags.py        AprilTag generation
│       └── GenerateAprilTags        Generate AprilTag images
│
├── apriltag_images/                Generated AprilTag images
├── package.xml                     ROS2 package manifest
├── setup.py                        Python package setup
└── setup.cfg                       Setup configuration
```


## Behavior Tree Structure (Created by controller.py)

```
Mission (Sequence)
├── Plan Path
│
├── Nav To Pickup Complete (FailureIsSuccess)
│   └── Navigate Pickup Loop (Repeat x 999)
│       └── Try Cell (Retry x 5)
│           └── Navigate One Cell (Sequence)
│               ├── Battery Management (Selector)
│               │   ├── Check Battery
│               │   └── Charge Battery
│               └── Navigate (Sequence)
│                   ├── Get Next Waypoint
│                   ├── Rotate To Target
│                   ├── Move Or Handle (Selector)
│                   │   ├── Move To Target
│                   │   └── Handle Obstacle
│                   └── Center On Cell
│
├── Precise Pickup (Sequence)
│   ├── Align With AprilTag #4
│   └── Collect Object
│
├── Plan Return Path
│
├── Recenter After Pickup
│
├── Nav To Home Complete (FailureIsSuccess)
│   └── Navigate Home Loop (Repeat x 999)
│       └── Try Cell (Retry x 5)
│           └── Navigate One Cell (Sequence)
│               ├── Battery Management (Selector)
│               │   ├── Check Battery
│               │   └── Charge Battery
│               └── Navigate (Sequence)
│                   ├── Get Next Waypoint
│                   ├── Rotate To Target
│                   ├── Move Or Handle (Selector)
│                   │   ├── Move To Target
│                   │   └── Handle Obstacle
│                   └── Center On Cell
│
└── Precise Delivery (Sequence)
    ├── Align With AprilTag #0
    └── Deliver Object
```

## Data Flow

```
ROS2 Topics -> BehaviorTreeController -> Blackboard -> Behaviors
              (controller.py)           (shared state)  (behaviors/*.py)

Topics:
  /odom          -> robot pose (x, y, yaw)
  /scan          -> LIDAR distance
  /apriltag_pose -> localization correction
  /cmd_vel       <- movement commands

Blackboard Keys:
  - node                 -> BehaviorTreeController instance
  - path_queue           -> deque of (row, col) waypoints
  - current_target       -> (row, col) of current cell
  - target_world_x/y     -> target position in odom frame
  - target_yaw           -> target orientation
  - rotation_start_time  -> timestamp for rotation timeout
  - object_collected     -> mission state flag
  - returning_home       -> mission phase flag
```


## Key Components

- **BehaviorTreeController**: Main ROS2 node that creates, configures, and ticks the behavior tree. Handles sensor callbacks, path planning, and mission orchestration.
- **AprilTagLocalizer**: Detects AprilTags in camera images and publishes pose estimates for localization.
- **WorldSpawner**: Sets up the simulation world, including grid, obstacles, and robot spawning.
- **MissionController**: Manages high-level mission logic and state.
- **GenerateAprilTags**: Utility to generate AprilTag images for the environment.
- **Behaviors**: Modular actions inheriting from `py_trees.behaviour.Behaviour`, each handling a specific aspect of robot behavior.

## Benefits

- **Modularity**: Each file has a focused responsibility
- **ROS2 Integration**: Proper node structure and topic handling
- **Behavior Tree**: Hierarchical decision-making with py_trees
- **Localization**: AprilTag-based pose correction
- **Simulation Ready**: World spawning and obstacle handling
- **Extensible**: Easy to add new behaviors or components

## Mission Flow

1. **Planning**: Plan initial path from start to pickup cell
2. **Navigation to Pickup**: Navigate grid, handling obstacles and battery
3. **Pickup**: Align with AprilTag #4 and collect object
4. **Return Planning**: Plan path back to start cell
5. **Navigation Home**: Navigate back, handling obstacles and battery
6. **Delivery**: Align with AprilTag #0 and deliver object
