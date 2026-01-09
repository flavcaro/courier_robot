# Refactored Architecture Diagram

## File Structure
```
courier_nav/
├── courier_nav/
│   ├── behaviors/                    📁 Behavior Tree Behaviors Package
│   │   ├── __init__.py              ← Exports all behaviors
│   │   ├── navigation.py            ← Movement primitives (230 lines)
│   │   │   ├── RotateToTarget       🔄 Rotate to cardinal direction
│   │   │   ├── MoveToTarget         ➡️  Move straight with safety checks
│   │   │   └── GetNextWaypoint      🎯 Process path queue
│   │   │
│   │   ├── mission.py               ← High-level actions (155 lines)
│   │   │   ├── CollectObject        🤖 Gripper collection animation
│   │   │   ├── DeliverObject        📦 Gripper delivery animation
│   │   │   └── PlanReturnPath       🗺️  BFS path planning
│   │   │
│   │   ├── conditions.py            ← State checks (20 lines)
│   │   │   └── IsPathComplete       ✅ Check if path empty
│   │   │
│   │   └── obstacle.py              ← Obstacle handling (65 lines)
│   │       └── HandleObstacle       🚧 Backup and replan
│   │
│   ├── controller.py                ← Main controller (495 lines)
│   │   └── BehaviorTreeController   🌳 ROS2 node + BT orchestration
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
│   └── nav2_mission_controller_bt.py ← Entry point (35 lines)
│       └── main()                    🚀 Launch function
│
└── REFACTORING_SUMMARY.md            📄 Documentation
```

## Module Dependencies

```
nav2_mission_controller_bt.py
    └── controller.py
        ├── behaviors/navigation.py
        │   ├── RotateToTarget
        │   ├── MoveToTarget
        │   └── GetNextWaypoint
        ├── behaviors/mission.py
        │   ├── CollectObject
        │   ├── DeliverObject
        │   └── PlanReturnPath
        ├── behaviors/conditions.py
        │   └── IsPathComplete
        └── behaviors/obstacle.py
            └── HandleObstacle
```

## Behavior Tree Structure (Created by controller.py)

```
Mission (Sequence) 🎯
├── Nav To Pickup Complete (FailureIsSuccess)
│   └── Navigate Pickup Loop (Repeat × 999)
│       └── Try Cell (Retry × 5)
│           └── Navigate One Cell (Sequence)
│               ├── Get Next Waypoint ➡️
│               ├── Rotate To Target 🔄
│               └── Move Or Handle (Selector)
│                   ├── Move To Target ➡️
│                   └── Handle Obstacle 🚧
│
├── Collect Object 🤖
│
├── Plan Return Path 🗺️
│
├── Nav To Home Complete (FailureIsSuccess)
│   └── Navigate Home Loop (Repeat × 999)
│       └── Try Cell (Retry × 5)
│           └── Navigate One Cell (Sequence)
│               ├── Get Next Waypoint ➡️
│               ├── Rotate To Target 🔄
│               └── Move Or Handle (Selector)
│                   ├── Move To Target ➡️
│                   └── Handle Obstacle 🚧
│
└── Deliver Object 📦
```

## Data Flow

```
ROS2 Topics → BehaviorTreeController → Blackboard → Behaviors
              (controller.py)           (shared state)  (behaviors/*.py)

Topics:
  /odom          → robot pose (x, y, yaw)
  /scan          → LIDAR distance
  /apriltag_pose → localization correction
  /cmd_vel       ← movement commands

Blackboard Keys:
  - node                 → BehaviorTreeController instance
  - path_queue           → deque of (row, col) waypoints
  - current_target       → (row, col) of current cell
  - target_world_x/y     → target position in odom frame
  - target_yaw           → target orientation
  - rotation_start_time  → timestamp for rotation timeout
  - object_collected     → mission state flag
  - returning_home       → mission phase flag
```


## Benefits Summary

✅ **Modularity**: Each file has a single, clear responsibility  
✅ **Testability**: Behaviors can be unit tested independently  
✅ **Maintainability**: Easy to locate and modify specific functionality  
✅ **Reusability**: Behaviors can be imported into other projects  
✅ **Collaboration**: Multiple developers can work on different modules  
✅ **Debugging**: Smaller files are easier to navigate and understand  
✅ **IDE Support**: Better autocomplete, go-to-definition, find references  
✅ **Documentation**: Each module can have focused documentation  

## No Breaking Changes

🎉 The refactoring maintains **100% functional equivalence**:
- Same behavior tree structure
- Same ROS2 topics and messages
- Same control logic and parameters
- Same mission execution flow
- No changes to launch files
- No changes to dependencies
