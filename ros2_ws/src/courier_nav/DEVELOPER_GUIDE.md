# Quick Reference Guide - Modular Behavior Tree Controller

## 🚀 Quick Start

### Run the Controller
```bash
ros2 run courier_nav nav2_mission_controller_bt
```

### Rebuild After Changes
```bash
cd ros2_ws
colcon build --packages-select courier_nav
source install/setup.bash  # Linux
# or
.\install\setup.ps1  # Windows
```

## 📁 Where to Find Things

### Want to modify...
| What | File | Line Range |
|------|------|------------|
| **Rotation logic** | `behaviors/navigation.py` | 9-68 |
| **Movement logic** | `behaviors/navigation.py` | 71-175 |
| **Waypoint processing** | `behaviors/navigation.py` | 178-233 |
| **Collection animation** | `behaviors/mission.py` | 9-59 |
| **Delivery animation** | `behaviors/mission.py` | 62-117 |
| **Path planning** | `behaviors/mission.py` | 120-148 |
| **Obstacle handling** | `behaviors/obstacle.py` | 10-62 |
| **Condition checks** | `behaviors/conditions.py` | 8-19 |
| **Behavior tree structure** | `controller.py` | 107-182 |
| **Grid parameters** | `controller.py` | 29-37 |
| **Control parameters** | `controller.py` | 48-52 |
| **LIDAR processing** | `controller.py` | 383-410 |
| **Odometry handling** | `controller.py` | 316-323 |
| **AprilTag fusion** | `controller.py` | 325-363 |
| **BFS pathfinding** | `controller.py` | 288-314 |

## 🔧 Common Modifications

### Change Rotation Speed
**File**: `controller.py`  
**Line**: 48
```python
self.rotation_speed = 0.5  # Change this value
```

### Change Movement Speed
**File**: `controller.py`  
**Line**: 49
```python
self.linear_speed = 0.25  # Change this value
```

### Modify Grid Obstacles
**File**: `controller.py`  
**Line**: 31
```python
self.obstacles = {(1, 1), (1, 2), (3, 1), (3, 3)}  # Add/remove cells
```

### Change Start/Goal Positions
**File**: `controller.py`  
**Lines**: 34-35
```python
self.start_cell = (0, 0)  # Home position
self.goal_cell = (4, 2)   # Pickup location
```

### Adjust Collection Animation
**File**: `behaviors/mission.py`  
**Lines**: 29-57
```python
# Modify animation steps and timing
if elapsed < 1.0 and self.animation_step == 0:
    # First animation step
```

### Change Obstacle Threshold
**File**: `controller.py`  
**Line**: 55
```python
self.obstacle_threshold = 0.50  # Distance in meters
```

### Modify Drift Tolerance
**File**: `behaviors/navigation.py`  
**Line**: 156
```python
if abs(angle_error) > 0.15:  # ~8.5 degrees - adjust this
```

## 🧪 Testing Individual Behaviors

### Test Navigation Behavior
```python
from courier_nav.behaviors.navigation import RotateToTarget

# Create test behavior
behavior = RotateToTarget(name="test_rotate")
# Test logic...
```

### Test Mission Behavior
```python
from courier_nav.behaviors.mission import CollectObject

# Create test behavior
behavior = CollectObject(name="test_collect")
# Test logic...
```

## 🐛 Debugging Tips

### Enable Debug Logging
Modify logging level in your launch file or:
```python
node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
```

### Monitor Blackboard State
```python
# In controller.py tick_tree() method
print(self.blackboard.get("path_queue"))
print(self.blackboard.get("current_target"))
```

### Visualize Behavior Tree
The tree structure is logged at startup:
```
[INFO] ✅ Behavior tree created and initialized
[INFO] 
Mission [○]
├── Nav To Pickup Complete [○]
│   └── Navigate Pickup Loop [○]
...
```

### Check ROS2 Topics
```bash
# List active topics
ros2 topic list

# Monitor velocity commands
ros2 topic echo /cmd_vel

# Check LIDAR data
ros2 topic echo /scan --no-arr

# View odometry
ros2 topic echo /odom
```

## 📊 Understanding Behavior Status

| Status | Symbol | Meaning |
|--------|--------|---------|
| SUCCESS | ✓ | Behavior completed successfully |
| FAILURE | ✗ | Behavior failed (might retry) |
| RUNNING | ○ | Behavior is executing |

## 🔄 Behavior Tree Flow

### Navigation Phase
1. `GetNextWaypoint` pops cell from queue → SUCCESS
2. `RotateToTarget` rotates to cardinal direction → SUCCESS
3. `MoveToTarget` moves straight → SUCCESS or FAILURE
4. If FAILURE → `HandleObstacle` backs up and replans → SUCCESS
5. Repeat until path_queue is empty

### Mission Phase
1. Path complete → `CollectObject` (4 seconds) → SUCCESS
2. `PlanReturnPath` calculates return route → SUCCESS
3. Navigate home (same as navigation phase)
4. `DeliverObject` (4 seconds) → SUCCESS
5. Mission complete!

## 🎯 Blackboard Keys Reference

| Key | Type | Purpose |
|-----|------|---------|
| `node` | BehaviorTreeController | Reference to ROS2 node |
| `path_queue` | deque[(row, col)] | Remaining waypoints |
| `current_target` | (row, col) | Cell being navigated to |
| `target_world_x` | float | Target X in odom frame |
| `target_world_y` | float | Target Y in odom frame |
| `target_yaw` | float | Target orientation (radians) |
| `rotation_start_time` | Time | When rotation began |
| `object_collected` | bool | Has object been collected? |
| `returning_home` | bool | In return phase? |

## 📦 Adding New Behaviors

### 1. Create New Behavior File
```python
# behaviors/my_new_behavior.py
import py_trees
from py_trees import common

class MyNewBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        # Register blackboard keys...
        
    def update(self):
        # Your logic here
        return py_trees.common.Status.SUCCESS
```

### 2. Add to __init__.py
```python
# behaviors/__init__.py
from .my_new_behavior import MyNewBehavior

__all__ = [
    # ... existing exports
    'MyNewBehavior',
]
```

### 3. Use in Controller
```python
# controller.py
from .behaviors.my_new_behavior import MyNewBehavior

# In create_behavior_tree():
my_behavior = MyNewBehavior(name="Do Something")
root.add_child(my_behavior)
```

## 🔍 Useful ROS2 Commands

```bash
# Run with logging
ros2 run courier_nav nav2_mission_controller_bt --ros-args --log-level debug

# Check node info
ros2 node info /behavior_tree_controller

# List parameters
ros2 param list /behavior_tree_controller

# Monitor transforms
ros2 run tf2_ros tf2_echo odom base_link

# Record rosbag
ros2 bag record -a

# Visualize in RViz
ros2 run rviz2 rviz2
```

## 📝 Code Style Guidelines

- **Docstrings**: Every class and method should have a docstring
- **Type hints**: Use type hints for function parameters
- **Line length**: Keep lines under 100 characters where possible
- **Imports**: Group stdlib → ROS2 → local
- **Naming**: 
  - Classes: `PascalCase`
  - Functions/methods: `snake_case`
  - Constants: `UPPER_SNAKE_CASE`
  - Private: prefix with `_`

## 🚨 Common Issues & Solutions

### Import Error
```
ModuleNotFoundError: No module named 'courier_nav.behaviors'
```
**Solution**: Rebuild package with `colcon build`

### Behavior Not Executing
**Solution**: Check blackboard key registration and access permissions

### Robot Not Moving
**Solution**: Check `/cmd_vel` topic with `ros2 topic echo /cmd_vel`

### Path Not Found
**Solution**: Check obstacle configuration and start/goal cells

### Tree Stuck in RUNNING
**Solution**: Add debug logging to identify which behavior is stuck

## 📚 Further Reading

- [py_trees Documentation](https://py-trees.readthedocs.io/)
- [ROS2 Python Client](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [Behavior Trees Primer](https://www.behaviortree.dev/)

## 💡 Pro Tips

1. **Use `memory=True`** in Sequences to resume after FAILURE
2. **Log frequently** during development, reduce for production
3. **Test behaviors individually** before integrating
4. **Use decorators** (Retry, Repeat) for robust execution
5. **Keep behaviors atomic** - one responsibility per behavior
6. **Document assumptions** in docstrings
7. **Use blackboard** for state, not instance variables
