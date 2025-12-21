#!/usr/bin/env python3
"""
Cell-to-Cell Navigation Controller - BEHAVIOR TREE ARCHITECTURE

This is a complete rewrite of the mission controller using py_trees behavior tree.
All low-level control logic (rotation, movement, LIDAR) remains identical,
but organized hierarchically instead of as a flat state machine.

Architecture:
- Root Sequence: Navigate → Collect → Plan Return → Navigate Home → Deliver
- Each navigation phase repeats: Get Waypoint → Rotate → Move (with obstacle handling)
- Blackboard shares state between behaviors
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

import math
import time
from collections import deque

import py_trees
from py_trees import common


# ============================================================================
# BEHAVIOR TREE LEAF BEHAVIORS
# ============================================================================

class RotateToTarget(py_trees.behaviour.Behaviour):
    """Rotate robot to target yaw angle with strict orthogonal alignment."""
    
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="node", access=common.Access.READ)
        self.blackboard.register_key(key="target_yaw", access=common.Access.READ)
        self.blackboard.register_key(key="rotation_start_time", access=common.Access.WRITE)
        
    def initialise(self):
        """Called when behavior starts."""
        node = self.blackboard.get("node")
        self.blackboard.set("rotation_start_time", node.get_clock().now())
        
    def update(self):
        """Execute rotation logic."""
        node = self.blackboard.get("node")
        target_yaw = self.blackboard.get("target_yaw")
        rotation_start = self.blackboard.get("rotation_start_time")
        
        angle_error = node.normalize_angle(target_yaw - node.robot_yaw)
        rotation_elapsed = (node.get_clock().now() - rotation_start).nanoseconds / 1e9
        
        node.get_logger().debug(
            f'ROTATING: target={math.degrees(target_yaw):.1f}° '
            f'current={math.degrees(node.robot_yaw):.1f}° '
            f'error={math.degrees(angle_error):.1f}° '
            f'time={rotation_elapsed:.2f}s'
        )
        
        # Check if rotation complete (minimum time + angle tolerance)
        min_rotation_time = 0.2
        if abs(angle_error) < node.angle_tolerance and rotation_elapsed > min_rotation_time:
            node.stop_robot()
            node.get_logger().info(f'ROTATION DONE! Yaw={math.degrees(node.robot_yaw):.1f}°')
            
            # Check LIDAR before moving
            if node.front_distance < node.obstacle_threshold:
                node.get_logger().warn(f'BLOCKED AHEAD! Distance={node.front_distance:.2f}m')
                return py_trees.common.Status.FAILURE  # Trigger obstacle handling
            
            return py_trees.common.Status.SUCCESS
        
        # Continue rotating
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = node.rotation_speed if angle_error > 0 else -node.rotation_speed
        
        # Slow down when close
        if abs(angle_error) < 0.2:
            cmd.angular.z *= 0.5
        
        node.cmd_vel_pub.publish(cmd)
        return py_trees.common.Status.RUNNING


class MoveToTarget(py_trees.behaviour.Behaviour):
    """Move robot straight to target position (no angular correction during movement)."""
    
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="node", access=common.Access.READ)
        self.blackboard.register_key(key="target_world_x", access=common.Access.READ)
        self.blackboard.register_key(key="target_world_y", access=common.Access.READ)
        self.blackboard.register_key(key="target_yaw", access=common.Access.READ)
        self.blackboard.register_key(key="current_target", access=common.Access.READ)
        
    def update(self):
        """Execute movement logic."""
        node = self.blackboard.get("node")
        target_x = self.blackboard.get("target_world_x")
        target_y = self.blackboard.get("target_world_y")
        target_yaw = self.blackboard.get("target_yaw")
        current_target = self.blackboard.get("current_target")
        
        # Check obstacle
        if node.front_distance < node.obstacle_threshold:
            node.stop_robot()
            node.get_logger().warn(f'OBSTACLE at {node.front_distance:.2f}m!')
            return py_trees.common.Status.FAILURE
        
        # Check boundaries
        if node.robot_x < -0.4 or node.robot_x > 4.4 or node.robot_y < -0.4 or node.robot_y > 4.4:
            node.stop_robot()
            node.get_logger().warn(f'BOUNDARY! Robot at ({node.robot_x:.2f}, {node.robot_y:.2f})')
            return py_trees.common.Status.FAILURE
        
        # Calculate distance
        dx = target_x - node.robot_x
        dy = target_y - node.robot_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Log less frequently (every 2s)
        if not hasattr(node, '_last_log') or (node.get_clock().now().nanoseconds - node._last_log) > 2_000_000_000:
            node.get_logger().debug(f'Moving: dist={distance:.2f}m | LIDAR={node.front_distance:.2f}m')
            node._last_log = node.get_clock().now().nanoseconds
        
        # Check if reached
        if distance < node.position_tolerance:
            node.stop_robot()
            node.get_logger().info(f'REACHED Cell{current_target}!')
            return py_trees.common.Status.SUCCESS
        
        # Check drift - if drifted too much, fail to trigger re-rotation
        angle_error = node.normalize_angle(target_yaw - node.robot_yaw)
        if abs(angle_error) > 0.15:  # ~8.5 degrees
            node.get_logger().debug('DRIFT detected - need realignment')
            node.stop_robot()
            return py_trees.common.Status.FAILURE
        
        # Move straight - NO angular correction
        cmd = Twist()
        cmd.linear.x = node.linear_speed
        cmd.angular.z = 0.0
        
        # Slow down when approaching
        if distance < 0.25:
            cmd.linear.x *= 0.7
        
        node.cmd_vel_pub.publish(cmd)
        return py_trees.common.Status.RUNNING


class GetNextWaypoint(py_trees.behaviour.Behaviour):
    """Pop next waypoint from path queue and calculate target angle."""
    
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="node", access=common.Access.READ)
        self.blackboard.register_key(key="path_queue", access=common.Access.WRITE)
        self.blackboard.register_key(key="current_target", access=common.Access.WRITE)
        self.blackboard.register_key(key="target_world_x", access=common.Access.WRITE)
        self.blackboard.register_key(key="target_world_y", access=common.Access.WRITE)
        self.blackboard.register_key(key="target_yaw", access=common.Access.WRITE)
        
    def update(self):
        """Get next waypoint and calculate cardinal direction."""
        node = self.blackboard.get("node")
        path_queue = self.blackboard.get("path_queue")
        
        if not path_queue:
            return py_trees.common.Status.FAILURE
        
        # Pop next waypoint
        current_target = path_queue.popleft()
        self.blackboard.set("path_queue", path_queue)
        self.blackboard.set("current_target", current_target)
        
        row, col = current_target
        target_x, target_y = node.cell_to_world(row, col)
        self.blackboard.set("target_world_x", target_x)
        self.blackboard.set("target_world_y", target_y)
        
        # Calculate target yaw - use grid delta for precision
        try:
            current_cell = node.world_to_cell(node.robot_x, node.robot_y)
        except Exception:
            current_cell = None
        
        dx = target_x - node.robot_x
        dy = target_y - node.robot_y
        
        if current_cell is not None:
            crow, ccol = current_cell
            drow = row - crow
            dcol = col - ccol
            
            if dcol != 0:
                target_yaw = 0.0 if dcol > 0 else math.pi
            elif drow != 0:
                target_yaw = math.pi / 2 if drow > 0 else -math.pi / 2
            else:
                # Fallback to world coordinates
                if abs(dx) > abs(dy):
                    target_yaw = 0.0 if dx > 0 else math.pi
                else:
                    target_yaw = math.pi / 2 if dy > 0 else -math.pi / 2
        else:
            # Fallback if world_to_cell failed
            if abs(dx) > abs(dy):
                target_yaw = 0.0 if dx > 0 else math.pi
            else:
                target_yaw = math.pi / 2 if dy > 0 else -math.pi / 2
        
        self.blackboard.set("target_yaw", target_yaw)
        
        node.get_logger().debug(f'TARGET: Cell{current_target} = ({target_x:.2f}, {target_y:.2f})')
        node.get_logger().info(f'ROTATE TO: {math.degrees(target_yaw):.0f} deg')
        
        return py_trees.common.Status.SUCCESS


class IsPathComplete(py_trees.behaviour.Behaviour):
    """Check if path queue is empty."""
    
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="path_queue", access=common.Access.READ)
        
    def update(self):
        path_queue = self.blackboard.get("path_queue")
        if not path_queue:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class CollectObject(py_trees.behaviour.Behaviour):
    """4-second gripper animation for object collection."""
    
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="node", access=common.Access.READ)
        self.blackboard.register_key(key="object_collected", access=common.Access.WRITE)
        self.animation_start = None
        self.animation_step = 0
        
    def initialise(self):
        """Start animation."""
        node = self.blackboard.get("node")
        self.animation_start = node.get_clock().now()
        self.animation_step = 0
        node.get_logger().info('='*50)
        node.get_logger().info('REACHED GOAL! Collecting object...')
        node.get_logger().info('='*50)
        node.stop_robot()
        
    def update(self):
        """Run animation steps."""
        node = self.blackboard.get("node")
        
        if self.animation_start is None:
            return py_trees.common.Status.FAILURE
        
        elapsed = (node.get_clock().now() - self.animation_start).nanoseconds / 1e9
        
        if elapsed < 1.0 and self.animation_step == 0:
            node.get_logger().info('🤖 Activating gripper...')
            node.get_logger().info('   Opening gripper...')
            self.animation_step = 1
        elif 1.0 <= elapsed < 2.0 and self.animation_step == 1:
            node.get_logger().info('   Lowering arm...')
            self.animation_step = 2
        elif 2.0 <= elapsed < 3.0 and self.animation_step == 2:
            node.get_logger().info('   Closing gripper...')
            self.animation_step = 3
        elif 3.0 <= elapsed < 4.0 and self.animation_step == 3:
            node.get_logger().info('   Lifting arm...')
            self.animation_step = 4
        elif elapsed >= 4.0 and self.animation_step == 4:
            node.get_logger().info('✅ Object collected!')
            self.blackboard.set("object_collected", True)
            return py_trees.common.Status.SUCCESS
        
        return py_trees.common.Status.RUNNING


class DeliverObject(py_trees.behaviour.Behaviour):
    """4-second gripper animation for object delivery."""
    
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="node", access=common.Access.READ)
        self.animation_start = None
        self.animation_step = 0
        
    def initialise(self):
        """Start animation."""
        node = self.blackboard.get("node")
        self.animation_start = node.get_clock().now()
        self.animation_step = 0
        node.get_logger().info('='*50)
        node.get_logger().info('RETURNED HOME! Delivering object...')
        node.get_logger().info('='*50)
        node.stop_robot()
        
    def update(self):
        """Run animation steps."""
        node = self.blackboard.get("node")
        
        if self.animation_start is None:
            return py_trees.common.Status.FAILURE
        
        elapsed = (node.get_clock().now() - self.animation_start).nanoseconds / 1e9
        
        if elapsed < 1.0 and self.animation_step == 0:
            node.get_logger().info('📦 Delivering object...')
            node.get_logger().info('   Lowering arm...')
            self.animation_step = 1
        elif 1.0 <= elapsed < 2.0 and self.animation_step == 1:
            node.get_logger().info('   Opening gripper...')
            self.animation_step = 2
        elif 2.0 <= elapsed < 3.0 and self.animation_step == 2:
            node.get_logger().info('   Releasing object...')
            self.animation_step = 3
        elif 3.0 <= elapsed < 4.0 and self.animation_step == 3:
            node.get_logger().info('   Raising arm...')
            self.animation_step = 4
        elif elapsed >= 4.0 and self.animation_step == 4:
            node.get_logger().info('✅ Object delivered!')
            node.get_logger().info('='*50)
            node.get_logger().info('🎉 MISSION COMPLETE!')
            node.get_logger().info('='*50)
            return py_trees.common.Status.SUCCESS
        
        return py_trees.common.Status.RUNNING


class PlanReturnPath(py_trees.behaviour.Behaviour):
    """Plan BFS path back to home."""
    
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="node", access=common.Access.READ)
        self.blackboard.register_key(key="path_queue", access=common.Access.WRITE)
        self.blackboard.register_key(key="returning_home", access=common.Access.WRITE)
        
    def update(self):
        """Calculate return path."""
        node = self.blackboard.get("node")
        
        node.get_logger().info('='*50)
        node.get_logger().info('PLANNING RETURN PATH TO HOME')
        node.get_logger().info('='*50)
        
        current_cell = node.world_to_cell(node.robot_x, node.robot_y)
        node.get_logger().info(f'Current: {current_cell} -> Home: {node.start_cell}')
        
        path = node.bfs_path(current_cell, node.start_cell)
        
        if path:
            self.blackboard.set("path_queue", deque(path))
            self.blackboard.set("returning_home", True)
            node.get_logger().info(f'RETURN PATH FOUND with {len(path)} waypoints')
            for i, cell in enumerate(path):
                wx, wy = node.cell_to_world(cell[0], cell[1])
                node.get_logger().info(f'  {i+1}. Cell{cell} -> ({wx:.2f}, {wy:.2f})')
            return py_trees.common.Status.SUCCESS
        else:
            node.get_logger().error('NO RETURN PATH FOUND!')
            return py_trees.common.Status.FAILURE


class HandleObstacle(py_trees.behaviour.Behaviour):
    """Back up, mark cell blocked, and replan path."""
    
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="node", access=common.Access.READ)
        self.blackboard.register_key(key="path_queue", access=common.Access.WRITE)
        self.blackboard.register_key(key="current_target", access=common.Access.READ)
        self.blackboard.register_key(key="returning_home", access=common.Access.READ)
        
    def update(self):
        """Handle obstacle detection."""
        node = self.blackboard.get("node")
        current_target = self.blackboard.get("current_target")
        returning_home = self.blackboard.get("returning_home")
        
        node.stop_robot()
        node.get_logger().info('Backing up...')
        
        # Back up
        cmd = Twist()
        cmd.linear.x = -0.15
        for _ in range(15):
            node.cmd_vel_pub.publish(cmd)
            time.sleep(0.05)
        node.stop_robot()
        
        # Mark cell as blocked (never block start cell)
        if current_target and current_target != node.start_cell:
            node.obstacles.add(current_target)
            node.get_logger().info(f'Marked Cell{current_target} as blocked')
        
        current_cell = node.world_to_cell(node.robot_x, node.robot_y)
        node.get_logger().info(f'Current position: ({node.robot_x:.2f}, {node.robot_y:.2f}) = Cell{current_cell}')
        node.get_logger().info(f'Obstacles now: {node.obstacles}')
        
        # Determine target based on mission phase
        if returning_home:
            target = node.start_cell
            node.get_logger().info(f'Replanning RETURN path to {target}')
        else:
            target = node.goal_cell
            node.get_logger().info(f'Replanning path to goal {target}')
        
        # Replan
        new_path = node.bfs_path(current_cell, target)
        if new_path:
            self.blackboard.set("path_queue", deque(new_path))
            node.get_logger().info(f'NEW PATH: {list(new_path)}')
            return py_trees.common.Status.SUCCESS
        else:
            node.get_logger().warn(f'NO PATH AVAILABLE to {target}!')
            return py_trees.common.Status.FAILURE


# ============================================================================
# ROS2 NODE WITH BEHAVIOR TREE
# ============================================================================

class BehaviorTreeController(Node):
    """ROS2 node that runs a behavior tree for courier robot mission."""
    
    def __init__(self):
        super().__init__('behavior_tree_controller')
        
        # === Grid Configuration ===
        self.cell_size = 1.0
        self.grid_size = 5
        self.obstacles = {(1, 1), (1, 2), (3, 1), (3, 3)}
        
        # Mission parameters
        self.start_cell = (0, 0)
        self.goal_cell = (4, 2)
        
        # === Robot State (in ODOM frame) ===
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        # === Control Parameters ===
        self.rotation_speed = 0.5
        self.linear_speed = 0.25
        self.angle_tolerance = 0.08      # ~4.5 degrees
        self.position_tolerance = 0.12   # 12cm
        
        # === LIDAR Parameters ===
        self.front_distance = 5.0
        self.obstacle_threshold = 0.50
        self.lidar_received = False
        
        # === Publishers/Subscribers ===
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/path_markers', 10)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        # === Behavior Tree Setup ===
        self.tree = None
        self.blackboard = py_trees.blackboard.Client(name="CourierRobot")
        self.blackboard.register_key(key="node", access=common.Access.WRITE)
        self.blackboard.register_key(key="path_queue", access=common.Access.WRITE)
        self.blackboard.register_key(key="current_target", access=common.Access.WRITE)
        self.blackboard.register_key(key="target_world_x", access=common.Access.WRITE)
        self.blackboard.register_key(key="target_world_y", access=common.Access.WRITE)
        self.blackboard.register_key(key="target_yaw", access=common.Access.WRITE)
        self.blackboard.register_key(key="rotation_start_time", access=common.Access.WRITE)
        self.blackboard.register_key(key="object_collected", access=common.Access.WRITE)
        self.blackboard.register_key(key="returning_home", access=common.Access.WRITE)
        
        # Initialize blackboard
        self.blackboard.set("node", self)
        self.blackboard.set("path_queue", deque())
        self.blackboard.set("object_collected", False)
        self.blackboard.set("returning_home", False)
        
        # === Timers ===
        self.tree_timer = self.create_timer(0.05, self.tick_tree)  # 20Hz
        self.marker_timer = self.create_timer(1.0, self.publish_markers)
        self.lidar_check_timer = self.create_timer(2.0, self.check_lidar)
        self.startup_timer = self.create_timer(3.0, self.start_mission)
        
        self.get_logger().info('='*50)
        self.get_logger().info('COURIER ROBOT - BEHAVIOR TREE CONTROLLER')
        self.get_logger().info(f'Mission: Start {self.start_cell} -> Pickup {self.goal_cell} -> Return {self.start_cell}')
        self.get_logger().info('='*50)

    def create_behavior_tree(self):
        """
        Create hierarchical behavior tree for courier mission.
        
        Tree Structure:
        Root (Sequence)
        ├── Navigate To Pickup (Repeat until path empty)
        │   └── Move To Cell (with obstacle handling)
        ├── Collect Object
        ├── Plan Return Path
        ├── Navigate To Home (Repeat until path empty)
        │   └── Move To Cell (with obstacle handling)
        └── Deliver Object
        """
        
        # Root sequence - execute mission steps in order
        root = py_trees.composites.Sequence(name="Mission", memory=True)
        
        # === PHASE 1: Navigate to pickup ===
        # Repeat navigation until path is complete
        nav_to_pickup = py_trees.decorators.Retry(
            name="Navigate To Pickup",
            child=self.create_move_to_cell_subtree(),
            num_failures=100  # Keep trying with replanning
        )
        
        # Wrap to keep running until path is empty
        repeat_pickup = py_trees.decorators.SuccessIsRunning(
            name="Repeat Until Pickup",
            child=nav_to_pickup
        )
        
        # === PHASE 2: Collect object ===
        collect = CollectObject(name="Collect Object")
        
        # === PHASE 3: Plan return ===
        plan_return = PlanReturnPath(name="Plan Return Path")
        
        # === PHASE 4: Navigate home ===
        nav_to_home = py_trees.decorators.Retry(
            name="Navigate To Home",
            child=self.create_move_to_cell_subtree(),
            num_failures=100
        )
        
        repeat_home = py_trees.decorators.SuccessIsRunning(
            name="Repeat Until Home",
            child=nav_to_home
        )
        
        # === PHASE 5: Deliver object ===
        deliver = DeliverObject(name="Deliver Object")
        
        # Assemble tree
        root.add_children([
            repeat_pickup,
            collect,
            plan_return,
            repeat_home,
            deliver
        ])
        
        return root
    
    def create_move_to_cell_subtree(self):
        """
        Create subtree for moving to one cell.
        
        Logic:
        - If path is complete (empty): return SUCCESS to exit repeat loop
        - Else: try navigation, handle obstacles if navigation fails
        """
        
        # Check if path is complete
        path_check = IsPathComplete(name="Path Complete?")
        
        # Selector: try normal navigation OR handle obstacle
        nav_or_handle = py_trees.composites.Selector(name="Nav Or Handle Obstacle", memory=False)
        
        # Normal navigation sequence: get waypoint → rotate → move
        # CRITICAL: memory=True so sequence remembers progress during RUNNING states
        nav_sequence = py_trees.composites.Sequence(name="Navigate Cell", memory=True)
        get_waypoint = GetNextWaypoint(name="Get Next Waypoint")
        rotate = RotateToTarget(name="Rotate To Target")
        move = MoveToTarget(name="Move To Target")
        nav_sequence.add_children([get_waypoint, rotate, move])
        
        # Obstacle handling (runs if navigation fails)
        handle_obs = HandleObstacle(name="Handle Obstacle")
        
        nav_or_handle.add_children([nav_sequence, handle_obs])
        
        # Combine: if path complete return SUCCESS, otherwise navigate
        # When IsPathComplete returns SUCCESS → Selector returns SUCCESS (exit loop)
        # When IsPathComplete returns FAILURE → Selector tries nav_or_handle
        move_cell = py_trees.composites.Selector(name="Move One Cell", memory=False)
        move_cell.add_children([path_check, nav_or_handle])
        
        return move_cell

    def start_mission(self):
        """Calculate initial path and start behavior tree."""
        self.startup_timer.cancel()
        
        current_cell = self.world_to_cell(self.robot_x, self.robot_y)
        self.get_logger().info(f'Robot at ({self.robot_x:.2f}, {self.robot_y:.2f}) = cell {current_cell}')
        
        # Calculate BFS path
        path = self.bfs_path(current_cell, self.goal_cell)
        
        if path:
            self.blackboard.set("path_queue", deque(path))
            self.get_logger().info(f'PATH FOUND with {len(path)} waypoints')
            for i, cell in enumerate(path):
                wx, wy = self.cell_to_world(cell[0], cell[1])
                self.get_logger().debug(f'   {i+1}. Cell{cell} -> World({wx:.2f}, {wy:.2f})')
            
            # Create and setup behavior tree
            self.tree = self.create_behavior_tree()
            self.tree.setup_with_descendants()
            
            self.get_logger().info('✅ Behavior tree created and initialized')
            self.get_logger().info('\n' + py_trees.display.unicode_tree(root=self.tree, show_status=True))
        else:
            self.get_logger().error('NO PATH FOUND!')
    
    def tick_tree(self):
        """Tick the behavior tree at 20Hz."""
        if self.tree is None:
            return
        
        # Tick tree once
        self.tree.tick_once()
        
        # Check if mission complete
        if self.tree.status == py_trees.common.Status.SUCCESS:
            self.get_logger().info('✅ Behavior tree completed successfully!')
            self.tree = None  # Stop ticking

    # ========================================================================
    # UTILITY METHODS (unchanged from state machine version)
    # ========================================================================

    def check_lidar(self):
        """Periodically check if LIDAR is working."""
        if not self.lidar_received:
            self.get_logger().warn('NO LIDAR DATA RECEIVED! Check /scan topic')
        else:
            self.get_logger().info(f'LIDAR OK: front_distance = {self.front_distance:.2f}m')
    
    def cell_to_world(self, row, col):
        """Convert grid cell (row, col) to ODOM frame coordinates."""
        world_x = (col + 0.5) * self.cell_size
        world_y = (row + 0.5) * self.cell_size
        odom_x = world_x - 0.5
        odom_y = world_y - 0.5
        return odom_x, odom_y
    
    def world_to_cell(self, odom_x, odom_y):
        """Convert ODOM frame coordinates to grid cell."""
        world_x = odom_x + 0.5
        world_y = odom_y + 0.5
        col = int(world_x / self.cell_size)
        row = int(world_y / self.cell_size)
        return row, col
    
    def bfs_path(self, start, goal):
        """BFS pathfinding - ONLY 4 directions (no diagonals)."""
        if start == goal:
            return []
        
        directions = [(1, 0), (-1, 0), (0, -1), (0, 1)]
        queue = deque([(start, [start])])
        visited = {start}
        
        while queue:
            (row, col), path = queue.popleft()
            
            for drow, dcol in directions:
                nrow, ncol = row + drow, col + dcol
                next_cell = (nrow, ncol)
                
                if not (0 <= nrow < self.grid_size and 0 <= ncol < self.grid_size):
                    continue
                
                if next_cell in self.obstacles or next_cell in visited:
                    continue
                
                new_path = path + [next_cell]
                
                if next_cell == goal:
                    return new_path[1:]  # Exclude start
                
                visited.add(next_cell)
                queue.append((next_cell, new_path))
        
        return []

    def odom_callback(self, msg):
        """Update robot pose from odometry."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def scan_callback(self, msg):
        """Process LIDAR for obstacle detection."""
        if len(msg.ranges) == 0:
            return
        
        self.lidar_received = True
        num_readings = len(msg.ranges)
        angle_increment = msg.angle_increment
        angle_min = msg.angle_min
        
        # Front sector: -30° to +30°
        start_idx = int((-0.52 - angle_min) / angle_increment)
        end_idx = int((0.52 - angle_min) / angle_increment)
        
        start_idx = max(0, min(start_idx, num_readings - 1))
        end_idx = max(0, min(end_idx, num_readings - 1))
        
        valid_ranges = []
        for i in range(start_idx, end_idx + 1):
            r = msg.ranges[i]
            if msg.range_min < r < msg.range_max:
                valid_ranges.append(r)
        
        if valid_ranges:
            self.front_distance = min(valid_ranges)
        else:
            self.front_distance = float('inf')

    def stop_robot(self):
        """Stop all motion."""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        for _ in range(3):
            try:
                if hasattr(rclpy, 'ok') and rclpy.ok():
                    self.cmd_vel_pub.publish(cmd)
                else:
                    break
            except Exception as e:
                try:
                    self.get_logger().debug(f'stop_robot: publish failed: {e}')
                except Exception:
                    pass
                break

    def publish_markers(self):
        """Publish visualization markers for current target and path."""
        marker_array = MarkerArray()
        
        # Get current target from blackboard
        try:
            current_target = self.blackboard.get("current_target")
        except:
            current_target = None
        
        # Current target marker
        if current_target:
            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'target'
            marker.id = 0
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            wx, wy = self.cell_to_world(current_target[0], current_target[1])
            marker.pose.position.x = wx
            marker.pose.position.y = wy
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.2
            
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            marker_array.markers.append(marker)
        
        # Path markers
        try:
            path_queue = self.blackboard.get("path_queue")
            for i, cell in enumerate(path_queue):
                marker = Marker()
                marker.header.frame_id = 'odom'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = 'path'
                marker.id = i + 1
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                
                wx, wy = self.cell_to_world(cell[0], cell[1])
                marker.pose.position.x = wx
                marker.pose.position.y = wy
                marker.pose.position.z = 0.15
                marker.pose.orientation.w = 1.0
                
                marker.scale.x = 0.15
                marker.scale.y = 0.15
                marker.scale.z = 0.15
                
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.8
                
                marker_array.markers.append(marker)
        except:
            pass
        
        self.marker_pub.publish(marker_array)


# ============================================================================
# MAIN
# ============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = BehaviorTreeController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
