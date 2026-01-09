"""Behavior tree controller for courier robot mission."""

import math
import rclpy
from rclpy.node import Node
from collections import deque

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

import py_trees
from py_trees import common

from .behaviors.navigation import RotateToTarget, MoveToTarget, GetNextWaypoint, CenterOnCell
from .behaviors.mission import CollectObject, DeliverObject, PlanReturnPath, PlanPath
from .behaviors.conditions import IsPathComplete
from .behaviors.obstacle import HandleObstacle
from .behaviors.battery import CheckBattery, ChargeBattery


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
        
        # === Battery State ===
        self.battery_level = 100.0
        
        # === Robot State (in ODOM frame) ===
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        # === AprilTag Localization ===
        self.last_apriltag_pose = None
        self.apriltag_correction_weight = 0.3  # 30% tag, 70% odometry
        self.last_apriltag_time = None
        
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
        self.apriltag_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/apriltag_pose', self.apriltag_callback, 10)
        
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
        self.get_logger().info(f'🔋 Battery: {self.battery_level:.0f}%')
        self.get_logger().info('='*50)

    def create_behavior_tree(self):
        """
        Create hierarchical behavior tree for courier mission.
        
        Tree Structure:
        Root (Sequence)
        ├── Navigate To Pickup (Loop until complete)
        ├── Collect Object
        ├── Plan Return Path
        ├── Navigate To Home (Loop until complete)
        └── Deliver Object
        """
        
        # Root sequence - execute mission steps in order
        root = py_trees.composites.Sequence(name="Mission", memory=True)
        
        # === PHASE 0: Plan initial path ===
        plan_path = PlanPath(name="Plan Path")

        # === PHASE 1: Navigate to pickup ===
        nav_to_pickup = py_trees.decorators.FailureIsSuccess(
            name="Nav To Pickup Complete",
            child=py_trees.decorators.Repeat(
                name="Navigate Pickup Loop",
                child=py_trees.decorators.Retry(
                    name="Try Cell",
                    child=self.create_navigate_one_cell(),
                    num_failures=5
                ),
                num_success=999
            )
        )
        
        # === PHASE 2: Collect object ===
        collect = CollectObject(name="Collect Object")
        
        # === PHASE 3: Plan return ===
        plan_return = PlanReturnPath(name="Plan Return Path")
        
        # === PHASE 4: Navigate home ===
        nav_to_home = py_trees.decorators.FailureIsSuccess(
            name="Nav To Home Complete",
            child=py_trees.decorators.Repeat(
                name="Navigate Home Loop",
                child=py_trees.decorators.Retry(
                    name="Try Cell",
                    child=self.create_navigate_one_cell(),
                    num_failures=5
                ),
                num_success=999
            )
        )
        
        # === PHASE 5: Deliver object ===
        deliver = DeliverObject(name="Deliver Object")
        
        # Assemble tree
        root.add_children([
            plan_path,
            nav_to_pickup,
            collect,
            plan_return,
            nav_to_home,
            deliver
        ])
        
        return root
    
    def create_navigate_one_cell(self):
        """
        Navigate to one cell with battery management:
        - Check battery level
        - If low, charge
        - Otherwise: get waypoint → rotate → move
        
        Returns FAILURE when path is empty (to exit repeat loop).
        Returns SUCCESS when cell reached.
        Returns FAILURE on obstacle (retry will replan).
        """
        # Main sequence: check battery → navigate
        nav_with_battery = py_trees.composites.Sequence(name="Navigate One Cell", memory=True)
        
        # Battery management: check → if low, charge
        battery_check = py_trees.composites.Selector(name="Battery Management", memory=False)
        check_battery = CheckBattery(name="Check Battery")
        charge_battery = ChargeBattery(name="Charge Battery")
        battery_check.add_children([check_battery, charge_battery])
        
        # Navigation sequence: get waypoint → rotate → move
        nav_sequence = py_trees.composites.Sequence(name="Navigate", memory=True)
        get_waypoint = GetNextWaypoint(name="Get Next Waypoint")
        rotate = RotateToTarget(name="Rotate To Target")  
        move = MoveToTarget(name="Move To Target")
        center = CenterOnCell(name="Center On Cell")
        
        # Add obstacle handling as a fallback
        move_with_fallback = py_trees.composites.Selector(name="Move Or Handle", memory=False)
        move_with_fallback.add_children([move, HandleObstacle(name="Handle Obstacle")])
        
        nav_sequence.add_children([get_waypoint, rotate, move_with_fallback, center])
        
        # Combine battery check with navigation
        nav_with_battery.add_children([battery_check, nav_sequence])
        
        return nav_with_battery

    def start_mission(self):
        """Calculate initial path and start behavior tree."""
        self.startup_timer.cancel()
        
        current_cell = self.world_to_cell(self.robot_x, self.robot_y)
        self.get_logger().info(f'Robot at ({self.robot_x:.2f}, {self.robot_y:.2f}) = cell {current_cell}')
        # Create and setup behavior tree; planning is handled by `PlanPath`
        self.tree = self.create_behavior_tree()
        self.tree.setup_with_descendants()

        self.get_logger().info('✅ Behavior tree created and initialized')
        self.get_logger().info('\n' + py_trees.display.unicode_tree(root=self.tree, show_status=True))
    
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
    # UTILITY METHODS
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

    def apriltag_callback(self, msg):
        """Apply AprilTag localization correction to reduce odometry drift."""
        # Extract pose from AprilTag detection
        tag_x = msg.pose.pose.position.x
        tag_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        tag_yaw = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)
        
        # Get covariance (confidence based on distance)
        cov_x = msg.pose.covariance[0]  # x variance
        cov_y = msg.pose.covariance[7]  # y variance
        avg_covariance = (cov_x + cov_y) / 2.0
        
        # Reject detections with medium-high uncertainty (very conservative)
        if avg_covariance > 0.25:
            return
        
        # Calculate dynamic weight based on tag confidence
        dynamic_weight = min(0.05, 0.0025 / avg_covariance) if avg_covariance > 0.01 else 0.05
        
        # Blend AprilTag pose with current odometry estimate
        self.robot_x = (1 - dynamic_weight) * self.robot_x + dynamic_weight * tag_x
        self.robot_y = (1 - dynamic_weight) * self.robot_y + dynamic_weight * tag_y
        
        # Angular correction with wraparound handling
        angle_diff = self.normalize_angle(tag_yaw - self.robot_yaw)
        self.robot_yaw = self.normalize_angle(self.robot_yaw + dynamic_weight * angle_diff)
        
        # Store for diagnostics
        self.last_apriltag_pose = (tag_x, tag_y, tag_yaw)
        self.last_apriltag_time = self.get_clock().now()
        
        # Log each successful correction
        self.get_logger().info(
            f'✓ AprilTag correction applied: '
            f'weight={dynamic_weight*100:.1f}% '
            f'pose=({self.robot_x:.2f}, {self.robot_y:.2f}, {math.degrees(self.robot_yaw):.0f}°) '
            f'cov={avg_covariance:.3f}'
        )

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
