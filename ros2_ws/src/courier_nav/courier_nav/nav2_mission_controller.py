#!/usr/bin/env python3
"""
Cell-to-Cell Navigation Controller - STRICT ORTHOGONAL MOVEMENT

Il robot si muove SOLO in linea retta tra celle adiacenti:
1. STOP completo
2. ROTATE in place fino ad allineamento perfetto (0°, 90°, 180°, -90°)
3. MOVE STRAIGHT senza correzioni angolari
4. Ripeti per ogni cella

NO diagonal movements, NO curved paths.
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

import math
from enum import Enum
from collections import deque


class RobotState(Enum):
    WAITING_START = 0
    ROTATING = 1
    MOVING = 2
    REACHED_WAYPOINT = 3
    MISSION_COMPLETE = 4


class CellToCellController(Node):
    
    def __init__(self):
        super().__init__('cell_to_cell_controller')
        
        # === Grid Configuration ===
        self.cell_size = 1.0
        self.grid_size = 5
        
        # Obstacles as (row, col) - row is Y, col is X in world
        self.obstacles = {(1, 1), (1, 2), (3, 1), (3, 3)}
        
        # Mission: start (0,0) -> goal (4,2)
        self.start_cell = (0, 0)
        self.goal_cell = (4, 2)
        
        # === Robot State (in ODOM frame) ===
        # Robot spawns at world (0.5, 0.5) which is odom (0, 0)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        # === Navigation State ===
        self.state = RobotState.WAITING_START
        self.path_queue = deque()
        self.current_target = None
        self.target_world_x = 0.0
        self.target_world_y = 0.0
        self.target_yaw = 0.0  # The EXACT angle we need (0, π/2, π, -π/2)
        
        # === Control Parameters - STRICT ===
        self.rotation_speed = 0.5        # Increased
        self.linear_speed = 0.25         # Increased
        self.angle_tolerance = 0.08      # ~4.5 degrees (slightly looser for stability)
        self.position_tolerance = 0.12   # 12cm
        
        # Track rotation start to ensure we rotate at least a minimum time
        self.rotation_start_time = None
        
        # === LIDAR Parameters ===
        self.front_distance = 5.0        # Start with large value until LIDAR data received
        self.obstacle_threshold = 0.50   # Stop if obstacle closer than 50cm
        self.lidar_received = False      # Track if we got LIDAR data
        
        # === Publishers/Subscribers ===
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/path_markers', 10)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        # === Timers ===
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20Hz
        self.marker_timer = self.create_timer(1.0, self.publish_markers)
        self.lidar_check_timer = self.create_timer(2.0, self.check_lidar)  # Check LIDAR
        self.startup_timer = self.create_timer(3.0, self.start_mission)
        self.waypoint_timer = None  # One-shot timer for waypoint transitions
        
        self.get_logger().info('='*50)
        self.get_logger().info('STRICT ORTHOGONAL CONTROLLER')
        self.get_logger().info(f'Start: {self.start_cell} -> Goal: {self.goal_cell}')
        self.get_logger().info('='*50)

    def check_lidar(self):
        """Periodically check if LIDAR is working."""
        if not self.lidar_received:
            self.get_logger().warn('NO LIDAR DATA RECEIVED! Check /scan topic')
        else:
            self.get_logger().info(f'LIDAR OK: front_distance = {self.front_distance:.2f}m')

    def cell_to_world(self, row, col):
        """Convert grid cell (row, col) to ODOM frame coordinates.
        
        Robot spawns at world (0.5, 0.5) which is odom (0, 0).
        So odom_coord = world_coord - 0.5
        """
        # World coordinates (center of cell)
        world_x = (col + 0.5) * self.cell_size
        world_y = (row + 0.5) * self.cell_size
        # Convert to odom frame (robot spawns at world 0.5, 0.5 = odom 0, 0)
        odom_x = world_x - 0.5
        odom_y = world_y - 0.5
        return odom_x, odom_y
    
    def world_to_cell(self, odom_x, odom_y):
        """Convert ODOM frame coordinates to grid cell."""
        # Convert odom to world first
        world_x = odom_x + 0.5
        world_y = odom_y + 0.5
        col = int(world_x / self.cell_size)
        row = int(world_y / self.cell_size)
        return row, col

    def start_mission(self):
        """Calculate path and start navigation."""
        self.startup_timer.cancel()
        
        current_cell = self.world_to_cell(self.robot_x, self.robot_y)
        self.get_logger().info(f'Robot at ({self.robot_x:.2f}, {self.robot_y:.2f}) = cell {current_cell}')
        
        # Calculate BFS path
        path = self.bfs_path(current_cell, self.goal_cell)
        
        if path:
            self.path_queue = deque(path)
            self.get_logger().info(f'PATH FOUND with {len(path)} waypoints:')
            for i, cell in enumerate(path):
                wx, wy = self.cell_to_world(cell[0], cell[1])
                self.get_logger().info(f'   {i+1}. Cell{cell} -> World({wx:.2f}, {wy:.2f})')
            self.go_to_next_waypoint()
        else:
            self.get_logger().error('NO PATH FOUND!')
            self.state = RobotState.MISSION_COMPLETE

    def bfs_path(self, start, goal):
        """BFS pathfinding - ONLY 4 directions (no diagonals)."""
        if start == goal:
            return []
        
        # 4 directions ONLY: (row_delta, col_delta)
        # up=row+1, down=row-1, left=col-1, right=col+1
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

    def go_to_next_waypoint(self):
        """Set next target cell and calculate EXACT required angle."""
        if not self.path_queue:
            self.get_logger().info('='*50)
            self.get_logger().info('MISSION COMPLETE!')
            self.get_logger().info('='*50)
            self.stop_robot()
            self.state = RobotState.MISSION_COMPLETE
            return
        
        self.current_target = self.path_queue.popleft()
        row, col = self.current_target
        self.target_world_x, self.target_world_y = self.cell_to_world(row, col)
        
        # Calculate direction to target
        dx = self.target_world_x - self.robot_x
        dy = self.target_world_y - self.robot_y
        
        self.get_logger().info(f'Direction calc: dx={dx:.3f}, dy={dy:.3f}')
        
        # Determine EXACT cardinal direction (0°, 90°, 180°, -90°)
        # Use a threshold to avoid floating-point issues when dx or dy is near zero
        threshold = 0.05  # 5cm threshold
        
        if abs(dx) < threshold and abs(dy) < threshold:
            # Already at target? Keep current heading
            self.get_logger().warn(f'Already near target! Keeping current heading.')
            self.target_yaw = self.robot_yaw
        elif abs(dx) > abs(dy):
            # Moving primarily in X direction
            if dx > 0:
                self.target_yaw = 0.0  # East (+X)
            else:
                self.target_yaw = math.pi  # West (-X)
        else:
            # Moving primarily in Y direction
            if dy > 0:
                self.target_yaw = math.pi / 2  # North (+Y)
            else:
                self.target_yaw = -math.pi / 2  # South (-Y)
        
        self.get_logger().info('')
        self.get_logger().info(f'TARGET: Cell{self.current_target} = ({self.target_world_x:.2f}, {self.target_world_y:.2f})')
        self.get_logger().info(f'ROTATE TO: {math.degrees(self.target_yaw):.0f} deg (current: {math.degrees(self.robot_yaw):.0f} deg)')
        
        self.state = RobotState.ROTATING
        self.rotation_start_time = self.get_clock().now()

    def odom_callback(self, msg):
        """Update robot pose from odometry."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
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

    def control_loop(self):
        """Main control state machine - STRICT orthogonal movement."""
        if self.state == RobotState.WAITING_START:
            return
        
        if self.state == RobotState.MISSION_COMPLETE:
            return
        
        if self.state == RobotState.REACHED_WAYPOINT:
            return
        
        # SAFETY CHECK: Always check LIDAR first!
        if self.front_distance < self.obstacle_threshold:
            self.stop_robot()
            self.get_logger().warn(f'EMERGENCY STOP! Obstacle at {self.front_distance:.2f}m')
            self.handle_obstacle()
            return
        
        # BOUNDARY CHECK: Don't go outside grid (odometry frame: -0.5 to 4.5)
        # Robot spawns at world (0.5, 0.5) which is odom (0, 0)
        # Grid is 5x5, so valid range in odom frame is approximately -0.4 to 4.4
        if self.robot_x < -0.4 or self.robot_x > 4.4 or self.robot_y < -0.4 or self.robot_y > 4.4:
            self.stop_robot()
            self.get_logger().warn(f'BOUNDARY! Robot at ({self.robot_x:.2f}, {self.robot_y:.2f}) - backing up')
            self.handle_obstacle()
            return
        
        cmd = Twist()
        
        # === STATE: ROTATING ===
        if self.state == RobotState.ROTATING:
            angle_error = self.normalize_angle(self.target_yaw - self.robot_yaw)
            
            # Calculate time spent rotating
            rotation_elapsed = 0.0
            if self.rotation_start_time is not None:
                rotation_elapsed = (self.get_clock().now() - self.rotation_start_time).nanoseconds / 1e9
            
            self.get_logger().debug(f'ROTATING: target={math.degrees(self.target_yaw):.1f}° current={math.degrees(self.robot_yaw):.1f}° error={math.degrees(angle_error):.1f}° time={rotation_elapsed:.2f}s')
            
            # Only complete rotation after minimum time (0.2s) AND angle is correct
            # This prevents false "done" when angle hasn't been updated yet
            min_rotation_time = 0.2
            if abs(angle_error) < self.angle_tolerance and rotation_elapsed > min_rotation_time:
                # Rotation complete - FULL STOP before moving
                self.stop_robot()
                self.get_logger().info(f'ROTATION DONE! Yaw={math.degrees(self.robot_yaw):.1f}° target={math.degrees(self.target_yaw):.1f}° | LIDAR={self.front_distance:.2f}m')
                
                # CHECK LIDAR BEFORE MOVING!
                if self.front_distance < self.obstacle_threshold:
                    self.get_logger().warn(f'BLOCKED AHEAD! Distance={self.front_distance:.2f}m - Finding new path')
                    self.handle_obstacle()
                    return
                
                self.state = RobotState.MOVING
                self.get_logger().info('>>> STATE: MOVING')
                return
            
            # Rotate in place - NO linear velocity!
            cmd.linear.x = 0.0
            if angle_error > 0:
                cmd.angular.z = self.rotation_speed
            else:
                cmd.angular.z = -self.rotation_speed
            
            # Slow down when close to target angle
            if abs(angle_error) < 0.2:
                cmd.angular.z *= 0.5
        
        # === STATE: MOVING ===
        elif self.state == RobotState.MOVING:
            # CHECK OBSTACLE CONTINUOUSLY!
            if self.front_distance < self.obstacle_threshold:
                self.stop_robot()
                self.get_logger().warn(f'OBSTACLE at {self.front_distance:.2f}m! Stopping and recalculating...')
                self.handle_obstacle()
                return
            
            dx = self.target_world_x - self.robot_x
            dy = self.target_world_y - self.robot_y
            distance = math.sqrt(dx*dx + dy*dy)
            
            # Log periodically
            if not hasattr(self, '_last_log') or (self.get_clock().now().nanoseconds - self._last_log) > 500000000:
                self.get_logger().info(f'Moving: dist={distance:.2f}m | LIDAR={self.front_distance:.2f}m')
                self._last_log = self.get_clock().now().nanoseconds
            
            if distance < self.position_tolerance:
                # Waypoint reached - FULL STOP
                self.stop_robot()
                self.get_logger().info(f'REACHED Cell{self.current_target}!')
                self.state = RobotState.REACHED_WAYPOINT
                # Go to next waypoint after short delay (cancel any existing timer first)
                if self.waypoint_timer is not None:
                    self.waypoint_timer.cancel()
                self.waypoint_timer = self.create_timer(0.3, self.waypoint_reached_callback)
                return
            
            # Check if we've drifted off course
            angle_error = self.normalize_angle(self.target_yaw - self.robot_yaw)
            if abs(angle_error) > 0.15:  # ~8.5 degrees
                # Re-align!
                self.get_logger().warn(f'DRIFT! Re-aligning...')
                self.stop_robot()
                self.state = RobotState.ROTATING
                return
            
            # Move STRAIGHT - NO angular correction!
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0
            
            # Slow down when approaching target
            if distance < 0.25:
                cmd.linear.x *= 0.7
        
        self.cmd_vel_pub.publish(cmd)

    def waypoint_reached_callback(self):
        """Called after reaching a waypoint - one-shot timer callback."""
        # Cancel timer to make it one-shot
        if self.waypoint_timer is not None:
            self.waypoint_timer.cancel()
            self.waypoint_timer = None
        
        if self.state == RobotState.REACHED_WAYPOINT:
            self.go_to_next_waypoint()

    def handle_obstacle(self):
        """Handle obstacle - back up, mark cell as blocked and recalculate path."""
        self.stop_robot()
        # Back up a little
        self.get_logger().info('Backing up...')
        cmd = Twist()
        cmd.linear.x = -0.15
        import time
        for _ in range(15):
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.05)
        self.stop_robot()

        # Only mark the cell we tried to move into as blocked, never the start cell or all border cells
        if self.current_target and self.current_target != self.start_cell:
            self.obstacles.add(self.current_target)
            self.get_logger().info(f'Marked Cell{self.current_target} as blocked')

        current_cell = self.world_to_cell(self.robot_x, self.robot_y)
        self.get_logger().info(f'Current position: ({self.robot_x:.2f}, {self.robot_y:.2f}) = Cell{current_cell}')
        self.get_logger().info(f'Obstacles now: {self.obstacles}')

        new_path = self.bfs_path(current_cell, self.goal_cell)
        if new_path:
            self.path_queue = deque(new_path)
            self.get_logger().info(f'NEW PATH: {list(self.path_queue)}')
            self.go_to_next_waypoint()
        else:
            self.get_logger().error('NO PATH AVAILABLE! Mission failed.')
            self.state = RobotState.MISSION_COMPLETE

    def stop_robot(self):
        """Stop all motion."""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        for _ in range(3):
            self.cmd_vel_pub.publish(cmd)

    def publish_markers(self):
        """Publish visualization markers."""
        marker_array = MarkerArray()
        
        # Current target marker
        if self.current_target:
            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'target'
            marker.id = 0
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            wx, wy = self.cell_to_world(self.current_target[0], self.current_target[1])
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
        for i, cell in enumerate(self.path_queue):
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
        
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = CellToCellController()
    
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
