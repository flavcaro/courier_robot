#!/usr/bin/env python3
"""
Nav2 Mission Controller - Cell-to-Cell Navigation

Navigazione diretta cell-to-cell SENZA percorsi diagonali:
1. Il robot si ferma
2. Si ruota verso la cella target
3. Controlla se ci sono ostacoli davanti
4. Se libero, avanza dritto alla cella successiva
5. Se ostacolo, ricalcola percorso alternativo

Questo approccio evita i percorsi diagonali generati da Nav2.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

import math
import time
from enum import Enum
from collections import deque


class RobotState(Enum):
    IDLE = 0
    ROTATING = 1
    CHECKING_PATH = 2
    MOVING_FORWARD = 3
    WAITING_OBSTACLE = 4
    ARRIVED = 5
    MISSION_COMPLETE = 6


class CellToCellController(Node):
    """
    Controller per navigazione cell-to-cell senza diagonali.
    """
    
    def __init__(self):
        super().__init__('cell_to_cell_controller')
        
        # === Configuration ===
        self.cell_size = 1.0
        self.grid_size = 5
        
        # Obstacles
        self.obstacles = {(1, 1), (2, 1), (1, 3), (3, 3)}
        
        # Mission
        self.start_cell = (0, 0)
        self.goal_cell = (2, 4)
        
        # === Publishers/Subscribers ===
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/grid_markers', 10)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.apriltag_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/apriltag_pose', self.apriltag_callback, 10)
        
        # === Robot State ===
        self.current_x = 0.5
        self.current_y = 0.5
        self.current_yaw = 0.0
        
        self.state = RobotState.IDLE
        self.current_cell = self.start_cell
        self.target_cell = None
        self.path_queue = deque()
        
        # === LIDAR Data ===
        self.front_distance = float('inf')
        self.front_clear = True
        self.min_safe_distance = 0.45  # Distanza minima sicura
        
        # === AprilTag ===
        self.apriltag_x = None
        self.apriltag_y = None
        self.apriltag_time = None
        
        # === Control Parameters ===
        self.rotation_speed = 0.5  # rad/s
        self.linear_speed = 0.2   # m/s
        self.angle_tolerance = 0.1  # rad (~6 degrees)
        self.position_tolerance = 0.15  # meters
        
        # === Timers ===
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20 Hz
        self.marker_timer = self.create_timer(1.0, self.publish_markers)
        
        # Delay start to allow sensors to initialize
        self.startup_timer = self.create_timer(2.0, self.start_mission)
        
        self.get_logger().info('='*60)
        self.get_logger().info('CELL-TO-CELL NAVIGATION CONTROLLER')
        self.get_logger().info(f'Start: {self.start_cell} -> Goal: {self.goal_cell}')
        self.get_logger().info(f'Obstacles: {self.obstacles}')
        self.get_logger().info('='*60)
    
    def start_mission(self):
        """Start the mission after initial delay."""
        self.startup_timer.cancel()
        
        # Calculate initial path
        self.path_queue = deque(self.calculate_path(self.current_cell, self.goal_cell))
        
        if self.path_queue:
            self.get_logger().info(f'Path calculated: {list(self.path_queue)}')
            self.state = RobotState.IDLE
            self.next_waypoint()
        else:
            self.get_logger().error('No path found to goal!')
            self.state = RobotState.MISSION_COMPLETE
    
    def calculate_path(self, start, goal):
        """
        BFS pathfinding che genera solo movimenti ortogonali (no diagonali).
        """
        if start == goal:
            return []
        
        # BFS
        queue = deque([(start, [start])])
        visited = {start}
        
        # Solo 4 direzioni: su, giù, sinistra, destra (NO diagonali)
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        
        while queue:
            current, path = queue.popleft()
            
            for dx, dy in directions:
                nx, ny = current[0] + dx, current[1] + dy
                next_cell = (nx, ny)
                
                # Check bounds
                if not (0 <= nx < self.grid_size and 0 <= ny < self.grid_size):
                    continue
                
                # Check if visited or obstacle
                if next_cell in visited or next_cell in self.obstacles:
                    continue
                
                new_path = path + [next_cell]
                
                if next_cell == goal:
                    return new_path[1:]  # Exclude start cell
                
                visited.add(next_cell)
                queue.append((next_cell, new_path))
        
        return []  # No path found
    
    def next_waypoint(self):
        """Set next waypoint from queue."""
        if not self.path_queue:
            self.get_logger().info('PATH COMPLETED - Arrived at goal!')
            self.state = RobotState.MISSION_COMPLETE
            self.stop_robot()
            return
        
        self.target_cell = self.path_queue.popleft()
        self.get_logger().info(f'Next waypoint: {self.target_cell} (remaining: {len(self.path_queue)})')
        self.state = RobotState.ROTATING
    
    def odom_callback(self, msg):
        """Update robot position from odometry."""
        pos = msg.pose.pose.position
        self.current_x = pos.x
        self.current_y = pos.y
        
        # Yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Update current cell
        self.current_cell = (
            int(self.current_x / self.cell_size),
            int(self.current_y / self.cell_size)
        )
    
    def scan_callback(self, msg):
        """Process LIDAR scan."""
        if len(msg.ranges) == 0:
            return
        
        num_readings = len(msg.ranges)
        angle_increment = msg.angle_increment
        angle_min = msg.angle_min
        
        # Front sector: -30° to +30° (stretto, solo davanti)
        front_start = int((-0.52 - angle_min) / angle_increment)
        front_end = int((0.52 - angle_min) / angle_increment)
        
        front_start = max(0, min(front_start, num_readings - 1))
        front_end = max(0, min(front_end, num_readings - 1))
        
        valid_ranges = []
        for i in range(front_start, front_end + 1):
            r = msg.ranges[i]
            if msg.range_min < r < msg.range_max:
                valid_ranges.append(r)
        
        self.front_distance = min(valid_ranges) if valid_ranges else float('inf')
        self.front_clear = self.front_distance > self.min_safe_distance
    
    def apriltag_callback(self, msg):
        """Receive AprilTag position."""
        self.apriltag_x = msg.pose.pose.position.x
        self.apriltag_y = msg.pose.pose.position.y
        self.apriltag_time = self.get_clock().now()
    
    def get_best_position(self):
        """Get best position estimate (AprilTag if recent, else odometry)."""
        if self.apriltag_time is not None:
            age = (self.get_clock().now() - self.apriltag_time).nanoseconds / 1e9
            if age < 1.0 and self.apriltag_x is not None:
                return self.apriltag_x, self.apriltag_y
        return self.current_x, self.current_y
    
    def control_loop(self):
        """Main control loop - state machine."""
        if self.state == RobotState.MISSION_COMPLETE:
            return
        
        if self.state == RobotState.IDLE:
            self.next_waypoint()
            return
        
        if self.target_cell is None:
            return
        
        # Target position (center of cell)
        target_x = (self.target_cell[0] + 0.5) * self.cell_size
        target_y = (self.target_cell[1] + 0.5) * self.cell_size
        
        # Current position
        curr_x, curr_y = self.get_best_position()
        
        # Vector to target
        dx = target_x - curr_x
        dy = target_y - curr_y
        distance = math.sqrt(dx*dx + dy*dy)
        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - self.current_yaw)
        
        cmd = Twist()
        
        # === STATE MACHINE ===
        
        if self.state == RobotState.ROTATING:
            # Fase 1: Ruota verso il target
            if abs(angle_error) < self.angle_tolerance:
                self.get_logger().info(f'Rotation complete. Angle error: {math.degrees(angle_error):.1f}°')
                self.state = RobotState.CHECKING_PATH
            else:
                cmd.angular.z = self.rotation_speed if angle_error > 0 else -self.rotation_speed
                # Slow down when close to target angle
                if abs(angle_error) < 0.3:
                    cmd.angular.z *= 0.5
        
        elif self.state == RobotState.CHECKING_PATH:
            # Fase 2: Controlla se il percorso è libero
            if self.front_clear:
                self.get_logger().info(f'Path clear (distance: {self.front_distance:.2f}m). Moving forward.')
                self.state = RobotState.MOVING_FORWARD
            else:
                self.get_logger().warn(f'OBSTACLE DETECTED at {self.front_distance:.2f}m! Finding alternate path...')
                self.state = RobotState.WAITING_OBSTACLE
                self.handle_obstacle()
        
        elif self.state == RobotState.MOVING_FORWARD:
            # Fase 3: Avanza dritto
            
            # Check if arrived
            if distance < self.position_tolerance:
                self.get_logger().info(f'Arrived at cell {self.target_cell}')
                self.stop_robot()
                self.state = RobotState.ARRIVED
                # Small delay then next waypoint
                self.create_timer(0.5, self.arrived_callback, callback_group=None)
                return
            
            # Check for obstacle while moving
            if not self.front_clear:
                self.get_logger().warn(f'OBSTACLE while moving! Distance: {self.front_distance:.2f}m')
                self.stop_robot()
                self.state = RobotState.WAITING_OBSTACLE
                self.handle_obstacle()
                return
            
            # Move forward with minor corrections
            cmd.linear.x = self.linear_speed
            
            # Adjust speed based on distance to target
            if distance < 0.3:
                cmd.linear.x *= 0.5
            
            # Small angular correction to stay on course
            if abs(angle_error) > 0.05:
                cmd.angular.z = angle_error * 0.8
        
        elif self.state == RobotState.ARRIVED:
            # Waiting for callback
            pass
        
        elif self.state == RobotState.WAITING_OBSTACLE:
            # Handled by handle_obstacle()
            pass
        
        self.cmd_vel_pub.publish(cmd)
    
    def arrived_callback(self):
        """Called when arrived at waypoint."""
        if self.state == RobotState.ARRIVED:
            self.state = RobotState.IDLE
    
    def handle_obstacle(self):
        """Handle obstacle - recalculate path."""
        self.stop_robot()
        
        # Mark current target as temporarily blocked
        blocked_cell = self.target_cell
        self.get_logger().info(f'Cell {blocked_cell} blocked. Recalculating path...')
        
        # Add blocked cell to obstacles temporarily
        temp_obstacles = self.obstacles.copy()
        temp_obstacles.add(blocked_cell)
        
        # Store original obstacles
        original_obstacles = self.obstacles
        self.obstacles = temp_obstacles
        
        # Recalculate path from current position
        new_path = self.calculate_path(self.current_cell, self.goal_cell)
        
        # Restore obstacles
        self.obstacles = original_obstacles
        
        if new_path:
            self.path_queue = deque(new_path)
            self.get_logger().info(f'New path found: {list(self.path_queue)}')
            self.state = RobotState.IDLE
        else:
            self.get_logger().error('No alternative path found! Waiting...')
            # Wait and retry
            self.create_timer(2.0, self.retry_pathfinding)
    
    def retry_pathfinding(self):
        """Retry pathfinding after waiting."""
        if self.state == RobotState.WAITING_OBSTACLE:
            self.get_logger().info('Retrying pathfinding...')
            new_path = self.calculate_path(self.current_cell, self.goal_cell)
            if new_path:
                self.path_queue = deque(new_path)
                self.get_logger().info(f'Path found on retry: {list(self.path_queue)}')
                self.state = RobotState.IDLE
            else:
                self.get_logger().error('Still no path. Mission failed.')
                self.state = RobotState.MISSION_COMPLETE
    
    def stop_robot(self):
        """Stop the robot."""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def publish_markers(self):
        """Publish visualization markers."""
        marker_array = MarkerArray()
        marker_id = 0
        
        for x in range(self.grid_size):
            for y in range(self.grid_size):
                marker = Marker()
                marker.header.frame_id = "odom"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "grid"
                marker.id = marker_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                
                marker.pose.position.x = (x + 0.5) * self.cell_size
                marker.pose.position.y = (y + 0.5) * self.cell_size
                marker.pose.position.z = 0.025
                marker.pose.orientation.w = 1.0
                
                marker.scale.x = self.cell_size * 0.9
                marker.scale.y = self.cell_size * 0.9
                marker.scale.z = 0.05
                
                cell = (x, y)
                if cell in self.obstacles:
                    marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0
                    marker.color.a = 0.8
                elif cell == self.start_cell:
                    marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.0
                    marker.color.a = 0.8
                elif cell == self.goal_cell:
                    marker.color.r, marker.color.g, marker.color.b = 0.0, 0.0, 1.0
                    marker.color.a = 0.8
                elif self.target_cell and cell == self.target_cell:
                    marker.color.r, marker.color.g, marker.color.b = 1.0, 1.0, 0.0
                    marker.color.a = 0.8
                else:
                    marker.color.r, marker.color.g, marker.color.b = 0.8, 0.8, 0.8
                    marker.color.a = 0.3
                
                marker_array.markers.append(marker)
                marker_id += 1
        
        # Path visualization
        if self.path_queue:
            for i, cell in enumerate(self.path_queue):
                marker = Marker()
                marker.header.frame_id = "odom"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "path"
                marker.id = marker_id
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                
                marker.pose.position.x = (cell[0] + 0.5) * self.cell_size
                marker.pose.position.y = (cell[1] + 0.5) * self.cell_size
                marker.pose.position.z = 0.2
                marker.pose.orientation.w = 1.0
                
                marker.scale.x = marker.scale.y = marker.scale.z = 0.15
                
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 1.0
                marker.color.a = 0.8
                
                marker_array.markers.append(marker)
                marker_id += 1
        
        # Current state text
        text_marker = Marker()
        text_marker.header.frame_id = "odom"
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = "status"
        text_marker.id = marker_id
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        
        text_marker.pose.position.x = 2.5
        text_marker.pose.position.y = -0.5
        text_marker.pose.position.z = 1.0
        text_marker.pose.orientation.w = 1.0
        
        text_marker.scale.z = 0.3
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        
        text_marker.text = f"State: {self.state.name}\nTarget: {self.target_cell}\nFront: {self.front_distance:.2f}m"
        
        marker_array.markers.append(text_marker)
        
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
