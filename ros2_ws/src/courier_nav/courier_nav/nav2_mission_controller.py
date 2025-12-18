#!/usr/bin/env python3
"""
Nav2 Mission Controller with Behavior Tree

This node uses a py_trees behavior tree for high-level mission logic
while delegating navigation to Nav2's action servers.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

import py_trees
from courier_nav.courier_behavior_tree import create_courier_behavior_tree

import math


class Nav2MissionController(Node):
    """
    Mission controller that combines py_trees behavior tree with Nav2 navigation.
    """
    
    def __init__(self):
        super().__init__('nav2_mission_controller')
        
        # === Configuration ===
        self.cell_size = 1.0  # meters per grid cell
        
        # Grid map (0=free, 1=obstacle)
        self.grid_map = [
            [0, 0, 0, 0, 0],
            [0, 1, 1, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 1, 0, 1, 0],
            [0, 0, 0, 0, 0]
        ]
        
        # Mission waypoints
        self.start_cell = (0, 0)
        self.goal_cell = (4, 2)  # Pickup location
        
        # Path to pickup (Nav2 will handle the actual path planning,
        # but we can specify intermediate waypoints if needed)
        self.initial_path = [self.goal_cell]  # Just the destination
        
        # === Nav2 Action Client ===
        self.nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # === Subscribers ===
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # === Publishers ===
        self.marker_pub = self.create_publisher(MarkerArray, '/grid_markers', 10)
        
        # === Robot State ===
        self.current_pose = None
        
        # === Wait for Nav2 ===
        self.get_logger().info('Waiting for Nav2 action server...')
        self.nav2_client.wait_for_server()
        self.get_logger().info('Nav2 action server available!')
        
        # === Behavior Tree Setup ===
        self.setup_behavior_tree()
        
        # === Timers ===
        self.bt_timer = self.create_timer(0.1, self.tick_behavior_tree)  # 10 Hz
        self.marker_timer = self.create_timer(1.0, self.publish_markers)
        
        self.get_logger().info('='*50)
        self.get_logger().info('COURIER MISSION CONTROLLER STARTED')
        self.get_logger().info(f'Start: {self.start_cell}')
        self.get_logger().info(f'Pickup: {self.goal_cell}')
        self.get_logger().info('='*50)
        
    def setup_behavior_tree(self):
        """Initialize the behavior tree and blackboard."""
        
        # Create behavior tree
        self.behavior_tree = create_courier_behavior_tree()
        
        # Setup blackboard
        self.blackboard = py_trees.blackboard.Client(name="MissionController")
        
        # Register all keys we need to write
        keys_to_register = [
            "battery_level",
            "current_target",
            "path_queue",
            "cell_size",
            "nav2_client",
            "node",
            "logger",
            "object_collected",
            "return_planned",
            "mission_complete",
            "grid_map",
            "start_cell",
            "goal_cell",
            "robot_position",
        ]
        
        for key in keys_to_register:
            self.blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)
        
        # Initialize blackboard values
        self.blackboard.set("battery_level", 100.0)
        self.blackboard.set("current_target", None)
        self.blackboard.set("path_queue", list(self.initial_path))
        self.blackboard.set("cell_size", self.cell_size)
        self.blackboard.set("nav2_client", self.nav2_client)
        self.blackboard.set("node", self)
        self.blackboard.set("logger", self.get_logger())
        self.blackboard.set("object_collected", False)
        self.blackboard.set("return_planned", False)
        self.blackboard.set("mission_complete", False)
        self.blackboard.set("grid_map", self.grid_map)
        self.blackboard.set("start_cell", self.start_cell)
        self.blackboard.set("goal_cell", self.goal_cell)
        self.blackboard.set("robot_position", (0.5, 0.5))
        
        # Setup the tree
        self.behavior_tree.setup_with_descendants()
        
        self.get_logger().info('Behavior tree initialized')
        self.get_logger().info('\n' + py_trees.display.unicode_tree(root=self.behavior_tree))
    
    def odom_callback(self, msg):
        """Update robot position from odometry."""
        self.current_pose = msg.pose.pose
        pos = self.current_pose.position
        self.blackboard.set("robot_position", (pos.x, pos.y))
        
    def tick_behavior_tree(self):
        """Tick the behavior tree once."""
        # Check if mission is complete
        if self.blackboard.get("mission_complete"):
            self.get_logger().info('Mission complete! Stopping behavior tree.')
            self.bt_timer.cancel()
            return
            
        # Tick the tree
        self.behavior_tree.tick_once()
        
        # Log tree status periodically (every 5 seconds)
        # Uncomment for debugging:
        # self.get_logger().debug(f'Tree status: {self.behavior_tree.status}')
        
    def publish_markers(self):
        """Publish visualization markers for RViz."""
        marker_array = MarkerArray()
        rows = len(self.grid_map)
        cols = len(self.grid_map[0])
        
        marker_id = 0
        
        for r in range(rows):
            for c in range(cols):
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "grid"
                marker.id = marker_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                
                # Position at center of cell
                marker.pose.position.x = c * self.cell_size + 0.5
                marker.pose.position.y = r * self.cell_size + 0.5
                marker.pose.position.z = 0.025
                marker.pose.orientation.w = 1.0
                
                # Size
                marker.scale.x = self.cell_size * 0.9
                marker.scale.y = self.cell_size * 0.9
                marker.scale.z = 0.05
                
                # Color based on cell type
                if self.grid_map[r][c] == 1:
                    # Obstacle - Red
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                    marker.color.a = 0.8
                elif (r, c) == self.start_cell:
                    # Start - Green
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                    marker.color.a = 0.8
                elif (r, c) == self.goal_cell:
                    # Goal - Blue
                    marker.color.r = 0.0
                    marker.color.g = 0.0
                    marker.color.b = 1.0
                    marker.color.a = 0.8
                else:
                    # Free - Light gray
                    marker.color.r = 0.8
                    marker.color.g = 0.8
                    marker.color.b = 0.8
                    marker.color.a = 0.3
                    
                marker_array.markers.append(marker)
                marker_id += 1
                
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    
    node = Nav2MissionController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down...')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
