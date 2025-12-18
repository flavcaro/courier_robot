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
        
        # Coordinate system: cell (x, y) maps to world position (x+0.5, y+0.5)
        # x = column (0-4 left to right)
        # y = row (0-4 bottom to top)
        #
        # Visual grid (y increases upward):
        #   y=4: [ ][ ][G][ ][ ]   G = Goal (2,4)
        #   y=3: [ ][X][ ][X][ ]   X = Obstacle at (1,3), (3,3)
        #   y=2: [ ][ ][ ][ ][ ]
        #   y=1: [ ][X][X][ ][ ]   X = Obstacle at (1,1), (2,1)
        #   y=0: [S][ ][ ][ ][ ]   S = Start (0,0)
        #        x=0 x=1 x=2 x=3 x=4
        
        # Obstacles at: (1,1), (2,1), (1,3), (3,3)
        self.obstacles = [(1, 1), (2, 1), (1, 3), (3, 3)]
        
        # Mission waypoints
        self.start_cell = (0, 0)   # Bottom-left corner
        self.goal_cell = (2, 4)    # Top middle area (pickup location)
        
        # Path to pickup - go cell by cell through safe cells
        # Safe path: (0,0) -> (0,1) -> (0,2) -> (0,3) -> (0,4) -> (1,4) -> (2,4)
        # Goes up column 0 (safe) then moves right to goal at y=4
        self.initial_path = [
            (0, 1),  # Move up along x=0
            (0, 2),  # Continue up
            (0, 3),  # Continue up  
            (0, 4),  # Top-left corner
            (1, 4),  # Move right along y=4
            (2, 4),  # Arrive at goal
        ]
        
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
            "obstacles",
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
        self.blackboard.set("obstacles", self.obstacles)
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
        
        marker_id = 0
        
        # Grid is 5x5, iterate over all cells
        for x in range(5):
            for y in range(5):
                marker = Marker()
                marker.header.frame_id = "odom"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "grid"
                marker.id = marker_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                
                # Position at center of cell (x, y) -> world (x+0.5, y+0.5)
                marker.pose.position.x = (x + 0.5) * self.cell_size
                marker.pose.position.y = (y + 0.5) * self.cell_size
                marker.pose.position.z = 0.025
                marker.pose.orientation.w = 1.0
                
                # Size
                marker.scale.x = self.cell_size * 0.9
                marker.scale.y = self.cell_size * 0.9
                marker.scale.z = 0.05
                
                # Color based on cell type
                if (x, y) in self.obstacles:
                    # Obstacle - Red
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                    marker.color.a = 0.8
                elif (x, y) == self.start_cell:
                    # Start - Green
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                    marker.color.a = 0.8
                elif (x, y) == self.goal_cell:
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
