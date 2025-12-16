"""
Mission Controller Node
Manages complete courier mission with Behavior Tree
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import py_trees
from ros2_ws.src.courier_nav.mission_bt import create_mission_behavior_tree


class MissionController(Node):
    """
    Mission Controller using Behavior Tree
    Executes full courier mission: Init → Navigate → Pickup → Return → Release
    """
    
    def __init__(self):
        super().__init__('mission_controller')
        
        # Publishers
        self.target_pub = self.create_publisher(PoseStamped, '/target_pose', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Current state
        self.current_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.system_ready = False
        
        # Behavior Tree
        self.behavior_tree = create_mission_behavior_tree()
        self.behavior_tree.setup_with_descendants()
        
        # Blackboard
        self.blackboard = py_trees.blackboard.Client(name="MissionController")
        self.blackboard.register_key(key="system_ready", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="distance_to_target", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="distance_to_base", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="target_publisher", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="logger", access=py_trees.common.Access.WRITE)
        
        # Set initial blackboard values
        self.blackboard.set("system_ready", True)
        self.blackboard.set("target_publisher", self.publish_target)
        self.blackboard.set("logger", self.get_logger())
        
        # Timer for BT ticks
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Mission Controller started')
    
    def odom_callback(self, msg):
        """Update current pose from odometry"""
        self.current_pose['x'] = msg.pose.pose.position.x
        self.current_pose['y'] = msg.pose.pose.position.y
        # TODO: Extract theta from quaternion
    
    def publish_target(self, target_x, target_y):
        """Publish target pose for PID controller"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = target_x
        msg.pose.position.y = target_y
        msg.pose.position.z = 0.0
        self.target_pub.publish(msg)
    
    def control_loop(self):
        """Tick behavior tree"""
        if not self.system_ready:
            # Wait for system initialization
            if self.current_pose['x'] != 0.0 or self.current_pose['y'] != 0.0:
                self.system_ready = True
                self.blackboard.set("system_ready", True)
            return
        
        # Tick the behavior tree
        self.behavior_tree.tick_once()


def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()