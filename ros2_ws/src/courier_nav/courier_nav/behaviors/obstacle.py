"""Obstacle handling behavior."""

import time
import py_trees
from py_trees import common
from geometry_msgs.msg import Twist
from collections import deque


class HandleObstacle(py_trees.behaviour.Behaviour):
    """Back up, and replan path WITHOUT marking obstacle (transient only)."""
    
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="node", access=common.Access.READ)
        self.blackboard.register_key(key="path_queue", access=common.Access.WRITE)
        self.blackboard.register_key(key="current_target", access=common.Access.READ)
        self.blackboard.register_key(key="returning_home", access=common.Access.READ)
    
    def update(self):
        """Handle obstacle detection - back up and replan WITHOUT permanent marking."""
        node = self.blackboard.get("node")
        current_target = self.blackboard.get("current_target")
        returning_home = self.blackboard.get("returning_home")
        
        node.stop_robot()
        node.get_logger().info('🚧 Obstacle detected! Backing up...')
        
        # Back up with shorter distance
        cmd = Twist()
        cmd.linear.x = -0.15
        for _ in range(10):
            node.cmd_vel_pub.publish(cmd)
            time.sleep(0.05)
        node.stop_robot()
        
        # Get current position after backing up
        current_cell = node.world_to_cell(node.robot_x, node.robot_y)
        
        # ✨ IMPORTANT: Do NOT permanently mark obstacles
        # Just skip this cell temporarily in replanning
        # The permanent grid_map obstacles remain unchanged for consistent BFS
        
        # Determine replanning target
        if returning_home:
            target = node.start_cell
            node.get_logger().info(f'🔄 Replanning RETURN path from {current_cell} to {target}')
        else:
            target = node.goal_cell
            node.get_logger().info(f'🔄 Replanning path from {current_cell} to {target}')
        
        # Check if we're already at the target
        if current_cell == target:
            node.get_logger().info(f'✓ Already at target {target}! Continuing mission.')
            self.blackboard.set("path_queue", deque())
            return py_trees.common.Status.SUCCESS
        
        # Replan using original static obstacles ONLY
        new_path = node.bfs_path(current_cell, target)
        
        if new_path:
            self.blackboard.set("path_queue", deque(new_path))
            node.get_logger().info(f'✓ NEW PATH found: {new_path}')
            return py_trees.common.Status.SUCCESS
        else:
            node.get_logger().warn(f'❌ NO PATH AVAILABLE to {target}!')
            return py_trees.common.Status.FAILURE
