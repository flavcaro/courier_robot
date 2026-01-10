"""Mission-related behaviors for object collection and delivery."""

import py_trees
from py_trees import common
from collections import deque
import math
from geometry_msgs.msg import Twist


class AlignWithAprilTag(py_trees.behaviour.Behaviour):
    """Use AprilTag detection to perfectly align robot before gripper operation."""
    
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="node", access=common.Access.READ)
        self.start_time = None
        self.max_wait_time = 10.0  # Wait max 10 seconds for AprilTag
        self.search_phase = True  # First rotate to find tag
        self.alignment_done = False
        self.rotation_start_yaw = None
        
    def initialise(self):
        """Start AprilTag alignment."""
        node = self.blackboard.get("node")
        self.start_time = node.get_clock().now()
        self.alignment_done = False
        self.search_phase = True
        self.rotation_start_yaw = node.robot_yaw
        node.get_logger().info('🎯 Searching for AprilTag (360° scan)...')
        node.stop_robot()
        
    def update(self):
        """Align using AprilTag correction."""
        node = self.blackboard.get("node")
        
        elapsed = (node.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        # Check if we have recent AprilTag data
        has_recent_tag = False
        tag_age = None
        if node.last_apriltag_pose is not None and node.last_apriltag_time is not None:
            tag_age = (node.get_clock().now() - node.last_apriltag_time).nanoseconds / 1e9
            has_recent_tag = (tag_age < 1.0)
        
        # PHASE 1: Search for AprilTag by rotating
        if self.search_phase:
            if has_recent_tag:
                # Found a tag! Switch to alignment phase
                node.stop_robot()
                self.search_phase = False
                node.get_logger().info(f'✓ AprilTag detected (age={tag_age:.1f}s)! Starting precise alignment...')
                return py_trees.common.Status.RUNNING
            
            # Keep rotating slowly to search for tags
            rotated = abs(node.normalize_angle(node.robot_yaw - self.rotation_start_yaw))
            
            if rotated < 6.28:  # Full 360 degrees
                cmd = Twist()
                cmd.angular.z = 0.3  # Slow rotation for search
                node.cmd_vel_pub.publish(cmd)
                
                # Log search status every 2 seconds
                if int(elapsed * 2) != int((elapsed - 0.05) * 2):
                    if node.last_apriltag_time is None:
                        node.get_logger().info(f'🔍 Scanning... {math.degrees(rotated):.0f}°/360° - NO AprilTag data ever received')
                    else:
                        node.get_logger().info(f'🔍 Scanning... {math.degrees(rotated):.0f}°/360° - Last AprilTag: {tag_age:.1f}s ago')
                
                return py_trees.common.Status.RUNNING
            else:
                # Completed full rotation, no tag found
                node.stop_robot()
                if node.last_apriltag_time is None:
                    node.get_logger().warn('⚠️  AprilTag localizer may not be working (no data received)')
                else:
                    node.get_logger().warn(f'⚠️  No AprilTag visible (last seen {tag_age:.1f}s ago)')
                node.get_logger().warn('    Proceeding with odometry position')
                return py_trees.common.Status.SUCCESS
        
        # PHASE 2: Precise alignment using AprilTag
        if has_recent_tag:
            tag_x, tag_y, tag_yaw = node.last_apriltag_pose
            
            # Compute error from current position
            error_x = tag_x - node.robot_x
            error_y = tag_y - node.robot_y
            error_yaw = node.normalize_angle(tag_yaw - node.robot_yaw)
            
            position_error = math.hypot(error_x, error_y)
            
            # Tight tolerances for final alignment
            pos_tolerance = 0.02  # 2cm
            angle_tolerance = 0.05  # ~3 degrees
            
            if position_error < pos_tolerance and abs(error_yaw) < angle_tolerance:
                node.stop_robot()
                node.get_logger().info(f'✅ AprilTag alignment complete! pos_err={position_error*100:.1f}cm, yaw_err={math.degrees(error_yaw):.1f}°')
                return py_trees.common.Status.SUCCESS
            
            # Apply corrective motion
            cmd = Twist()
            
            # Rotate robot frame errors to world frame
            cy = math.cos(node.robot_yaw)
            sy = math.sin(node.robot_yaw)
            x_r =  cy * error_x + sy * error_y
            y_r = -sy * error_x + cy * error_y
            
            # Proportional control
            k_linear = 0.3
            k_angular = 0.8
            
            cmd.linear.x = max(-0.05, min(0.05, k_linear * x_r))
            cmd.angular.z = max(-0.2, min(0.2, k_angular * error_yaw))
            
            node.cmd_vel_pub.publish(cmd)
            
            node.get_logger().info(
                f'🎯 Aligning: pos_err={position_error*100:.1f}cm, '
                f'yaw_err={math.degrees(error_yaw):.1f}°'
            )
            
            return py_trees.common.Status.RUNNING
        
        # If no recent AprilTag or timeout
        if elapsed > self.max_wait_time:
            node.stop_robot()
            node.get_logger().warn('⚠️  AprilTag alignment timeout, proceeding with odometry position')
            return py_trees.common.Status.SUCCESS
        
        return py_trees.common.Status.RUNNING


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


class PlanPath(py_trees.behaviour.Behaviour):
    """Plan BFS path from start to goal."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="node", access=common.Access.READ)
        self.blackboard.register_key(key="path_queue", access=common.Access.WRITE)
        self.blackboard.register_key(key="returning_home", access=common.Access.WRITE)

    def update(self):
        """Calculate path from start cell to goal cell."""
        node = self.blackboard.get("node")

        node.get_logger().info('='*50)
        node.get_logger().info('PLANNING PATH FROM START TO GOAL')
        node.get_logger().info('='*50)

        node.get_logger().info(f'Start: {node.start_cell} -> Goal: {node.goal_cell}')

        path = node.bfs_path(node.start_cell, node.goal_cell)

        if path:
            self.blackboard.set("path_queue", deque(path))
            self.blackboard.set("returning_home", False)
            node.get_logger().info(f'PATH FOUND with {len(path)} waypoints')
            for i, cell in enumerate(path):
                wx, wy = node.cell_to_world(cell[0], cell[1])
                node.get_logger().info(f'  {i+1}. Cell{cell} -> ({wx:.2f}, {wy:.2f})')
            return py_trees.common.Status.SUCCESS
        else:
            node.get_logger().error('NO PATH FOUND!')
            return py_trees.common.Status.FAILURE
