"""Mission-related behaviors for object collection and delivery."""

import py_trees
from py_trees import common
from collections import deque
import math
from geometry_msgs.msg import Twist


class AlignWithAprilTag(py_trees.behaviour.Behaviour):
    """Use AprilTag detection to perfectly align robot before gripper operation."""
    
    def __init__(self, name: str, target_tag_id: int = 4):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="node", access=common.Access.READ)
        self.start_time = None
        self.max_wait_time = 60.0  # Wait max 60 seconds for AprilTag
        self.search_phase = True  # First rotate to find tag
        self.alignment_done = False
        self.rotation_start_yaw = None
        self.target_tag_id = target_tag_id  # Which tag to look for
        
    def initialise(self):
        """Start AprilTag alignment."""
        node = self.blackboard.get("node")
        self.start_time = node.get_clock().now()
        self.alignment_done = False
        self.search_phase = True
        self.rotation_start_yaw = node.robot_yaw
        node.get_logger().info(f'🎯 Searching for AprilTag #{self.target_tag_id} (360° scan)...')
        node.stop_robot()
        
    def update(self):
        """Align using AprilTag correction."""
        node = self.blackboard.get("node")
        
        elapsed = (node.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        # Check if we have recent AprilTag data for the TARGET tag
        has_recent_tag = False
        tag_age = None
        detected_tag_id = node.last_apriltag_id
        
        if node.last_apriltag_pose is not None and node.last_apriltag_time is not None:
            tag_age = (node.get_clock().now() - node.last_apriltag_time).nanoseconds / 1e9
            # Only accept the tag if it's the one we're looking for
            has_recent_tag = (tag_age < 1.0) and (detected_tag_id == self.target_tag_id)
        
        # PHASE 1: Search for AprilTag by rotating
        if self.search_phase:
            if has_recent_tag:
                # Found the TARGET tag! Reset odometry drift using AprilTag position
                node.stop_robot()
                
                # Get precise position from AprilTag
                tag_x, tag_y, tag_yaw = node.last_apriltag_pose
                
                # HARD RESET: Override odometry with AprilTag position (zero drift)
                old_x, old_y, old_yaw = node.robot_x, node.robot_y, node.robot_yaw
                node.robot_x = tag_x
                node.robot_y = tag_y
                node.robot_yaw = tag_yaw
                
                drift_x = abs(old_x - tag_x)
                drift_y = abs(old_y - tag_y)
                drift_angle = abs(math.degrees(node.normalize_angle(old_yaw - tag_yaw)))
                
                node.get_logger().info(f'✅ AprilTag #{self.target_tag_id} DETECTED! Robot is in position.')
                node.get_logger().info(f'🔄 Odometry RESET - Drift corrected: '
                                     f'Δx={drift_x:.3f}m, Δy={drift_y:.3f}m, Δθ={drift_angle:.1f}°')
                return py_trees.common.Status.SUCCESS
            
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
                    elif detected_tag_id != self.target_tag_id:
                        node.get_logger().info(f'🔍 Scanning... {math.degrees(rotated):.0f}°/360° - Detected tag #{detected_tag_id}, looking for #{self.target_tag_id}')
                    else:
                        node.get_logger().info(f'🔍 Scanning... {math.degrees(rotated):.0f}°/360° - Last tag #{self.target_tag_id}: {tag_age:.1f}s ago')
                
                return py_trees.common.Status.RUNNING
            else:
                # Completed full rotation, no target tag found
                node.stop_robot()
                if node.last_apriltag_time is None:
                    node.get_logger().warn('⚠️  AprilTag localizer may not be working (no data received)')
                elif detected_tag_id != self.target_tag_id:
                    node.get_logger().warn(f'⚠️  AprilTag #{self.target_tag_id} not found (detected #{detected_tag_id} instead)')
                else:
                    node.get_logger().warn(f'⚠️  AprilTag #{self.target_tag_id} not visible (last seen {tag_age:.1f}s ago)')
                node.get_logger().warn('    Proceeding with odometry position')
                return py_trees.common.Status.SUCCESS
        
        # No alignment phase - just wait for timeout if tag is lost
        if elapsed > self.max_wait_time:
            node.stop_robot()
            node.get_logger().warn('⚠️  AprilTag search timeout, proceeding with odometry position')
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
