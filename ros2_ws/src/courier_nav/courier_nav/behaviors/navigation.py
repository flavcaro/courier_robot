"""Navigation behaviors for the behavior tree."""

import math
import py_trees
from py_trees import common
from geometry_msgs.msg import Twist


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
        
        # Check obstacle with adaptive threshold based on target proximity to walls
        # When targeting cells near boundaries, use smaller obstacle threshold
        target_near_edge = (target_x < 0.3 or target_x > 3.7 or target_y < 0.3 or target_y > 3.7)
        obstacle_threshold = 0.35 if target_near_edge else node.obstacle_threshold
        
        if node.front_distance < obstacle_threshold:
            node.stop_robot()
            node.get_logger().warn(f'OBSTACLE at {node.front_distance:.2f}m (threshold={obstacle_threshold:.2f}m)!')
            return py_trees.common.Status.FAILURE
        
        # Check boundaries with relaxed constraints for edge cells
        # Allow robot to be closer to boundary when targeting edge cells
        boundary_margin = 0.4
        
        # If target is near edge (x or y close to 0 or 4), relax boundary check
        target_near_west = target_x < 0.3
        target_near_east = target_x > 3.7
        target_near_south = target_y < 0.3
        target_near_north = target_y > 3.7
        
        # Use tighter boundary for edge targets
        if target_near_west or target_near_east or target_near_south or target_near_north:
            boundary_margin = 0.2  # Allow closer to wall for edge cells
        
        if node.robot_x < -boundary_margin or node.robot_x > (4.0 + boundary_margin) or \
           node.robot_y < -boundary_margin or node.robot_y > (4.0 + boundary_margin):
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
            # Decrement battery by 10% for each cell traversed
            node.battery_level -= 10.0
            if node.battery_level < 0:
                node.battery_level = 0.0
            node.get_logger().info(f'REACHED Cell{current_target}! 🔋 Battery: {node.battery_level:.0f}%')
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
