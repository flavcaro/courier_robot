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
        self.blackboard.register_key(key="returning_home", access=common.Access.READ)
        
    def initialise(self):
        """Called when behavior starts."""
        node = self.blackboard.get("node")
        self.blackboard.set("rotation_start_time", node.get_clock().now())
    
    def get_lidar_angle_correction(self, node, target_yaw):
        """
        Use LIDAR to fine-tune rotation angle based on wall alignment.
        Returns angle correction in radians (0 if not applicable).
        """
        if not hasattr(node, 'lidar_ranges') or node.lidar_ranges is None:
            return 0.0
        
        ranges = node.lidar_ranges
        num_rays = len(ranges)
        if num_rays == 0:
            return 0.0
        
        # Only apply when targeting cardinal directions (0°, 90°, 180°, 270°)
        target_deg = (math.degrees(target_yaw) + 360) % 360
        
        # Check if target is cardinal (within 10°)
        is_cardinal = any(abs(target_deg - card) < 10 for card in [0, 90, 180, 270])
        if not is_cardinal:
            return 0.0
        
        # Get left (90°) and right (270°) LIDAR readings
        left_idx = num_rays // 4
        right_idx = 3 * num_rays // 4
        
        left_dist = ranges[left_idx] if left_idx < len(ranges) else float('inf')
        right_dist = ranges[right_idx] if right_idx < len(ranges) else float('inf')
        
        # Only use if both walls visible within reasonable range
        max_wall_dist = 1.5
        if left_dist > max_wall_dist or right_dist > max_wall_dist:
            return 0.0
        
        # If walls are not symmetric, robot is not perpendicular to corridor
        # Positive correction = rotate CCW (left wall closer)
        # Negative correction = rotate CW (right wall closer)
        wall_diff = left_dist - right_dist
        
        # Convert wall distance difference to angle correction
        # Assume ~1m corridor width, small angle approximation
        angle_correction = math.atan2(wall_diff, 1.0) * 0.3  # Damped correction
        
        return angle_correction
        
    def update(self):
        """Execute rotation logic."""
        node = self.blackboard.get("node")
        target_yaw = self.blackboard.get("target_yaw")
        rotation_start = self.blackboard.get("rotation_start_time")
        returning_home = self.blackboard.get("returning_home")
        
        # Guard: ensure we have a valid target
        if target_yaw is None:
            return py_trees.common.Status.FAILURE

        # Apply LIDAR-based angle correction ONLY during return journey
        lidar_correction = 0.0
        if returning_home:
            lidar_correction = self.get_lidar_angle_correction(node, target_yaw)
        
        corrected_target_yaw = node.normalize_angle(target_yaw + lidar_correction)

        # Compute shortest angle error (using corrected target)
        angle_error = node.normalize_angle(corrected_target_yaw - node.robot_yaw)

        if returning_home and lidar_correction != 0.0:
            node.get_logger().debug(
                f'ROTATING (RETURN): target={math.degrees(target_yaw):.1f}° '
                f'lidar_corr={math.degrees(lidar_correction):.1f}° '
                f'current={math.degrees(node.robot_yaw):.1f}° '
                f'error={math.degrees(angle_error):.1f}°'
            )
        else:
            node.get_logger().debug(
                f'ROTATING: target={math.degrees(target_yaw):.1f}° '
                f'current={math.degrees(node.robot_yaw):.1f}° '
                f'error={math.degrees(angle_error):.1f}°'
            )

        # If within tolerance, stop and succeed
        if abs(angle_error) < node.angle_tolerance:
            node.stop_robot()
            node.get_logger().info(f'ROTATION DONE! Yaw={math.degrees(node.robot_yaw):.1f}°')

            # Check LIDAR before moving
            if node.front_distance < node.obstacle_threshold:
                node.get_logger().warn(f'BLOCKED AHEAD! Distance={node.front_distance:.2f}m')
                return py_trees.common.Status.FAILURE

            return py_trees.common.Status.SUCCESS

        # Proportional angular controller (smooth, respects `rotation_speed` limit)
        k_p = 1.6
        ang_cmd = max(-node.rotation_speed, min(node.rotation_speed, k_p * angle_error))

        # Gentle slow-down when close to target angle
        if abs(angle_error) < 0.25:
            ang_cmd *= 0.5

        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = ang_cmd

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
        # Ensure target_yaw is valid before checking drift
        if target_yaw is None:
            node.get_logger().debug('No target_yaw while moving - failing to trigger rotation')
            node.stop_robot()
            return py_trees.common.Status.FAILURE

        angle_error = node.normalize_angle(target_yaw - node.robot_yaw)
        if abs(angle_error) > 0.12:  # ~6.9 degrees, tighter drift check
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
        
        # Normalize yaw to avoid wrap-around inconsistencies
        target_yaw = node.normalize_angle(target_yaw)
        self.blackboard.set("target_yaw", target_yaw)
        
        node.get_logger().debug(f'TARGET: Cell{current_target} = ({target_x:.2f}, {target_y:.2f})')
        node.get_logger().info(f'ROTATE TO: {math.degrees(target_yaw):.0f} deg')
        
        return py_trees.common.Status.SUCCESS


class CenterOnCell(py_trees.behaviour.Behaviour):
    """Fine centering behavior: uses ODOMETRY + LIDAR for better accuracy."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="node", access=common.Access.READ)
        self.blackboard.register_key(key="current_target", access=common.Access.READ)
        self.blackboard.register_key(key="target_world_x", access=common.Access.READ)
        self.blackboard.register_key(key="target_world_y", access=common.Access.READ)
        # For chaining: peek next waypoint and set target yaw for rotation stage
        self.blackboard.register_key(key="path_queue", access=common.Access.READ)
        self.blackboard.register_key(key="target_yaw", access=common.Access.WRITE)
        self.start_time = None

    def initialise(self):
        node = self.blackboard.get("node")
        self.start_time = node.get_clock().now()

    def get_lidar_correction(self, node):
        """
        Use LIDAR to estimate lateral offset from cell center.
        Returns (dx_correction, dy_correction) in world frame, or (0, 0) if not applicable.
        
        Strategy: 
        - Check left/right LIDAR readings (perpendicular to robot forward)
        - If both sides see walls at reasonable distance, use difference to center
        - Only apply when robot is aligned with grid (yaw ~0, 90, 180, 270 deg)
        """
        # DISABLED: lateral correction was causing issues
        return 0.0, 0.0
        
        if not hasattr(node, 'lidar_ranges') or node.lidar_ranges is None:
            return 0.0, 0.0
        
        # Get LIDAR data (assuming 360 rays, 0=front, 90=left, 180=back, 270=right)
        ranges = node.lidar_ranges
        num_rays = len(ranges)
        if num_rays == 0:
            return 0.0, 0.0
        
        # Calculate indices for left and right (±90° from front)
        left_idx = num_rays // 4  # 90°
        right_idx = 3 * num_rays // 4  # 270°
        
        left_dist = ranges[left_idx] if left_idx < len(ranges) else float('inf')
        right_dist = ranges[right_idx] if right_idx < len(ranges) else float('inf')
        
        # Only use LIDAR correction if both sides see walls within cell distance (< 1.5m)
        max_wall_dist = 1.5
        if left_dist > max_wall_dist or right_dist > max_wall_dist:
            return 0.0, 0.0
        
        # Lateral offset: positive means robot is too far right
        lateral_offset = (right_dist - left_dist) / 2.0
        
        # Convert to world frame based on robot orientation
        # Determine if robot is axis-aligned
        yaw_deg = math.degrees(node.robot_yaw) % 360
        
        # Tolerance for "aligned with grid"
        align_tol = 15  # degrees
        
        dx_corr = 0.0
        dy_corr = 0.0
        
        if abs(yaw_deg) < align_tol or abs(yaw_deg - 360) < align_tol:
            # Facing +X (East): left is +Y, right is -Y
            dy_corr = -lateral_offset
        elif abs(yaw_deg - 90) < align_tol:
            # Facing +Y (North): left is -X, right is +X
            dx_corr = lateral_offset
        elif abs(yaw_deg - 180) < align_tol:
            # Facing -X (West): left is -Y, right is +Y
            dy_corr = lateral_offset
        elif abs(yaw_deg - 270) < align_tol:
            # Facing -Y (South): left is +X, right is -X
            dx_corr = -lateral_offset
        
        return dx_corr, dy_corr

    def update(self):
        node = self.blackboard.get("node")

        target_x = self.blackboard.get("target_world_x")
        target_y = self.blackboard.get("target_world_y")
        current_target = self.blackboard.get("current_target")

        if target_x is None or target_y is None or current_target is None:
            return py_trees.common.Status.FAILURE

        # Compute error in world frame from ODOMETRY
        ex_odom = target_x - node.robot_x
        ey_odom = target_y - node.robot_y

        # Get LIDAR-based correction
        dx_lidar, dy_lidar = self.get_lidar_correction(node)
        
        # Combine: use LIDAR to correct lateral offset
        ex = ex_odom + dx_lidar
        ey = ey_odom + dy_lidar
        
        # Transform to robot frame
        cy = math.cos(node.robot_yaw)
        sy = math.sin(node.robot_yaw)
        x_r =  cy * ex + sy * ey
        y_r = -sy * ex + cy * ey

        distance = math.hypot(ex, ey)
        center_tolerance = 0.04  # 4 cm centering tolerance

        # Timeout to avoid blocking forever
        elapsed = (node.get_clock().now() - self.start_time).nanoseconds / 1e9
        max_time = 4.0

        if abs(x_r) < center_tolerance and abs(y_r) < center_tolerance:
            # If centered, set the next target yaw (if a next waypoint exists)
            def _set_next_target_yaw():
                path_queue = self.blackboard.get("path_queue")
                if not path_queue:
                    return
                try:
                    next_target = path_queue[0]
                except Exception:
                    return

                nr, nc = next_target
                nx, ny = node.cell_to_world(nr, nc)

                try:
                    current_cell = node.world_to_cell(node.robot_x, node.robot_y)
                except Exception:
                    current_cell = None

                dx = nx - node.robot_x
                dy = ny - node.robot_y

                if current_cell is not None:
                    crow, ccol = current_cell
                    drow = nr - crow
                    dcol = nc - ccol

                    if dcol != 0:
                        target_yaw = 0.0 if dcol > 0 else math.pi
                    elif drow != 0:
                        target_yaw = math.pi / 2 if drow > 0 else -math.pi / 2
                    else:
                        if abs(dx) > abs(dy):
                            target_yaw = 0.0 if dx > 0 else math.pi
                        else:
                            target_yaw = math.pi / 2 if dy > 0 else -math.pi / 2
                else:
                    if abs(dx) > abs(dy):
                        target_yaw = 0.0 if dx > 0 else math.pi
                    else:
                        target_yaw = math.pi / 2 if dy > 0 else -math.pi / 2

                # Normalize before writing to blackboard
                target_yaw = node.normalize_angle(target_yaw)
                self.blackboard.set("target_yaw", target_yaw)
                node.get_logger().debug(f'CENTER: setting next target yaw {math.degrees(target_yaw):.0f} deg for Cell{next_target}')

            _set_next_target_yaw()
            node.stop_robot()
            node.get_logger().info(f'CENTERED Cell{current_target} (err={distance:.3f}m)')
            return py_trees.common.Status.SUCCESS

        if elapsed > max_time:
            # On timeout also attempt to set next yaw so rotation stage can proceed
            try:
                path_queue = self.blackboard.get("path_queue")
                if path_queue:
                    next_target = path_queue[0]
                    nr, nc = next_target
                    nx, ny = node.cell_to_world(nr, nc)
                    dx = nx - node.robot_x
                    dy = ny - node.robot_y
                    if abs(dx) > abs(dy):
                        target_yaw = 0.0 if dx > 0 else math.pi
                    else:
                        target_yaw = math.pi / 2 if dy > 0 else -math.pi / 2
                    target_yaw = node.normalize_angle(target_yaw)
                    self.blackboard.set("target_yaw", target_yaw)
                    node.get_logger().debug(f'CENTER timeout: setting next target yaw {math.degrees(target_yaw):.0f} deg for Cell{next_target}')
            except Exception:
                pass

            node.stop_robot()
            node.get_logger().warn(f'Centering timeout for Cell{current_target} (err={distance:.3f}m)')
            return py_trees.common.Status.SUCCESS

        # Simple proportional controller: small angular correction and forward speed
        k_linear = 0.6
        k_angular = 1.2

        # Desired heading in robot frame
        desired_heading = math.atan2(y_r, x_r)
        linear_speed = max(-0.08, min(0.08, k_linear * x_r))
        angular_speed = max(-0.6, min(0.6, k_angular * desired_heading))

        # If error is mostly lateral, prioritize rotation then small forward steps
        if abs(desired_heading) > 0.35:
            linear_speed = 0.02

        cmd = Twist()
        cmd.linear.x = linear_speed
        cmd.angular.z = angular_speed
        node.cmd_vel_pub.publish(cmd)

        return py_trees.common.Status.RUNNING
