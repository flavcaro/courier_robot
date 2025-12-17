#!/usr/bin/env python3
"""
Courier Behavior Tree

This module implements a behavior tree for the courier robot that manages:
- Battery levels
- Path following
- Target reaching
- Charging decisions
"""

import py_trees
from py_trees.composites import Sequence, Selector
from py_trees.common import Status


class BatteryOK(py_trees.behaviour.Behaviour):
    """
    Condition node that checks if the battery level is sufficient.
    """
    
    def __init__(self, name="BatteryOK", battery_threshold=20.0):
        """
        Initialize the battery condition checker.
        
        Args:
            name: Name of the behavior
            battery_threshold: Minimum battery level required (percentage)
        """
        super(BatteryOK, self).__init__(name)
        self.battery_threshold = battery_threshold
        self.feedback_message = ""
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="battery_level",
            access=py_trees.common.Access.READ
        )
    
    def setup(self, **kwargs):
        """Setup the behavior (called once before first tick)."""
        pass
    
    def update(self):
        """
        Check the battery level.
        
        Returns:
            Status.SUCCESS if battery is OK, Status.FAILURE otherwise
        """
        battery_level = self.blackboard.get("battery_level")
        
        if battery_level >= self.battery_threshold:
            self.feedback_message = f"Battery OK: {battery_level:.1f}%"
            return Status.SUCCESS
        else:
            self.feedback_message = f"Battery low: {battery_level:.1f}%"
            return Status.FAILURE


class HasTarget(py_trees.behaviour.Behaviour):
    """
    Condition node that checks if there is a current target.
    """
    
    def __init__(self, name="HasTarget"):
        """
        Initialize the target checker.
        
        Args:
            name: Name of the behavior
        """
        super(HasTarget, self).__init__(name)
        self.feedback_message = ""
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="current_target",
            access=py_trees.common.Access.READ
        )
    
    def setup(self, **kwargs):
        """Setup the behavior (called once before first tick)."""
        pass
    
    def update(self):
        """
        Check if there is a current target.
        
        Returns:
            Status.SUCCESS if there is a target, Status.FAILURE otherwise
        """
        current_target = self.blackboard.get("current_target")
        
        if current_target is not None:
            self.feedback_message = f"Target exists: {current_target}"
            return Status.SUCCESS
        else:
            self.feedback_message = "No target"
            return Status.FAILURE


class HasPathQueue(py_trees.behaviour.Behaviour):
    """
    Condition node that checks if there are waypoints in the path queue.
    """
    
    def __init__(self, name="HasPathQueue"):
        """
        Initialize the path queue checker.
        
        Args:
            name: Name of the behavior
        """
        super(HasPathQueue, self).__init__(name)
        self.feedback_message = ""
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="path_queue",
            access=py_trees.common.Access.READ
        )
    
    def setup(self, **kwargs):
        """Setup the behavior (called once before first tick)."""
        pass
    
    def update(self):
        """
        Check if path queue has waypoints.
        
        Returns:
            Status.SUCCESS if path queue not empty, Status.FAILURE otherwise
        """
        path_queue = self.blackboard.get("path_queue")
        
        if path_queue and len(path_queue) > 0:
            self.feedback_message = f"Path queue has {len(path_queue)} waypoints"
            return Status.SUCCESS
        else:
            self.feedback_message = "Path queue empty"
            return Status.FAILURE


class IsAligned(py_trees.behaviour.Behaviour):
    """
    Condition node that checks if the robot is aligned with the target.
    """
    
    def __init__(self, name="IsAligned", angle_tolerance=0.15):
        """
        Initialize the alignment checker.
        
        Args:
            name: Name of the behavior
            angle_tolerance: Maximum angle difference (radians)
        """
        super(IsAligned, self).__init__(name)
        self.angle_tolerance = angle_tolerance
        self.feedback_message = ""
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="angle_diff",
            access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="return_planned",
            access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="logger",
            access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="initial_alignment_done",
            access=py_trees.common.Access.READ
        )
    
    def setup(self, **kwargs):
        """Setup the behavior (called once before first tick)."""
        pass
    
    def update(self):
        """
        Check if robot is aligned with target.
        
        Returns:
            Status.SUCCESS if aligned, Status.FAILURE otherwise
        """
        angle_diff = self.blackboard.get("angle_diff")
        return_planned = self.blackboard.get("return_planned")
        initial_alignment_done = self.blackboard.get("initial_alignment_done")
        logger = self.blackboard.get("logger")
        
        # Use appropriate tolerance based on phase
        if not initial_alignment_done:
            tolerance = 0.02  # ~1.15° for first alignment
        elif return_planned:
            tolerance = 0.03  # ~1.7° for return path
        else:
            tolerance = 0.05  # ~2.9° for normal navigation
        
        if abs(angle_diff) <= tolerance:
            if logger and return_planned:
                import math
                logger.info(f'✓ Allineato per ritorno: {abs(angle_diff)*180/math.pi:.3f}° (limite: {tolerance*180/math.pi:.2f}°)')
            self.feedback_message = f"Aligned: {angle_diff:.3f} rad"
            return Status.SUCCESS
        else:
            if logger and return_planned:
                import math
                logger.info(f'❌ NON allineato per ritorno: {abs(angle_diff)*180/math.pi:.3f}° > {tolerance*180/math.pi:.2f}°')
            self.feedback_message = f"Not aligned: {angle_diff:.3f} rad"
            return Status.FAILURE


class IsAtTarget(py_trees.behaviour.Behaviour):
    """
    Condition node that checks if the robot reached the target.
    """
    
    def __init__(self, name="IsAtTarget", dist_tolerance=0.10):
        """
        Initialize the target distance checker.
        
        Args:
            name: Name of the behavior
            dist_tolerance: Maximum distance to target (meters)
        """
        super(IsAtTarget, self).__init__(name)
        self.dist_tolerance = dist_tolerance
        self.feedback_message = ""
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="distance_to_target",
            access=py_trees.common.Access.READ
        )
    
    def setup(self, **kwargs):
        """Setup the behavior (called once before first tick)."""
        pass
    
    def update(self):
        """
        Check if robot is at target.
        
        Returns:
            Status.SUCCESS if at target, Status.FAILURE otherwise
        """
        distance = self.blackboard.get("distance_to_target")
        
        # Richiedi che il robot vada leggermente OLTRE il centro (tolleranza negativa simulata)
        # Usa una tolleranza più stretta per forzare ad arrivare più vicino
        if distance <= self.dist_tolerance:
            self.feedback_message = f"At target: {distance:.3f} m"
            return Status.SUCCESS
        else:
            self.feedback_message = f"Not at target: {distance:.3f} m"
            return Status.FAILURE


class IsAtGoal(py_trees.behaviour.Behaviour):
    """
    Condition node that checks if the robot is at the final goal (but hasn't collected yet).
    """
    
    def __init__(self, name="IsAtGoal", goal_tolerance=0.20, cell_size=1.0):
        """
        Initialize the goal checker.
        
        Args:
            name: Name of the behavior
        """
        super(IsAtGoal, self).__init__(name)
        self.goal_tolerance = goal_tolerance
        self.cell_size = cell_size
        self.feedback_message = ""
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="path_queue",
            access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="current_target",
            access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="object_collected",
            access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="robot_position",
            access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="goal_cell",
            access=py_trees.common.Access.READ
        )
    
    def setup(self, **kwargs):
        """Setup the behavior (called once before first tick)."""
        pass
    
    def update(self):
        """
        Check if robot is at final goal (no more waypoints and no target).
        
        Returns:
            Status.SUCCESS if at goal, Status.FAILURE otherwise
        """
        path_queue = self.blackboard.get("path_queue")
        current_target = self.blackboard.get("current_target")
        object_collected = self.blackboard.get("object_collected")
        robot_pos = self.blackboard.get("robot_position")
        goal_cell = self.blackboard.get("goal_cell")
        goal_xy = ((goal_cell[0] + 0.5) * self.cell_size, (goal_cell[1] + 0.5) * self.cell_size)
        dist_goal = ((robot_pos[0]-goal_xy[0])**2 + (robot_pos[1]-goal_xy[1])**2) ** 0.5

        if (not path_queue or len(path_queue) == 0) and current_target is None and not object_collected and dist_goal <= self.goal_tolerance:
            self.feedback_message = f"At final goal (dist {dist_goal:.2f} m)"
            return Status.SUCCESS
        else:
            self.feedback_message = f"Not at goal yet (dist {dist_goal:.2f} m)"
            return Status.FAILURE


class ObjectCollected(py_trees.behaviour.Behaviour):
    """
    Condition node that checks if object has been collected.
    """
    
    def __init__(self, name="ObjectCollected"):
        """
        Initialize the object collected checker.
        
        Args:
            name: Name of the behavior
        """
        super(ObjectCollected, self).__init__(name)
        self.feedback_message = ""
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="object_collected",
            access=py_trees.common.Access.READ
        )
    
    def setup(self, **kwargs):
        """Setup the behavior (called once before first tick)."""
        pass
    
    def update(self):
        """
        Check if object has been collected.
        
        Returns:
            Status.SUCCESS if collected, Status.FAILURE otherwise
        """
        object_collected = self.blackboard.get("object_collected")
        
        if object_collected:
            self.feedback_message = "Object already collected"
            return Status.SUCCESS
        else:
            self.feedback_message = "Object not collected yet"
            return Status.FAILURE


# ============================================================================
# ACTION NODES
# ============================================================================

class GoCharge(py_trees.behaviour.Behaviour):
    """
    Action node that makes the robot go to charging station.
    """
    
    def __init__(self, name="GoCharge", charge_rate=10.0):
        """
        Initialize the charging action.
        
        Args:
            name: Name of the behavior
            charge_rate: Battery charge rate per tick (percentage)
        """
        super(GoCharge, self).__init__(name)
        self.charge_rate = charge_rate
        self.feedback_message = ""
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="battery_level",
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="logger",
            access=py_trees.common.Access.READ
        )
    
    def setup(self, **kwargs):
        """Setup the behavior (called once before first tick)."""
        pass
    
    def update(self):
        """
        Simulate charging the battery.
        
        Returns:
            Status.SUCCESS after charging
        """
        battery_level = self.blackboard.get("battery_level")
        logger = self.blackboard.get("logger")
        
        if logger:
            logger.info(f"  {self.name} [Going to charging station...]")
        
        # Charge the battery
        new_level = min(100.0, battery_level + self.charge_rate)
        self.blackboard.set("battery_level", new_level)
        
        self.feedback_message = f"Charged battery to {new_level:.1f}%"
        
        if logger:
            logger.info(f"  {self.name} [Battery charged to {new_level:.1f}%]")
        
        return Status.SUCCESS


class AlignForReturn(py_trees.behaviour.Behaviour):
    """
    Action node that aligns the robot toward AprilTag marker (which points to first cell of return path).
    """
    
    def __init__(self, name="AlignForReturn", cell_size=1.0):
        super().__init__(name)
        self.cell_size = cell_size
        self.aligned = False
        self.stabilizing = False
        self.stabilize_start = None
        self.align_start_time = None
        self.max_align_time = 10.0  # Max 10 secondi per allineamento preciso
        self.stabilize_time = 0.5  # Pausa di 0.5s dopo allineamento per stabilizzare
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="path_queue", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="robot_position", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="robot_yaw", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="cmd_vel_publisher", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="logger", access=py_trees.common.Access.READ)
    
    def initialise(self):
        """Called when behavior switches to RUNNING."""
        import time
        from geometry_msgs.msg import Twist
        
        self.aligned = False
        self.stabilizing = False
        self.stabilize_start = None
        self.align_start_time = time.time()
        
        # STOP completamente prima di iniziare allineamento
        cmd_vel_pub = self.blackboard.get("cmd_vel_publisher")
        if cmd_vel_pub:
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            cmd_vel_pub(msg)
        
        logger = self.blackboard.get("logger")
        robot_yaw = self.blackboard.get("robot_yaw")
        robot_pos = self.blackboard.get("robot_position")
        
        if logger:
            import math
            logger.info(f'🎯 INIZIO allineamento verso AprilTag in posizione (4.5, 0.25)')
            logger.info(f'  Posizione attuale: ({robot_pos[0]:.2f}, {robot_pos[1]:.2f})')
            logger.info(f'  Orientamento attuale: {robot_yaw*180/math.pi:.1f}°')
    
    def update(self):
        import math
        import time
        from geometry_msgs.msg import Twist
        
        path_queue = self.blackboard.get("path_queue")
        robot_pos = self.blackboard.get("robot_position")
        robot_yaw = self.blackboard.get("robot_yaw")
        cmd_vel_pub = self.blackboard.get("cmd_vel_publisher")
        logger = self.blackboard.get("logger")
        
        if not path_queue or len(path_queue) == 0:
            return Status.FAILURE
        
        # Check timeout
        if time.time() - self.align_start_time > self.max_align_time:
            msg = Twist()
            if cmd_vel_pub:
                cmd_vel_pub(msg)
            if logger:
                logger.warn('⚠️ Timeout allineamento, procedo comunque')
            return Status.SUCCESS
        
        # Allinea verso il marker AprilTag di ritorno al centro della cella (4,0)
        target_x = 4.5
        target_y = 0.25
        
        if logger and not self.aligned:
            logger.info(f'📍 Target allineamento verso AprilTag in posizione fissa: ({target_x:.2f}, {target_y:.2f})')
        
        # Calculate angle toward target position
        dx = target_x - robot_pos[0]
        dy = target_y - robot_pos[1]
        target_angle = math.atan2(dy, dx)
        
        # Calculate angle difference
        angle_diff = target_angle - robot_yaw
        # Normalize to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Tolleranza MOLTO stretta: 0.015 rad = ~0.86 gradi per partenza perfettamente dritta
        if abs(angle_diff) < 0.015:
            # Allineato! Inizia fase di stabilizzazione
            if not self.aligned:
                self.aligned = True
                self.stabilizing = True
                self.stabilize_start = time.time()
                if logger:
                    logger.info(f'✅ PERFETTAMENTE allineato verso AprilTag in (4.5, 0.5)')
                    logger.info(f'   Errore angolare: {abs(angle_diff)*180/math.pi:.3f}°')
                    logger.info(f'   Angolo robot: {robot_yaw*180/math.pi:.1f}° → Target: {target_angle*180/math.pi:.1f}°')
                    logger.info(f'⏸️ Stabilizzazione per {self.stabilize_time}s...')
            
            # Mantieni fermo durante stabilizzazione
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            if cmd_vel_pub:
                cmd_vel_pub(msg)
            
            # Attendi tempo di stabilizzazione
            if self.stabilizing and (time.time() - self.stabilize_start >= self.stabilize_time):
                if logger:
                    logger.info('✓ Stabilizzazione completata, PRONTO per ritorno!')
                return Status.SUCCESS
            
            return Status.RUNNING
        
        # Controllo proporzionale per rotazione smooth e precisa
        Kp = 3.5  # Più aggressivo per convergenza rapida
        angular_vel = Kp * angle_diff
        
        # Limita velocità angolare
        max_angular_vel = 0.6  # rad/s - leggermente aumentato
        angular_vel = max(-max_angular_vel, min(max_angular_vel, angular_vel))
        
        # Log progress ogni secondo
        elapsed = time.time() - self.align_start_time
        if int(elapsed) != int(elapsed - 0.1) and logger:
            logger.info(f'  ↻ Allineamento in corso: errore={abs(angle_diff)*180/math.pi:.2f}°, vel={angular_vel:.2f} rad/s')
        
        # Limita velocità angolare per evitare overshooting
        max_angular_vel = 0.5  # rad/s
        angular_vel = max(-max_angular_vel, min(max_angular_vel, angular_vel))
        
        # Rotate toward AprilTag marker
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = angular_vel
        
        if cmd_vel_pub:
            cmd_vel_pub(msg)
        
        return Status.RUNNING


class GetNextTarget(py_trees.behaviour.Behaviour):
    """
    Action node that gets the next target from path queue.
    """
    
    def __init__(self, name="GetNextTarget", cell_size=1.0):
        """
        Initialize the get next target action.
        
        Args:
            name: Name of the behavior
            cell_size: Size of grid cells in meters
        """
        super(GetNextTarget, self).__init__(name)
        self.cell_size = cell_size
        self.feedback_message = ""
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="path_queue",
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="current_target",
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="logger",
            access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="robot_position",
            access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="return_planned",
            access=py_trees.common.Access.READ
        )
    
    def setup(self, **kwargs):
        """Setup the behavior (called once before first tick)."""
        pass
    
    def update(self):
        """
        Get next target from path queue.
        
        Returns:
            Status.SUCCESS if target was set, Status.FAILURE if queue empty
        """
        path_queue = self.blackboard.get("path_queue")
        robot_pos = self.blackboard.get("robot_position")
        return_planned = self.blackboard.get("return_planned")
        logger = self.blackboard.get("logger")
        
        if not path_queue or len(path_queue) == 0:
            self.feedback_message = "No more waypoints in queue"
            return Status.FAILURE
        
        # Get next cell
        next_cell = path_queue.pop(0)
        target_x = (next_cell[0] + 0.5) * self.cell_size
        target_y = (next_cell[1] + 0.5) * self.cell_size
        
        # Special case: during return, position (4,0) target al centro della cella per allinearsi al tag
        if return_planned and next_cell == (4, 0):
            target_y = 0.25
            if logger:
                logger.info(f'🎯 Target (4,0) centrato a y=0.25 per allinearsi al tag')
        
        # Special case: final target (0,0) should be at left edge, centered vertically
        if return_planned and next_cell == (0, 0):
            target_x = 0.0  # Left edge
            target_y = 0.5  # Centered vertically
            if logger:
                logger.info(f'🎯 Target finale (0,0) spostato a ({target_x:.2f}, {target_y:.2f}) per arrivo preciso')
        
        current_target = (target_x, target_y)
        
        # Update blackboard
        self.blackboard.set("path_queue", path_queue)
        self.blackboard.set("current_target", current_target)
        
        self.feedback_message = f"New target: {current_target}"
        
        if logger:
            logger.info(f'📍 Nuovo target: cella {next_cell} -> coord ({target_x:.2f}, {target_y:.2f}) | Celle rimanenti: {len(path_queue)}')
        
        return Status.SUCCESS


class RotateToTarget(py_trees.behaviour.Behaviour):
    """
    Action node that rotates the robot toward the target.
    """
    
    def __init__(self, name="RotateToTarget", battery_cost=1.0):
        """
        Initialize the rotate action.
        
        Args:
            name: Name of the behavior
            battery_cost: Battery consumption per tick (percentage)
        """
        super(RotateToTarget, self).__init__(name)
        self.battery_cost = battery_cost
        self.feedback_message = ""
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="angle_diff",
            access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="battery_level",
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="cmd_vel_publisher",
            access=py_trees.common.Access.READ
        )
    
    def setup(self, **kwargs):
        """Setup the behavior (called once before first tick)."""
        pass
    
    def update(self):
        """
        Rotate toward target.
        
        Returns:
            Status.RUNNING while rotating
        """
        angle_diff = self.blackboard.get("angle_diff")
        battery_level = self.blackboard.get("battery_level")
        cmd_vel_pub = self.blackboard.get("cmd_vel_publisher")
        
        # Consume battery
        new_level = max(0.0, battery_level - self.battery_cost * 0.1)
        self.blackboard.set("battery_level", new_level)
        
        # Publish rotation command with stronger control
        if cmd_vel_pub:
            from geometry_msgs.msg import Twist
            msg = Twist()
            # Controllo proporzionale più forte per allineamento preciso
            Kp = 3.0  # Aumentato da 1.0 a 3.0
            msg.angular.z = max(min(Kp * angle_diff, 0.8), -0.8)
            msg.linear.x = 0.0
            cmd_vel_pub(msg)
        
        self.feedback_message = f"Rotating: {angle_diff:.3f} rad"
        
        return Status.RUNNING


class MoveToTarget(py_trees.behaviour.Behaviour):
    """
    Action node that moves the robot toward the target.
    """
    
    def __init__(self, name="MoveToTarget", battery_cost=2.0):
        """
        Initialize the move action.
        
        Args:
            name: Name of the behavior
            battery_cost: Battery consumption per tick (percentage)
        """
        super(MoveToTarget, self).__init__(name)
        self.battery_cost = battery_cost
        self.feedback_message = ""
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="distance_to_target",
            access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="angle_diff",
            access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="battery_level",
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="cmd_vel_publisher",
            access=py_trees.common.Access.READ
        )
    
    def setup(self, **kwargs):
        """Setup the behavior (called once before first tick)."""
        pass
    
    def update(self):
        """
        Move toward target with axis-aligned correction.
        """
        distance = self.blackboard.get("distance_to_target")
        angle_diff = self.blackboard.get("angle_diff")
        battery_level = self.blackboard.get("battery_level")
        cmd_vel_pub = self.blackboard.get("cmd_vel_publisher")
        
        new_level = max(0.0, battery_level - self.battery_cost * 0.1)
        self.blackboard.set("battery_level", new_level)
        
        if cmd_vel_pub:
            from geometry_msgs.msg import Twist
            msg = Twist()
            
            # If misaligned, stop and let RotateToTarget handle it (stricter threshold)
            if abs(angle_diff) > 0.05:  # ~2.9° - stricter to avoid drifting into obstacles
                msg.linear.x = 0.0
                msg.angular.z = 0.0
            else:
                # Move forward with proportional angular correction to stay on axis
                speed = max(0.08, min(0.20, distance * 0.4))  # Slower speed for precision
                if distance < 0.10:  # Match IsAtTarget tolerance
                    speed = 0.0
                
                msg.linear.x = speed
                # Apply stronger angular correction while moving (proportional control)
                Kp_angular = 2.5  # Increased gain for better course correction
                msg.angular.z = Kp_angular * angle_diff
                # Limit angular correction during movement
                msg.angular.z = max(-0.4, min(0.4, msg.angular.z))
            
            cmd_vel_pub(msg)
        
        self.feedback_message = f"Moving: {distance:.3f} m, angle_diff: {angle_diff:.3f}"
        return Status.RUNNING


class ClearTarget(py_trees.behaviour.Behaviour):
    """
    Action node that clears the current target.
    """
    
    def __init__(self, name="ClearTarget"):
        """
        Initialize the clear target action.
        
        Args:
            name: Name of the behavior
        """
        super(ClearTarget, self).__init__(name)
        self.feedback_message = ""
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="current_target",
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="logger",
            access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="initial_alignment_done",
            access=py_trees.common.Access.WRITE
        )
    
    def setup(self, **kwargs):
        """Setup the behavior (called once before first tick)."""
        pass
    
    def update(self):
        """
        Clear the current target.
        
        Returns:
            Status.SUCCESS
        """
        current_target = self.blackboard.get("current_target")
        logger = self.blackboard.get("logger")
        
        self.blackboard.set("current_target", None)
        # Mark initial alignment as done after the first target is cleared
        self.blackboard.set("initial_alignment_done", True)
        
        self.feedback_message = f"Target cleared: {current_target}"
        
        if logger and current_target:
            logger.info(f'✓ Target raggiunto: ({current_target[0]:.2f}, {current_target[1]:.2f})')
        
        return Status.SUCCESS


class StopRobot(py_trees.behaviour.Behaviour):
    """
    Action node that stops the robot.
    """
    
    def __init__(self, name="StopRobot"):
        """
        Initialize the stop action.
        
        Args:
            name: Name of the behavior
        """
        super(StopRobot, self).__init__(name)
        self.feedback_message = ""
        self.logged_once = False  # Flag to log message only once
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="cmd_vel_publisher",
            access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="logger",
            access=py_trees.common.Access.READ
        )
    
    def setup(self, **kwargs):
        """Setup the behavior (called once before first tick)."""
        self.logged_once = False  # Reset flag on setup
    
    def update(self):
        """
        Stop the robot.
        
        Returns:
            Status.SUCCESS
        """
        cmd_vel_pub = self.blackboard.get("cmd_vel_publisher")
        logger = self.blackboard.get("logger")
        
        if cmd_vel_pub:
            from geometry_msgs.msg import Twist
            msg = Twist()
            cmd_vel_pub(msg)
        
        self.feedback_message = "Robot stopped"
        
        # Log only once
        if logger and not self.logged_once:
            logger.info('🎯 Goal raggiunto! Robot fermo.')
            self.logged_once = True
        
        return Status.SUCCESS


class ShouldPlanReturn(py_trees.behaviour.Behaviour):
    """
    Condition node that checks if robot should plan return path.
    """
    
    def __init__(self, name="ShouldPlanReturn"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="object_collected", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="return_planned", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="path_queue", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="current_target", access=py_trees.common.Access.READ)
    
    def update(self):
        object_collected = self.blackboard.get("object_collected")
        return_planned = self.blackboard.get("return_planned")
        path_queue = self.blackboard.get("path_queue")
        current_target = self.blackboard.get("current_target")
        
        # Should plan return if object collected, not yet planned, and no active navigation
        if object_collected and not return_planned and not path_queue and current_target is None:
            return Status.SUCCESS
        return Status.FAILURE


class AtTurnPoint(py_trees.behaviour.Behaviour):
    """
    Condition to check if robot is at (4,0) and needs to align toward start AprilTag.
    """
    
    def __init__(self, name="AtTurnPoint"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="robot_position", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="return_planned", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="aligned_at_turn", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="logger", access=py_trees.common.Access.READ)
    
    def update(self):
        robot_pos = self.blackboard.get("robot_position")
        return_planned = self.blackboard.get("return_planned")
        aligned_at_turn = self.blackboard.get("aligned_at_turn")
        logger = self.blackboard.get("logger")
        
        # Check if at (4,0) lower position: x≈4.5, y≈0.25
        if return_planned and not aligned_at_turn:
            if abs(robot_pos[0] - 4.5) < 0.15 and abs(robot_pos[1] - 0.25) < 0.15:
                if logger:
                    logger.info(f'🎯 Punto di svolta (4,0) basso raggiunto! Posizione: ({robot_pos[0]:.2f}, {robot_pos[1]:.2f})')
                return Status.SUCCESS
        return Status.FAILURE


class AlignToStartTag(py_trees.behaviour.Behaviour):
    """
    Align robot toward start AprilTag at (0,0).
    """
    
    def __init__(self, name="AlignToStartTag", cell_size=1.0):
        super().__init__(name)
        self.cell_size = cell_size
        self.aligned = False
        self.align_start_time = None
        self.max_align_time = 5.0
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="robot_position", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="robot_yaw", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="cmd_vel_publisher", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="logger", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="aligned_at_turn", access=py_trees.common.Access.WRITE)
    
    def initialise(self):
        import time
        self.aligned = False
        self.align_start_time = time.time()
        logger = self.blackboard.get("logger")
        robot_pos = self.blackboard.get("robot_position")
        robot_yaw = self.blackboard.get("robot_yaw")
        if logger:
            import math
            logger.info(f'🎯 INIZIO allineamento verso AprilTag START in (0,0)')
            logger.info(f'  Posizione corrente: ({robot_pos[0]:.2f}, {robot_pos[1]:.2f})')
            logger.info(f'  Orientamento corrente: {robot_yaw*180/math.pi:.1f}°')
    
    def update(self):
        import math
        import time
        from geometry_msgs.msg import Twist
        
        robot_pos = self.blackboard.get("robot_position")
        robot_yaw = self.blackboard.get("robot_yaw")
        cmd_vel_pub = self.blackboard.get("cmd_vel_publisher")
        logger = self.blackboard.get("logger")
        
        # Timeout check
        if time.time() - self.align_start_time > self.max_align_time:
            msg = Twist()
            if cmd_vel_pub:
                cmd_vel_pub(msg)
            self.blackboard.set("aligned_at_turn", True)
            if logger:
                logger.warn('⚠️ Timeout allineamento start AprilTag')
            return Status.SUCCESS
        
        # Target is start (0,0) center
        target_x = 0.5
        target_y = 0.5
        
        # Calculate angle toward start
        dx = target_x - robot_pos[0]
        dy = target_y - robot_pos[1]
        target_angle = math.atan2(dy, dx)
        
        angle_diff = target_angle - robot_yaw
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Check if aligned (0.005 rad ≈ 0.29°) - stricter tolerance
        if abs(angle_diff) < 0.005:
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            if cmd_vel_pub:
                cmd_vel_pub(msg)
            
            if not self.aligned:
                self.blackboard.set("aligned_at_turn", True)
                if logger:
                    logger.info(f'✅ PERFETTAMENTE allineato verso START AprilTag (0,0)')
                    logger.info(f'  Errore angolare: {abs(angle_diff)*180/math.pi:.3f}°')
                    logger.info(f'  Angolo target: {target_angle*180/math.pi:.1f}°, Yaw robot: {robot_yaw*180/math.pi:.1f}°')
                self.aligned = True
            
            return Status.SUCCESS
        
        # Proportional control with logging
        Kp = 2.5
        angular_vel = Kp * angle_diff
        max_angular_vel = 0.5
        angular_vel = max(-max_angular_vel, min(max_angular_vel, angular_vel))
        
        # Log alignment progress every 0.5 seconds
        if int((time.time() - self.align_start_time) * 2) % 1 == 0:
            if logger and not self.aligned:
                logger.info(f'  ↻ Allineamento in corso: errore={abs(angle_diff)*180/math.pi:.2f}°, vel={angular_vel:.2f} rad/s')
        
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = angular_vel
        
        if cmd_vel_pub:
            cmd_vel_pub(msg)
        
        return Status.RUNNING


class ShouldPlanReturn(py_trees.behaviour.Behaviour):
    """
    Condition node that checks if robot should plan return path.
    """
    
    def __init__(self, name="ShouldPlanReturn"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="object_collected", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="return_planned", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="path_queue", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="current_target", access=py_trees.common.Access.READ)
    
    def update(self):
        object_collected = self.blackboard.get("object_collected")
        return_planned = self.blackboard.get("return_planned")
        path_queue = self.blackboard.get("path_queue")
        current_target = self.blackboard.get("current_target")
        
        # Should plan return if object collected, not yet planned, and no active navigation
        if object_collected and not return_planned and not path_queue and current_target is None:
            return Status.SUCCESS
        return Status.FAILURE


class PlanSimpleReturn(py_trees.behaviour.Behaviour):
    """
    Action node that plans return path by reversing original path.
    """
    
    def __init__(self, name="PlanSimpleReturn"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="original_complete_path", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="path_queue", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="return_planned", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="logger", access=py_trees.common.Access.READ)
    
    def update(self):
        original_path = self.blackboard.get("original_complete_path")
        logger = self.blackboard.get("logger")
        
        # Percorso di ritorno inizia da (4,0) dove c'è l'AprilTag, non da (4,1)
        # (4,2) → (4,0) → (3,0) → (2,0) → (1,0) → (0,0)
        # Inverte il percorso BFS originale così da ripercorrere le stesse celle (adiacenti) al ritorno
        return_path = list(reversed(original_path))
        if return_path and original_path:
            # Salta la cella corrente (goal) perché ci siamo già
            return_path.pop(0)
        
        self.blackboard.set("path_queue", return_path)
        self.blackboard.set("return_planned", True)
        
        if logger:
            logger.info(f'🔙 Percorso di ritorno: {return_path} (usa AprilTag in 4,0)')
        
        return Status.SUCCESS


class CollectObject(py_trees.behaviour.Behaviour):
    """
    Action node that simulates collecting an object with the gripper.
    """
    
    def __init__(self, name="CollectObject", battery_cost=3.0, collection_time=3.0):
        """
        Initialize the collect object action.
        
        Args:
            name: Name of the behavior
            battery_cost: Battery consumption for this action (percentage)
            collection_time: Time in seconds to collect the object
        """
        super(CollectObject, self).__init__(name)
        self.battery_cost = battery_cost
        self.collection_time = collection_time
        self.start_time = None
        self.feedback_message = ""
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="battery_level",
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="object_collected",
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="logger",
            access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="cmd_vel_publisher",
            access=py_trees.common.Access.READ
        )
    
    def setup(self, **kwargs):
        """Setup the behavior (called once before first tick)."""
        pass
    
    def initialise(self):
        """Called when behavior switches from INVALID to RUNNING."""
        import time
        self.start_time = time.time()
        logger = self.blackboard.get("logger")
        if logger:
            logger.info('🤖 Inizio raccolta oggetto con il gripper...')
    
    def update(self):
        """
        Simulate collecting object with gripper.
        
        Returns:
            Status.RUNNING while collecting, Status.SUCCESS when done
        """
        import time
        
        battery_level = self.blackboard.get("battery_level")
        logger = self.blackboard.get("logger")
        cmd_vel_pub = self.blackboard.get("cmd_vel_publisher")
        
        # Ensure robot is stopped
        if cmd_vel_pub:
            from geometry_msgs.msg import Twist
            msg = Twist()
            cmd_vel_pub(msg)
        
        elapsed_time = time.time() - self.start_time
        
        if elapsed_time < self.collection_time:
            # Still collecting
            progress = (elapsed_time / self.collection_time) * 100
            self.feedback_message = f"Collecting object... {progress:.0f}%"
            
            # Log progress every second
            if int(elapsed_time) != int(elapsed_time - 0.1):
                if logger:
                    logger.info(f'  📦 Raccolta in corso: {progress:.0f}%')
            
            return Status.RUNNING
        else:
            # Collection complete
            # Consume battery
            new_level = max(0.0, battery_level - self.battery_cost)
            self.blackboard.set("battery_level", new_level)
            self.blackboard.set("object_collected", True)
            
            self.feedback_message = f"Object collected! (battery: {new_level:.1f}%)"
            
            if logger:
                logger.info(f'✅ Oggetto raccolto con successo! (batteria: {new_level:.1f}%)')
            
            return Status.SUCCESS
    
    def terminate(self, new_status):
        """Called when behavior terminates."""
        pass


# ============================================================================
# BEHAVIOR TREE CREATION
# ============================================================================

def create_courier_behavior_tree(cell_size=1.0):
    """
    Create the courier behavior tree structure.
    
    Structure:
        Root (Sequence)
        ├── Battery Manager (Selector)
        │   ├── BatteryOK (>= 20%)
        │   └── GoCharge
        └── Mission Selector (Selector)
            ├── Navigation Sequence
            │   ├── HasTarget
            │   └── Navigate to Target (Selector)
            │       ├── Reach Target Sequence
            │       │   ├── IsAtTarget
            │       │   └── ClearTarget
            │       └── Move Sequence
            │           ├── Rotation Sequence
            │           │   ├── IsAligned
            │           │   └── RotateToTarget
            │           └── MoveToTarget
            ├── Get Next Waypoint Sequence
            │   ├── HasPathQueue
            │   └── GetNextTarget
            ├── Goal Operations Sequence
            │   ├── IsAtGoal
            │   └── Collection Check (Selector)
            │       ├── ObjectCollected
            │       └── CollectObject
            ├── Return Planning Sequence
            │   ├── NeedToReturnHome
            │   └── PlanReturnPath
            ├── Home Arrival Sequence
            │   ├── IsAtHome
            │   ├── MarkReturnedHome
            │   └── StopAtHome
            └── StopRobotFallback
    
    Returns:
        The root node of the behavior tree
    """
    # Root sequence
    root = Sequence(name="Root", memory=False)
    
    # Battery management selector
    battery_selector = Selector(name="Battery Manager", memory=False)
    battery_ok = BatteryOK(name="BatteryOK", battery_threshold=20.0)
    go_charge = GoCharge(name="GoCharge", charge_rate=30.0)
    battery_selector.add_children([battery_ok, go_charge])
    
    # Mission selector - chooses what to do
    mission_selector = Selector(name="Mission Selector", memory=False)
    
    # 1. If has target, navigate to it
    navigation_sequence = Sequence(name="Navigation", memory=False)
    has_target = HasTarget(name="HasTarget")
    
    # Navigate to target selector
    navigate_selector = Selector(name="Navigate to Target", memory=False)
    
    # Option A: Already at target - clear it
    reach_sequence = Sequence(name="Reach Target", memory=False)
    is_at_target = IsAtTarget(name="IsAtTarget", dist_tolerance=0.10)
    clear_target = ClearTarget(name="ClearTarget")
    reach_sequence.add_children([is_at_target, clear_target])
    
    # Option B: Not at target - move toward it
    move_sequence = Sequence(name="Move to Target", memory=False)
    
    # First check alignment, then move
    rotation_selector = Selector(name="Rotation Check", memory=False)
    # Tolleranza più stretta: ruota completamente prima di avanzare
    is_aligned = IsAligned(name="IsAligned", angle_tolerance=0.05)
    rotate_to_target = RotateToTarget(name="RotateToTarget", battery_cost=0.05)
    rotation_selector.add_children([is_aligned, rotate_to_target])
    
    move_to_target = MoveToTarget(name="MoveToTarget", battery_cost=0.1)
    
    move_sequence.add_children([rotation_selector, move_to_target])
    
    navigate_selector.add_children([reach_sequence, move_sequence])
    navigation_sequence.add_children([has_target, navigate_selector])
    
    # 2. If no target but has path queue, get next target
    get_waypoint_sequence = Sequence(name="Get Next Waypoint", memory=False)
    has_path_queue = HasPathQueue(name="HasPathQueue")
    get_next_target = GetNextTarget(name="GetNextTarget", cell_size=cell_size)
    get_waypoint_sequence.add_children([has_path_queue, get_next_target])
    
    # 3. If at goal, collect object (if not already collected)
    goal_sequence = Sequence(name="Goal Operations", memory=False)
    is_at_goal = IsAtGoal(name="IsAtGoal")
    
    # Collection selector: skip if already collected
    collection_selector = Selector(name="Collection Check", memory=False)
    object_collected = ObjectCollected(name="ObjectCollected")
    collect_object = CollectObject(name="CollectObject", battery_cost=3.0, collection_time=3.0)
    collection_selector.add_children([object_collected, collect_object])
    
    goal_sequence.add_children([is_at_goal, collection_selector])
    
    # 4. If object collected and return not planned, plan return path and align
    return_sequence = Sequence(name="Plan Return", memory=True)  # MEMORY per permettere AlignForReturn di completare
    should_plan_return = ShouldPlanReturn(name="ShouldPlanReturn")
    plan_simple_return = PlanSimpleReturn(name="PlanSimpleReturn")
    align_for_return = AlignForReturn(name="AlignForReturn", cell_size=cell_size)
    return_sequence.add_children([should_plan_return, plan_simple_return, align_for_return])
    
    # 5. Stop when back at start after collecting
    class AtStartAfterReturn(py_trees.behaviour.Behaviour):
        def __init__(self):
            super().__init__("AtStartAfterReturn")
            self.blackboard = self.attach_blackboard_client(name=self.name)
            self.blackboard.register_key(key="return_planned", access=py_trees.common.Access.READ)
            self.blackboard.register_key(key="path_queue", access=py_trees.common.Access.READ)
            self.blackboard.register_key(key="current_target", access=py_trees.common.Access.READ)
        
        def update(self):
            return_planned = self.blackboard.get("return_planned")
            path_queue = self.blackboard.get("path_queue")
            current_target = self.blackboard.get("current_target")
            
            if return_planned and not path_queue and current_target is None:
                return Status.SUCCESS
            return Status.FAILURE
    
    final_stop_sequence = Sequence(name="Final Stop", memory=False)
    at_start_after_return = AtStartAfterReturn()
    final_stop = StopRobot(name="FinalStop")
    final_stop_sequence.add_children([at_start_after_return, final_stop])
    
    # 6. Fallback: just stop
    stop_robot_fallback = StopRobot(name="StopRobotFallback")
    
    # Add all mission options to selector
    mission_selector.add_children([
        navigation_sequence,        # Navigate to current target
        get_waypoint_sequence,      # Get next waypoint from path
        goal_sequence,              # Collect object at goal
        return_sequence,            # Plan return path after collection
        final_stop_sequence,        # Stop when returned to start
        stop_robot_fallback         # Fallback
    ])
    
    # Build the tree
    root.add_children([battery_selector, mission_selector])
    
    return root


