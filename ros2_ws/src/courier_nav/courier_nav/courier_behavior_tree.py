"""
Complete Mission Behavior Tree (according to behavioural tree.mmd)
"""

import py_trees
from py_trees.composites import Sequence, Selector
from py_trees.common import Status
from collections import deque

# ============================================================================
# CONDITION NODES
# ============================================================================

class SystemReady(py_trees.behaviour.Behaviour):
    """Check if all systems initialized"""
    def __init__(self, name="SystemReady"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="system_ready", access=py_trees.common.Access.READ)
    
    def update(self):
        ready = self.blackboard.get("system_ready")
        return Status.SUCCESS if ready else Status.FAILURE


class AtTarget(py_trees.behaviour.Behaviour):
    """Check if robot at target"""
    def __init__(self, name="AtTarget", tolerance=0.15):
        super().__init__(name)
        self.tolerance = tolerance
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="distance_to_target", access=py_trees.common.Access.READ)
    
    def update(self):
        distance = self.blackboard.get("distance_to_target")
        return Status.SUCCESS if distance <= self.tolerance else Status.FAILURE


class ObjectSecured(py_trees.behaviour.Behaviour):
    """Check if object grasped"""
    def __init__(self, name="ObjectSecured"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="gripper_force", access=py_trees.common.Access.READ)
    
    def update(self):
        force = self.blackboard.get("gripper_force")
        return Status.SUCCESS if force > 5.0 else Status.FAILURE


class AtBase(py_trees.behaviour.Behaviour):
    """Check if robot at base"""
    def __init__(self, name="AtBase", tolerance=0.15):
        super().__init__(name)
        self.tolerance = tolerance
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="distance_to_base", access=py_trees.common.Access.READ)
    
    def update(self):
        distance = self.blackboard.get("distance_to_base")
        return Status.SUCCESS if distance <= self.tolerance else Status.FAILURE


class PathComplete(py_trees.behaviour.Behaviour):
    """Check if path is complete (all waypoints visited)"""
    def __init__(self, name="PathComplete"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="path_queue", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="logger", access=py_trees.common.Access.READ)
    
    def update(self):
        path = self.blackboard.get("path_queue")
        if not path or len(path) == 0:
            logger = self.blackboard.get("logger")
            if logger:
                logger.info(f"{self.name}: All waypoints reached!")
            return Status.SUCCESS
        return Status.FAILURE


# ============================================================================
# ACTION NODES
# ============================================================================

class InitSensors(py_trees.behaviour.Behaviour):
    """Initialize robot sensors"""
    def __init__(self, name="InitSensors"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="logger", access=py_trees.common.Access.READ)
    
    def update(self):
        logger = self.blackboard.get("logger")
        if logger:
            logger.info(f"{self.name}: Sensors initialized")
        return Status.SUCCESS


class SetArmPose(py_trees.behaviour.Behaviour):
    """Move arm to target pose"""
    def __init__(self, name="SetArmPose", pose="HOME"):
        super().__init__(name)
        self.pose = pose
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="arm_controller", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="logger", access=py_trees.common.Access.READ)
    
    def update(self):
        arm = self.blackboard.get("arm_controller")
        logger = self.blackboard.get("logger")
        
        if arm:
            arm.move_to(self.pose)
        
        if logger:
            logger.info(f"{self.name}: Arm → {self.pose}")
        
        return Status.SUCCESS


def compute_bfs_path(grid_map, start, goal):
    """
    Compute BFS path from start to goal on grid_map.
    
    Args:
        grid_map: 2D list (0=free, 1=obstacle)
        start: tuple (row, col)
        goal: tuple (row, col)
    
    Returns:
        List of (row, col) waypoints
    """
    rows = len(grid_map)
    cols = len(grid_map[0])
    
    # BFS
    queue = deque([start])
    visited = {start}
    parent = {start: None}
    
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # right, down, left, up
    
    while queue:
        current = queue.popleft()
        
        if current == goal:
            # Reconstruct path
            path = []
            while current is not None:
                path.append(current)
                current = parent[current]
            return list(reversed(path))
        
        r, c = current
        for dr, dc in directions:
            nr, nc = r + dr, c + dc
            
            # Check bounds
            if 0 <= nr < rows and 0 <= nc < cols:
                # Check if free and not visited
                if grid_map[nr][nc] == 0 and (nr, nc) not in visited:
                    visited.add((nr, nc))
                    parent[(nr, nc)] = current
                    queue.append((nr, nc))
    
    return []  # No path found


class LoadGridBFS(py_trees.behaviour.Behaviour):
    """Load grid and compute BFS path"""
    def __init__(self, name="LoadGridBFS"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="path_queue", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="grid_map", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="start_cell", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="goal_cell", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="cell_size", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="logger", access=py_trees.common.Access.READ)
    
    def update(self):
        logger = self.blackboard.get("logger")
        
        # Get grid configuration from blackboard
        grid_map = self.blackboard.get("grid_map")
        start_cell = self.blackboard.get("start_cell")
        goal_cell = self.blackboard.get("goal_cell")
        
        # Compute BFS path
        path = compute_bfs_path(grid_map, start_cell, goal_cell)
        
        if not path:
            if logger:
                logger.error(f"{self.name}: ❌ No path found from {start_cell} to {goal_cell}!")
            return Status.FAILURE
        
        self.blackboard.set("path_queue", path)
        
        if logger:
            logger.info(f"{self.name}: ✅ Path computed = {len(path)} waypoints: {path}")
            # 🆕 Debug: Show converted world coordinates for each waypoint
            cell_size = self.blackboard.get("cell_size")
            rows = len(grid_map)
            logger.info(f"{self.name}: 📍 Path in world coordinates:")
            for i, (r, c) in enumerate(path):
                wx = (r + 0.5) * cell_size  # Row → X in Gazebo
                wy = (c + 0.5) * cell_size  # Column → Y in Gazebo
                logger.info(f"{self.name}:    [{i}] Cell({r},{c}) → World({wx:.2f},{wy:.2f})")
        
        return Status.SUCCESS


class FollowBFSPath(py_trees.behaviour.Behaviour):
    """Follow BFS path cell by cell"""
    def __init__(self, name="FollowBFSPath"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="path_queue", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="target_publisher", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="cell_size", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="grid_map", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="current_target", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="logger", access=py_trees.common.Access.READ)
    
    def update(self):
        path = self.blackboard.get("path_queue")
        logger = self.blackboard.get("logger")
        
        if not path or len(path) == 0:
            if logger:
                logger.info(f"{self.name}: ✅ Tutti i waypoint raggiunti!")
            return Status.SUCCESS  # Path completed
        
        # Get next cell (row, col)
        next_cell = path[0]
        row, col = next_cell
        
        # 🔄 Swap X/Y for Gazebo coordinate system
        cell_size = self.blackboard.get("cell_size")
        target_x = (row + 0.5) * cell_size  # Row → X in Gazebo
        target_y = (col + 0.5) * cell_size  # Column → Y in Gazebo
        
        # Publish target
        target_pub = self.blackboard.get("target_publisher")
        if target_pub:
            target_pub(target_x, target_y)
            if logger:
                logger.info(f"{self.name}: 📍 Waypoint {len(path)}/{len(path)}: Cell({row},{col}) → World({target_x:.2f},{target_y:.2f})")
        
        # Store current target for distance calculation
        self.blackboard.set("current_target", (target_x, target_y))
        
        return Status.RUNNING

class AlignWithAprilTag(py_trees.behaviour.Behaviour):
    """Align with AprilTag"""
    def __init__(self, name="AlignWithAprilTag"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="apriltag_pose", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="logger", access=py_trees.common.Access.READ)
    
    def update(self):
        tag_pose = self.blackboard.get("apriltag_pose")
        logger = self.blackboard.get("logger")
        
        if tag_pose is None:
            return Status.RUNNING  # Still searching for tag
        
        # TODO: Implement alignment logic
        
        if logger:
            logger.info(f"{self.name}: Aligned with AprilTag")
        
        return Status.SUCCESS


class ControlGripper(py_trees.behaviour.Behaviour):
    """Control gripper (open/close)"""
    def __init__(self, name="ControlGripper", action="CLOSE"):
        super().__init__(name)
        self.action = action
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="gripper_controller", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="logger", access=py_trees.common.Access.READ)
    
    def update(self):
        gripper = self.blackboard.get("gripper_controller")
        logger = self.blackboard.get("logger")
        
        if gripper:
            if self.action == "CLOSE":
                gripper.close()
            else:
                gripper.open()
        
        if logger:
            logger.info(f"{self.name}: Gripper {self.action}")
        
        return Status.SUCCESS


class RemoveReachedWaypoint(py_trees.behaviour.Behaviour):
    """Remove reached waypoint from path queue"""
    def __init__(self, name="RemoveReachedWaypoint"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="path_queue", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="logger", access=py_trees.common.Access.READ)
    
    def update(self):
        path = self.blackboard.get("path_queue")
        if path and len(path) > 0:
            removed = path.pop(0)
            self.blackboard.set("path_queue", path)
            
            logger = self.blackboard.get("logger")
            if logger:
                logger.info(f"{self.name}: Waypoint {removed} reached! {len(path)} remaining")
        
        return Status.SUCCESS



# ============================================================================
# TREE BUILDER
# ============================================================================

def create_mission_behavior_tree():
    """
    Create mission BT according to behavioural tree.mmd
    """
    
    # Root sequence
    root = Sequence(name="ROOT", memory=True)
    mission = Sequence(name="MISSION", memory=True)
    
    # 1. INIT Sequence (eseguita una sola volta)
    init_seq = Sequence(name="INIT", memory=True)
    init_seq.add_children([
        SystemReady(name="SystemReady"),
        InitSensors(name="InitSensors"),
        SetArmPose(name="ArmHome", pose="HOME"),
        LoadGridBFS(name="LoadGridBFS")
    ])
    
    # 2. GO_TARGET - Loop continuo per navigare tutti i waypoint
    go_target_seq = Sequence(name="GO_TARGET", memory=True)
    
    # Loop finché path non è completo
    # Usa un nodo personalizzato che gestisce il loop interno
    nav_loop = NavigateAllWaypoints(name="NavigateAllWaypoints")
    
    go_target_seq.add_children([
        nav_loop,
        AlignWithAprilTag(name="AlignWithAprilTag")
    ])
    
    # 3. PICKUP Sequence (invariato)
    pickup_seq = Sequence(name="PICKUP", memory=False)
    pickup_seq.add_children([
        SetArmPose(name="ArmGrip", pose="GRIP"),
        ControlGripper(name="CloseGripper", action="CLOSE"),
        ObjectSecured(name="ObjectSecured"),
        SetArmPose(name="ArmCarry", pose="CARRY")
    ])
    
    # 4. RETURN_BASE Sequence (invariato)
    return_seq = Sequence(name="RETURN_BASE", memory=False)
    return_sel = Selector(name="ReturnSelector", memory=False)
    return_sel.add_children([
        AtBase(name="AtBase"),
        FollowBFSPath(name="NavigateBack")
    ])
    return_seq.add_children([
        return_sel,
        AlignWithAprilTag(name="AlignAtBase")
    ])
    
    # 5. RELEASE Sequence (invariato)
    release_seq = Sequence(name="RELEASE", memory=False)
    release_seq.add_children([
        SetArmPose(name="ArmRelease", pose="RELEASE"),
        ControlGripper(name="OpenGripper", action="OPEN"),
        SetArmPose(name="ArmHomeEnd", pose="HOME")
    ])
    
    # Build mission
    mission.add_children([
        init_seq,
        go_target_seq,
        pickup_seq,
        return_seq,
        release_seq
    ])
    
    root.add_child(mission)
    
    return root

class NavigateAllWaypoints(py_trees.behaviour.Behaviour):
    """Navigate through all waypoints in path_queue"""
    def __init__(self, name="NavigateAllWaypoints"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="path_queue", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="target_publisher", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="cell_size", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="grid_map", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="current_pose", access=py_trees.common.Access.READ)  # 🆕
        self.blackboard.register_key(key="logger", access=py_trees.common.Access.READ)
        self.target_tolerance = 0.15
        self.current_target_world = None  # 🆕 Salva il target in coordinate mondo
    
    def update(self):
        import math  # 🆕
        
        path = self.blackboard.get("path_queue")
        logger = self.blackboard.get("logger")
        
        # Check if path complete
        if not path or len(path) == 0:
            if logger:
                logger.info(f"{self.name}: ✅ Tutti i waypoint raggiunti!")
            return py_trees.common.Status.SUCCESS
        
        # Get current waypoint
        current_waypoint = path[0]
        row, col = current_waypoint
        
        # Convert to world coordinates (swap X/Y for Gazebo)
        cell_size = self.blackboard.get("cell_size")
        target_x = (row + 0.5) * cell_size  # Row → X in Gazebo
        target_y = (col + 0.5) * cell_size  # Column → Y in Gazebo
        
        # 🆕 Get current pose FIRST before using it
        current_pose = self.blackboard.get("current_pose")
        
        # 🆕 Debug: verifica current_pose
        if current_pose is None:
            if logger:
                logger.warn(f"{self.name}: ⚠️ current_pose è None!")
            return py_trees.common.Status.FAILURE
        
        # 🆕 Se è un nuovo target, pubblicalo
        if self.current_target_world != (target_x, target_y):
            self.current_target_world = (target_x, target_y)
            
            # Pubblica target per PID
            target_pub = self.blackboard.get("target_publisher")
            if target_pub:
                target_pub(target_x, target_y)
            
            if logger:
                # 🆕 Show current robot position when setting new target
                logger.info(f"{self.name}: 🚀 Nuovo waypoint: Cell({row},{col}) → World({target_x:.2f},{target_y:.2f})")
                logger.info(f"{self.name}: 📍 Robot attualmente a: ({current_pose['x']:.2f},{current_pose['y']:.2f})")
        
        # 🆕 Calcola distanza DIRETTAMENTE dalla posizione corrente
        dx = target_x - current_pose['x']
        dy = target_y - current_pose['y']
        distance = math.sqrt(dx**2 + dy**2)
        
        # Log periodicamente per debug
        if not hasattr(self, '_last_log_time'):
            import time
            self._last_log_time = time.time()
        
        import time
        if time.time() - self._last_log_time > 1.0:  # Ogni secondo
            if logger:
                logger.info(
                    f"{self.name}: 📏 Cell({row},{col}): "
                    f"Robot@({current_pose['x']:.3f},{current_pose['y']:.3f}) → "
                    f"Target@({target_x:.2f},{target_y:.2f}) = "
                    f"dist={distance:.3f}m (tol={self.target_tolerance}m)"
                )
            self._last_log_time = time.time()
        
        # Check if waypoint reached
        if distance <= self.target_tolerance:
            # Waypoint reached, remove it
            path.pop(0)
            self.blackboard.set("path_queue", path)
            self.current_target_world = None  # 🆕 Reset per prossimo waypoint
            
            if logger:
                logger.info(f"{self.name}: ✅ Waypoint Cell({row},{col}) raggiunto! {len(path)} rimanenti")
            
            # If more waypoints, continue to next
            if len(path) > 0:
                return py_trees.common.Status.RUNNING
            else:
                if logger:
                    logger.info(f"{self.name}: 🎉 Missione completata!")
                return py_trees.common.Status.SUCCESS
        else:
            # Still navigating to current waypoint
            if logger and hasattr(self, '_last_log_time'):
                import time
                if time.time() - self._last_log_time > 2.0:  # Log ogni 2 secondi
                    logger.info(f"{self.name}: 📍 Navigando verso Cell({row},{col}) = ({target_x:.2f},{target_y:.2f}), distanza={distance:.2f}m")
                    self._last_log_time = time.time()
            elif not hasattr(self, '_last_log_time'):
                import time
                self._last_log_time = time.time()
            
            return py_trees.common.Status.RUNNING