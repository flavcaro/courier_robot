#!/usr/bin/env python3
"""
Courier Robot Behavior Tree with Nav2 Integration

This module implements a py_trees behavior tree that uses Nav2's
navigation stack for path planning and execution, while maintaining
the high-level mission logic in the behavior tree structure.
"""

import py_trees
from py_trees import common
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

import math
import time
import logging


# Lightweight behavior-tree logger adapter that prefixes messages and throttles
class BTLoggerAdapter:
    def __init__(self, raw_logger, min_interval: float = 1.0):
        self.raw = raw_logger
        self.min_interval = float(min_interval)
        self._last_emit = {}

    def _should_emit(self, key: str) -> bool:
        now = time.time()
        last = self._last_emit.get(key, 0.0)
        if (now - last) >= self.min_interval:
            self._last_emit[key] = now
            return True
        return False

    def info(self, msg: str, key: str | None = None):
        k = key or msg
        if self._should_emit(k):
            try:
                self.raw.info(f"[BT] {msg}")
            except Exception:
                logging.getLogger('courier_bt').info(msg)

    def warn(self, msg: str, key: str | None = None):
        k = key or msg
        if self._should_emit(k):
            try:
                self.raw.warn(f"[BT] {msg}")
            except Exception:
                logging.getLogger('courier_bt').warning(msg)

    def debug(self, msg: str, key: str | None = None):
        k = key or msg
        if self._should_emit(k):
            try:
                # Prefer debug-level emission when available
                if hasattr(self.raw, 'debug'):
                    self.raw.debug(f"[BT] {msg}")
                else:
                    self.raw.info(f"[BT] {msg}")
            except Exception:
                logging.getLogger('courier_bt').debug(msg)


def get_bt_logger_from_blackboard(blackboard):
    raw = blackboard.get('logger')
    if raw is None:
        return BTLoggerAdapter(logging.getLogger('courier_bt'))
    if isinstance(raw, BTLoggerAdapter):
        return raw
    return BTLoggerAdapter(raw)


# ============================================================================
# NAV2 NAVIGATION BEHAVIORS
# ============================================================================

class NavigateToCell(py_trees.behaviour.Behaviour):
    """
    Navigate to a grid cell using Nav2.
    
    This behavior sends a navigation goal to Nav2 and monitors its progress.
    It returns RUNNING while navigating, SUCCESS when reached, FAILURE on error.
    """
    
    def __init__(self, name: str, cell_key: str = "current_target"):
        """
        Args:
            name: Behavior name
            cell_key: Blackboard key containing target cell (row, col)
        """
        super().__init__(name)
        self.cell_key = cell_key
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key=cell_key, access=common.Access.READ)
        self.blackboard.register_key(key="cell_size", access=common.Access.READ)
        self.blackboard.register_key(key="nav2_client", access=common.Access.READ)
        self.blackboard.register_key(key="node", access=common.Access.READ)
        self.blackboard.register_key(key="logger", access=common.Access.READ)
        
        self.goal_handle = None
        self.navigation_complete = False
        self.navigation_succeeded = False
        
    def setup(self, **kwargs):
        """Setup is called once when the tree is first started."""
        pass
        
    def initialise(self):
        """Called when behavior transitions from not RUNNING to RUNNING."""
        self.navigation_complete = False
        self.navigation_succeeded = False
        self.goal_handle = None
        
        # Get target cell from blackboard
        target_cell = self.blackboard.get(self.cell_key)
        if target_cell is None:
            self.feedback_message = "No target cell specified"
            return
            
        cell_size = self.blackboard.get("cell_size")
        nav2_client = self.blackboard.get("nav2_client")
        logger = get_bt_logger_from_blackboard(self.blackboard)
        
        # Convert cell to world coordinates
        # Grid uses (row, col) where row -> Y and col -> X (row is north/south)
        row, col = target_cell
        x = (col + 0.5) * cell_size  # Center of cell (col -> X)
        y = (row + 0.5) * cell_size  # (row -> Y)
        
        logger.info(f"[{self.name}] Navigating to cell {target_cell} -> ({x:.2f}, {y:.2f})")
        
        # Create goal pose in the map frame for Nav2 global planning
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.blackboard.get("node").get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        
        # Send goal to Nav2
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        send_goal_future = nav2_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        send_goal_future.add_done_callback(self._goal_response_callback)
        
    def _goal_response_callback(self, future):
        """Handle goal acceptance/rejection from Nav2."""
        self.goal_handle = future.result()
        logger = get_bt_logger_from_blackboard(self.blackboard)
        
        # Share goal handle with mission controller for obstacle handling
        node = self.blackboard.get("node")
        if hasattr(node, 'nav2_goal_handle'):
            node.nav2_goal_handle = self.goal_handle
        
        if not self.goal_handle.accepted:
            logger.error(f"[{self.name}] Navigation goal rejected!")
            self.navigation_complete = True
            self.navigation_succeeded = False
            return
        
        logger.info(f"[{self.name}] Navigation goal accepted, waiting for result...")
            
        # Get result asynchronously
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)
        
    def _feedback_callback(self, feedback_msg):
        """Handle navigation feedback."""
        feedback = feedback_msg.feedback
        logger = get_bt_logger_from_blackboard(self.blackboard)
        # Log distance remaining at debug level to avoid flooding the console
        dist = feedback.distance_remaining
        logger.debug(f"[{self.name}] Distance remaining: {dist:.2f}m")
        
    def _result_callback(self, future):
        """Handle navigation completion."""
        result = future.result()
        status = result.status
        
        self.navigation_complete = True
        self.navigation_succeeded = (status == GoalStatus.STATUS_SUCCEEDED)
        
        logger = get_bt_logger_from_blackboard(self.blackboard)
        if self.navigation_succeeded:
            logger.info(f"[{self.name}] Navigation succeeded!")
        else:
            logger.warn(f"[{self.name}] Navigation failed with status: {status}")
        
    def update(self):
        """Called every tick while behavior is RUNNING."""
        if not self.navigation_complete:
            return py_trees.common.Status.RUNNING
            
        if self.navigation_succeeded:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
            
    def terminate(self, new_status):
        """Called when behavior transitions away from RUNNING."""
        # Cancel navigation if we're being preempted
        if self.goal_handle is not None and not self.navigation_complete:
            logger = get_bt_logger_from_blackboard(self.blackboard)
            logger.info(f"[{self.name}] Cancelling navigation")
            self.goal_handle.cancel_goal_async()


class GetNextWaypoint(py_trees.behaviour.Behaviour):
    """
    Get the next waypoint from the path queue and set it as current target.
    """
    
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="path_queue", access=common.Access.WRITE)
        self.blackboard.register_key(key="current_target", access=common.Access.WRITE)
        self.blackboard.register_key(key="logger", access=common.Access.READ)
        
    def update(self):
        path_queue = self.blackboard.get("path_queue")
        logger = get_bt_logger_from_blackboard(self.blackboard)
        
        if not path_queue:
            logger.info(f"[{self.name}] Path queue empty")
            return py_trees.common.Status.FAILURE
            
        next_cell = path_queue.pop(0)
        self.blackboard.set("current_target", next_cell)
        self.blackboard.set("path_queue", path_queue)
        
        logger.info(f"[{self.name}] Next waypoint: {next_cell}, remaining: {len(path_queue)}")
        return py_trees.common.Status.SUCCESS


class IsPathComplete(py_trees.behaviour.Behaviour):
    """Check if the path queue is empty (all waypoints visited)."""
    
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="path_queue", access=common.Access.READ)
        
    def update(self):
        path_queue = self.blackboard.get("path_queue")
        if not path_queue:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


# ============================================================================
# MISSION BEHAVIORS
# ============================================================================

class CheckBattery(py_trees.behaviour.Behaviour):
    """Check if battery level is sufficient."""
    
    def __init__(self, name: str, min_level: float = 20.0):
        super().__init__(name)
        self.min_level = min_level
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="battery_level", access=common.Access.READ)
        self.blackboard.register_key(key="logger", access=common.Access.READ)
        
    def update(self):
        battery = self.blackboard.get("battery_level")
        logger = get_bt_logger_from_blackboard(self.blackboard)
        
        if battery >= self.min_level:
            return py_trees.common.Status.SUCCESS
        else:
            logger.warn(f"[{self.name}] Battery low: {battery}%")
            return py_trees.common.Status.FAILURE


class CollectObject(py_trees.behaviour.Behaviour):
    """Simulate collecting an object at the current location."""
    
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="object_collected", access=common.Access.WRITE)
        self.blackboard.register_key(key="logger", access=common.Access.READ)
        self.collection_time = 0
        
    def initialise(self):
        self.collection_time = 0
        logger = get_bt_logger_from_blackboard(self.blackboard)
        logger.info(f"[{self.name}] Starting object collection...")
        
    def update(self):
        self.collection_time += 1
        logger = get_bt_logger_from_blackboard(self.blackboard)
        # Simulate collection taking 2 seconds (20 ticks at 10Hz)
        if self.collection_time >= 20:
            self.blackboard.set("object_collected", True)
            logger = get_bt_logger_from_blackboard(self.blackboard)
            logger.info(f"[{self.name}] Object collected!")
            return py_trees.common.Status.SUCCESS
            
        return py_trees.common.Status.RUNNING


class IsObjectCollected(py_trees.behaviour.Behaviour):
    """Check if object has been collected."""
    
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="object_collected", access=common.Access.READ)
        
    def update(self):
        if self.blackboard.get("object_collected"):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class DeliverObject(py_trees.behaviour.Behaviour):
    """Simulate delivering an object."""
    
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="object_collected", access=common.Access.WRITE)
        self.blackboard.register_key(key="mission_complete", access=common.Access.WRITE)
        self.blackboard.register_key(key="logger", access=common.Access.READ)
        self.delivery_time = 0
        
    def initialise(self):
        self.delivery_time = 0
        logger = get_bt_logger_from_blackboard(self.blackboard)
        logger.info(f"[{self.name}] Starting delivery...")
        
    def update(self):
        self.delivery_time += 1
        
        if self.delivery_time >= 20:
            self.blackboard.set("object_collected", False)
            self.blackboard.set("mission_complete", True)
            logger = get_bt_logger_from_blackboard(self.blackboard)
            logger.info(f"[{self.name}] Object delivered! Mission complete!")
            return py_trees.common.Status.SUCCESS
            
        return py_trees.common.Status.RUNNING


class PlanReturnPath(py_trees.behaviour.Behaviour):
    """Plan the return path from current location to home."""
    
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="path_queue", access=common.Access.WRITE)
        self.blackboard.register_key(key="start_cell", access=common.Access.READ)
        self.blackboard.register_key(key="return_planned", access=common.Access.WRITE)
        self.blackboard.register_key(key="logger", access=common.Access.READ)
        
    def update(self):
        logger = get_bt_logger_from_blackboard(self.blackboard)
        start_cell = self.blackboard.get("start_cell")
        grid_size = 5

        # Determine current cell: prefer explicit blackboard 'current_target' (last reached),
        # otherwise fallback to start_cell.
        current = self.blackboard.get("current_target")
        if current is None:
            current = start_cell

        # Obstacles from blackboard (list of (row,col)). If not present,
        # try to derive from a grid_map on the blackboard.
        obstacles_list = self.blackboard.get("obstacles")
        if obstacles_list is None:
            grid_map = self.blackboard.get("grid_map")
            if grid_map:
                # derive obstacles from grid_map (1 indicates obstacle)
                inferred = []
                for r in range(len(grid_map)):
                    for c in range(len(grid_map[0])):
                        if grid_map[r][c] == 1:
                            inferred.append((r, c))
                obstacles_list = inferred
                logger.debug(f"[{self.name}] Inferred obstacles from grid_map: {obstacles_list}")
            else:
                obstacles_list = []

        obstacles = set(obstacles_list or [])
        logger.debug(f"[{self.name}] Planning return from {self.blackboard.get('current_target') or 'UNKNOWN'} to {start_cell}; obstacles={obstacles}")

        # BFS from current -> start_cell on a 4-connected grid
        from collections import deque

        def bfs(start, goal):
            if start == goal:
                return []
            dirs = [(1, 0), (-1, 0), (0, -1), (0, 1)]
            q = deque([(start, [start])])
            visited = {start}
            while q:
                (r, c), path = q.popleft()
                for dr, dc in dirs:
                    nr, nc = r + dr, c + dc
                    nxt = (nr, nc)
                    if not (0 <= nr < grid_size and 0 <= nc < grid_size):
                        continue
                    if nxt in obstacles or nxt in visited:
                        continue
                    new_path = path + [nxt]
                    if nxt == goal:
                        return new_path[1:]
                    visited.add(nxt)
                    q.append((nxt, new_path))
            return []

        return_path = bfs(current, start_cell)
        if not return_path:
            logger.warn(f'[{self.name}] No return path found from {current} to {start_cell}')
            return py_trees.common.Status.FAILURE

        self.blackboard.set("path_queue", return_path)
        self.blackboard.set("return_planned", True)
        logger.info(f"[{self.name}] Return path set (dynamic BFS): {return_path}")
        return py_trees.common.Status.SUCCESS


class IsReturnPlanned(py_trees.behaviour.Behaviour):
    """Check if return path has been planned."""
    
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="return_planned", access=common.Access.READ)
        
    def update(self):
        if self.blackboard.get("return_planned"):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class IsMissionComplete(py_trees.behaviour.Behaviour):
    """Check if the entire mission is complete."""
    
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="mission_complete", access=common.Access.READ)
        
    def update(self):
        if self.blackboard.get("mission_complete"):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class LogMessage(py_trees.behaviour.Behaviour):
    """Log a message and succeed."""
    
    def __init__(self, name: str, message: str):
        super().__init__(name)
        self.message = message
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="logger", access=common.Access.READ)
        
    def update(self):
        logger = get_bt_logger_from_blackboard(self.blackboard)
        logger.info(f"{self.message}")
        return py_trees.common.Status.SUCCESS


# ============================================================================
# BEHAVIOR TREE CONSTRUCTION
# ============================================================================

def create_courier_behavior_tree():
    """
    Create the courier robot behavior tree with Nav2 integration.
    
    Tree Structure:
    
    Root (Selector)
    ├── Mission Complete Check
    └── Main Mission (Sequence)
        ├── Battery Check
        ├── Go To Pickup (Selector)
        │   ├── Already at pickup (object collected)
        │   └── Navigate to pickup (Sequence)
        │       ├── Get next waypoint
        │       └── Navigate to cell (Nav2)
        ├── Collect Object (Selector)
        │   ├── Already collected
        │   └── Collect
        ├── Plan Return (Selector)
        │   ├── Already planned
        │   └── Plan path home
        ├── Return Home (Sequence)
        │   ├── Get next waypoint
        │   └── Navigate to cell (Nav2)
        └── Deliver Object
    """
    
    # Initialize blackboard defaults and logger so behaviors have sensible keys
    import logging
    bb = py_trees.blackboard.Blackboard()
    # Setup a simple logger for the behavior tree and put it on the blackboard
    logger = logging.getLogger('courier_bt')
    logger.setLevel(logging.INFO)
    if not logger.handlers:
        ch = logging.StreamHandler()
        ch.setFormatter(logging.Formatter('%(asctime)s [%(name)s] %(levelname)s: %(message)s'))
        logger.addHandler(ch)
    bb.set('logger', logger)

    # Default blackboard state (can be overridden by mission controller)
    if bb.get('object_collected') is None:
        bb.set('object_collected', False)
    if bb.get('return_planned') is None:
        bb.set('return_planned', False)
    if bb.get('path_queue') is None:
        bb.set('path_queue', [])
    if bb.get('start_cell') is None:
        bb.set('start_cell', (0, 0))
    if bb.get('cell_size') is None:
        bb.set('cell_size', 1.0)
    if bb.get('obstacles') is None:
        # Default obstacles match world_spawner / mission controller
        bb.set('obstacles', [(1, 1), (1, 2), (3, 1), (3, 3)])
    if bb.get('battery_level') is None:
        bb.set('battery_level', 100.0)
    if bb.get('mission_complete') is None:
        bb.set('mission_complete', False)

    # Root selector - either mission is complete, or we work on it
    root = py_trees.composites.Selector(name="Root", memory=False)
    
    # Check if mission is already complete
    mission_done = IsMissionComplete(name="Mission Done?")
    
    # Main mission sequence
    main_mission = py_trees.composites.Sequence(name="Main Mission", memory=True)
    
    # 1. Battery check
    battery_check = CheckBattery(name="Check Battery", min_level=10.0)
    
    # 2. Navigate to pickup location
    go_to_pickup = py_trees.composites.Selector(name="Go To Pickup", memory=False)
    already_at_pickup = IsObjectCollected(name="Already Collected?")
    
    navigate_to_pickup = py_trees.composites.Sequence(name="Navigate To Pickup", memory=True)
    path_not_complete = py_trees.decorators.FailureIsSuccess(
        name="Path Check",
        child=IsPathComplete(name="Pickup Path Done?")
    )
    get_waypoint_pickup = GetNextWaypoint(name="Get Pickup Waypoint")
    nav_to_cell_pickup = NavigateToCell(name="Nav2 To Pickup", cell_key="current_target")
    
    navigate_to_pickup.add_children([get_waypoint_pickup, nav_to_cell_pickup])
    
    # Repeat navigation until path is complete
    repeat_pickup_nav = py_trees.decorators.Repeat(
        name="Repeat Pickup Nav",
        child=navigate_to_pickup,
        num_success=-1  # Repeat until failure (path empty)
    )
    
    go_to_pickup.add_children([already_at_pickup, repeat_pickup_nav])
    
    # 3. Collect object
    collect_selector = py_trees.composites.Selector(name="Collect Selector", memory=False)
    already_collected = IsObjectCollected(name="Object Collected?")
    collect_object = CollectObject(name="Collect Object")
    collect_selector.add_children([already_collected, collect_object])
    
    # 4. Plan return path
    plan_return_selector = py_trees.composites.Selector(name="Plan Return", memory=False)
    return_already_planned = IsReturnPlanned(name="Return Planned?")
    plan_return = PlanReturnPath(name="Plan Return Path")
    plan_return_selector.add_children([return_already_planned, plan_return])
    
    # 5. Navigate home
    navigate_home = py_trees.composites.Sequence(name="Navigate Home", memory=True)
    get_waypoint_home = GetNextWaypoint(name="Get Home Waypoint")
    nav_to_cell_home = NavigateToCell(name="Nav2 To Home", cell_key="current_target")
    navigate_home.add_children([get_waypoint_home, nav_to_cell_home])
    
    # 6. Deliver object
    deliver = DeliverObject(name="Deliver Object")
    
    # Assemble main mission
    main_mission.add_children([
        battery_check,
        go_to_pickup,
        collect_selector,
        plan_return_selector,
        navigate_home,
        deliver
    ])
    
    # Assemble root
    root.add_children([mission_done, main_mission])
    
    return root


def print_tree(tree):
    """Print the behavior tree structure."""
    print(py_trees.display.unicode_tree(root=tree))


