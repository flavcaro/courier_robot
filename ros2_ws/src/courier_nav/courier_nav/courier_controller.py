"""
Mission Controller Node
Manages complete courier mission with Behavior Tree
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import py_trees
from courier_nav.courier_behavior_tree import create_mission_behavior_tree
from visualization_msgs.msg import Marker, MarkerArray


class MissionController(Node):
    """
    Mission Controller using Behavior Tree
    Executes full courier mission: Init → Navigate → Pickup → Return → Release
    """
    
    def __init__(self):
        super().__init__('mission_controller')
        
        # Publishers
        self.target_pub = self.create_publisher(PoseStamped, '/target_pose', 10)
        # 🆕 Publisher per visualizzare il percorso
        self.path_viz_pub = self.create_publisher(MarkerArray, '/path_markers', 10)
        
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
        self.blackboard.register_key(key="arm_controller", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="gripper_force", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="apriltag_pose", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="gripper_controller", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="path_queue", access=py_trees.common.Access.READ)
        
        # Grid configuration (from world_spawner)
        # 🔍 VERIFICA: La griglia deve corrispondere allo spawn in world_spawner.py
        self.grid_map = [
            [0, 0, 0, 0, 0],  # Row 0
            [0, 1, 1, 0, 0],  # Row 1
            [0, 0, 0, 0, 0],  # Row 2
            [0, 1, 0, 1, 0],  # Row 3
            [0, 0, 0, 0, 0]   # Row 4
        ]
        self.start_cell = (0, 0)  # Row 0, Col 0 → (X=0.5, Y=0.5)
        self.goal_cell = (4, 2)   # Row 4, Col 2 → (X=2.5, Y=4.5)
        self.cell_size = 1.0
        
        # Add grid configuration to blackboard
        self.blackboard.register_key(key="grid_map", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="start_cell", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="goal_cell", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="cell_size", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="current_pose", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="current_target", access=py_trees.common.Access.WRITE)
        
        # Set initial blackboard values
        self.blackboard.set("system_ready", True)
        self.blackboard.set("target_publisher", self.publish_target)
        self.blackboard.set("logger", self.get_logger())
        self.blackboard.set("grid_map", self.grid_map)
        self.blackboard.set("start_cell", self.start_cell)
        self.blackboard.set("goal_cell", self.goal_cell)
        self.blackboard.set("cell_size", self.cell_size)
        self.blackboard.set("arm_controller", None)  # Placeholder for future arm control
        self.blackboard.set("gripper_force", 0.0)  # Placeholder for future gripper sensor
        self.blackboard.set("distance_to_target", 999.0)
        self.blackboard.set("distance_to_base", 999.0)
        self.blackboard.set("apriltag_pose", None)  # Placeholder for AprilTag detection
        self.blackboard.set("gripper_controller", None)  # Placeholder for gripper control
        
        # Current target for distance calculation
        self.current_target = (0.5, 0.5)  # Initial position (START cell)
        self.blackboard.set("current_target", self.current_target)
        
        # Timer for BT ticks
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Mission Controller started')
    
    def odom_callback(self, msg):
        """Update current pose from odometry"""
        old_x = self.current_pose['x']
        old_y = self.current_pose['y']
        
        self.current_pose['x'] = msg.pose.pose.position.x
        self.current_pose['y'] = msg.pose.pose.position.y
        # TODO: Extract theta from quaternion
        
        # Log quando la posizione cambia significativamente
        if not hasattr(self, '_last_odom_log_time'):
            import time
            self._last_odom_log_time = time.time()
        
        import time
        movement = ((self.current_pose['x'] - old_x)**2 + (self.current_pose['y'] - old_y)**2)**0.5
        if movement > 0.1 or time.time() - self._last_odom_log_time > 5.0:  # Ogni 10cm o 5 secondi
            self.get_logger().info(
                f"📍 Odometria: ({self.current_pose['x']:.3f}, {self.current_pose['y']:.3f})"
            )
            self._last_odom_log_time = time.time()
        
        # Update distance to target in blackboard
        if hasattr(self, 'current_target'):
            dx = self.current_target[0] - self.current_pose['x']
            dy = self.current_target[1] - self.current_pose['y']
            distance = (dx**2 + dy**2)**0.5
            self.blackboard.set("distance_to_target", distance)
    
    def publish_target(self, target_x, target_y):
        """Publish target pose for PID controller"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = target_x
        msg.pose.position.y = target_y
        msg.pose.position.z = 0.0
        self.target_pub.publish(msg)
        
        # Update current target for distance calculation
        self.current_target = (target_x, target_y)
    
    def visualize_path(self, path):
        """Visualizza il percorso BFS in RViz"""
        marker_array = MarkerArray()
        
        for i, cell in enumerate(path):
            row, col = cell
            
            # Converti in coordinate mondo
            x = (col + 0.5) * self.cell_size
            y = (row + 0.5) * self.cell_size
            
            # Crea marker
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "bfs_path"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.1
            
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            
            # Colore: verde → giallo → rosso
            marker.color.r = float(i) / len(path)
            marker.color.g = 1.0 - float(i) / len(path)
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            marker_array.markers.append(marker)
        
        self.path_viz_pub.publish(marker_array)
        self.get_logger().info(f"📍 Visualizzato percorso con {len(path)} waypoint")
    
    def control_loop(self):
        """Tick behavior tree"""
        if not self.system_ready:
            # Wait for system initialization
            if self.current_pose['x'] != 0.0 or self.current_pose['y'] != 0.0:
                self.system_ready = True
                self.blackboard.set("system_ready", True)
                
                # 🆕 LOG per confermare
                self.get_logger().info(
                    f"✅ Sistema pronto! Posizione iniziale: "
                    f"({self.current_pose['x']:.2f}, {self.current_pose['y']:.2f})"
                )
            return
        
        # 🆕 Update blackboard with current pose SEMPRE
        self.blackboard.set("current_pose", self.current_pose)
        
        # Tick the behavior tree
        self.behavior_tree.tick_once()

class LoadGridBFS(py_trees.behaviour.Behaviour):
    """Load grid and compute BFS path"""
    def __init__(self, name="LoadGridBFS"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="path_queue", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="grid_map", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="start_cell", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="goal_cell", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="logger", access=py_trees.common.Access.READ)
    
    def update(self):
        logger = self.blackboard.get("logger")
        
        # Get grid configuration from blackboard
        grid_map = self.blackboard.get("grid_map")
        start_cell = self.blackboard.get("start_cell")
        goal_cell = self.blackboard.get("goal_cell")
        
        if logger:
            logger.info(f"{self.name}: 🗺️  Grid: {len(grid_map)}x{len(grid_map[0])}")
            logger.info(f"{self.name}: 🟢 Start: {start_cell}")
            logger.info(f"{self.name}: 🔵 Goal: {goal_cell}")
            # 🆕 Stampa griglia
            for r, row in enumerate(grid_map):
                row_str = ''.join(['🟥' if cell == 1 else '⬜' for cell in row])
                logger.info(f"{self.name}:    Row {r}: {row_str}")
        
        # Compute BFS path
        path = compute_bfs_path(grid_map, start_cell, goal_cell)
        
        if not path:
            if logger:
                logger.error(f"{self.name}: ❌ No path found from {start_cell} to {goal_cell}!")
            return Status.FAILURE
        
        self.blackboard.set("path_queue", path)
        
        if logger:
            logger.info(f"{self.name}: ✅ Path computed = {len(path)} waypoints:")
            for i, cell in enumerate(path):
                logger.info(f"{self.name}:    {i+1}. Cell{cell}")
        
        return Status.SUCCESS
def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()