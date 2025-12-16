import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
import math
import py_trees
from courier_nav.courier_behavior_tree import create_courier_behavior_tree

def euler_from_quaternion(x, y, z, w):
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return yaw

class CourierController(Node):
    def __init__(self):
        super().__init__('courier_controller')
        
        # --- Configurazione ---
        self.cell_size = 1.0 
        # 0=Libero, 1=Ostacolo
        self.grid_map = [     
            [0, 0, 0, 0, 0],
            [0, 1, 1, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 1, 0, 1, 0],
            [0, 0, 0, 0, 0]
        ]
        
        self.dist_tolerance = 0.15  # 15 cm
        self.angle_tolerance = 0.15  # ~8.6° - più permissivo

        # --- Comunicazione ROS ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Publisher per la visualizzazione (RViz)
        self.marker_pub = self.create_publisher(MarkerArray, '/grid_markers', 10)
        
        self.current_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.current_target = None
        
        # Initialize robot position and orientation
        self.robot_position = (0.0, 0.0)
        self.robot_yaw = 0.0
        
        # --- Calcolo Percorso ---
        self.start_cell = (0, 0)
        self.goal_cell = (4, 2)
        
        # Flag per sapere se il robot è stato inizializzato
        self.robot_initialized = False
        
        self.path_queue = self.bfs_path_planner(self.start_cell, self.goal_cell)
        
        # Log del percorso calcolato
        self.get_logger().info(f'=== PERCORSO CALCOLATO ===')
        self.get_logger().info(f'Start: {self.start_cell}')
        self.get_logger().info(f'Goal: {self.goal_cell}')
        self.get_logger().info(f'Path (celle): {self.path_queue}')
        
        # Salviamo il percorso COMPLETO originale per il ritorno (PRIMA di rimuovere lo start)
        self.original_complete_path = list(self.path_queue)
        
        # Rimuovi lo start dal percorso (il robot è già lì)
        if self.path_queue and self.path_queue[0] == self.start_cell:
            self.path_queue.pop(0)
            self.get_logger().info(f'Path dopo rimozione start: {self.path_queue}')
        
        # Salviamo una copia del percorso completo per disegnarlo
        self.full_path = list(self.path_queue)
        
        # --- Timer ---
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Pubblichiamo i marker una volta all'avvio (o periodicamente)
        self.create_timer(1.0, self.publish_markers)
        
        # --- Behavior Tree Setup ---
        self.behavior_tree = create_courier_behavior_tree(cell_size=self.cell_size)
        self.behavior_tree.setup_with_descendants()
        
        # Initialize blackboard
        self.blackboard = py_trees.blackboard.Client(name="CourierController")
        self.blackboard.register_key(
            key="battery_level",
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="current_target",
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="path_queue",
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="angle_diff",
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="distance_to_target",
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="cmd_vel_publisher",
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="logger",
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="object_collected",
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="returned_home",
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="grid_map",
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="start_cell",
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="goal_cell",
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="original_path",
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="return_planned",
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="original_complete_path",
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="robot_position",
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="robot_yaw",
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="aligned_at_turn",
            access=py_trees.common.Access.WRITE
        )
        
        # Set initial blackboard values
        self.blackboard.set("battery_level", 100.0)  # Start with full battery
        self.blackboard.set("current_target", None)
        self.blackboard.set("path_queue", list(self.path_queue))
        self.blackboard.set("angle_diff", 0.0)
        self.blackboard.set("distance_to_target", 0.0)
        self.blackboard.set("cmd_vel_publisher", self.publish_cmd_vel)
        self.blackboard.set("logger", self.get_logger())
        self.blackboard.set("object_collected", False)  # Object not collected yet
        self.blackboard.set("return_planned", False)  # Return path not yet planned
        self.blackboard.set("grid_map", self.grid_map)
        self.blackboard.set("start_cell", self.start_cell)
        self.blackboard.set("goal_cell", self.goal_cell)
        self.blackboard.set("original_complete_path", list(self.original_complete_path))  # For return path
        self.blackboard.set("robot_position", self.robot_position)  # Current robot position
        self.blackboard.set("robot_yaw", self.robot_yaw)  # Current robot orientation
        self.blackboard.set("aligned_at_turn", False)  # Not yet aligned at turn point 

    def publish_markers(self):
        """Disegna la griglia, start, goal e ostacoli in RViz"""
        marker_array = MarkerArray()
        rows = len(self.grid_map)
        cols = len(self.grid_map[0])
        
        marker_id = 0
        
        for r in range(rows):
            for c in range(cols):
                # Crea un cubo per ogni cella importante
                marker = Marker()
                marker.header.frame_id = "odom" # Usa "odom" come riferimento fisso
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "grid"
                marker.id = marker_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                
                # Posizione: Centro della cella
                marker.pose.position.x = (r + 0.5) * self.cell_size
                marker.pose.position.y = (c + 0.5) * self.cell_size
                marker.pose.position.z = 0.05 # Leggermente sollevato
                
                # Dimensione: un po' più piccolo della cella per vedere i bordi
                marker.scale.x = self.cell_size * 0.9
                marker.scale.y = self.cell_size * 0.9
                marker.scale.z = 0.1
                
                marker.color.a = 0.5 # Trasparenza (0.0 inv - 1.0 solido)
                
                # COLORE IN BASE AL TIPO
                if self.grid_map[r][c] == 1: # OSTACOLO (ROSSO)
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                    marker_array.markers.append(marker)
                
                elif (r, c) == self.start_cell: # START (VERDE)
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                    marker.scale.z = 0.2 # Più alto
                    marker_array.markers.append(marker)
                    
                elif (r, c) == self.goal_cell: # GOAL (BLU)
                    marker.color.r = 0.0
                    marker.color.g = 0.0
                    marker.color.b = 1.0
                    marker.scale.z = 0.2
                    marker_array.markers.append(marker)

                # Se fa parte del percorso, coloralo di GIALLO (piccolo)
                if (r, c) in self.full_path:
                    path_marker = Marker()
                    # Copia le proprietà base...
                    path_marker.header = marker.header
                    path_marker.ns = "path"
                    path_marker.id = marker_id + 1000 # ID diverso
                    path_marker.type = Marker.SPHERE
                    path_marker.action = Marker.ADD
                    path_marker.pose = marker.pose
                    path_marker.scale.x = 0.3
                    path_marker.scale.y = 0.3
                    path_marker.scale.z = 0.3
                    path_marker.color.r = 1.0
                    path_marker.color.g = 1.0
                    path_marker.color.b = 0.0
                    path_marker.color.a = 1.0
                    marker_array.markers.append(path_marker)

                marker_id += 1
                
        self.marker_pub.publish(marker_array)

    def odom_callback(self, msg):
        self.current_pose['x'] = msg.pose.pose.position.x
        self.current_pose['y'] = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        self.current_pose['theta'] = euler_from_quaternion(
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        )
    
    def publish_cmd_vel(self, msg):
        """Wrapper method to publish cmd_vel (used by behavior tree)."""
        self.cmd_vel_pub.publish(msg)
    
    def update_blackboard_state(self):
        """Update blackboard with current state information."""
        # Get current target from blackboard
        current_target = self.blackboard.get("current_target")
        
        if current_target is not None:
            # Calculate distance and angle to target
            tx, ty = current_target
            dx = tx - self.current_pose['x']
            dy = ty - self.current_pose['y']
            distance = math.sqrt(dx**2 + dy**2)
            target_angle = math.atan2(dy, dx)
            angle_diff = target_angle - self.current_pose['theta']
            
            # Normalize angle difference
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            # Update blackboard
            self.blackboard.set("distance_to_target", distance)
            self.blackboard.set("angle_diff", angle_diff)
        else:
            self.blackboard.set("distance_to_target", 0.0)
            self.blackboard.set("angle_diff", 0.0)

    def control_loop(self):
        if not self.robot_initialized:
            if (self.current_pose['x'] != 0.0 or self.current_pose['y'] != 0.0):
                self.robot_initialized = True
                self.get_logger().info(f'Robot inizializzato a: pos=({self.current_pose["x"]:.2f}, {self.current_pose["y"]:.2f}) theta={self.current_pose["theta"]:.2f} rad')
            return
        
        # Update blackboard with current state
        self.update_blackboard_state()
        
        # Tick the behavior tree
        self.behavior_tree.tick_once()
        
        # Optional: Log tree status periodically (for debugging)
        if not hasattr(self, '_tree_log_counter'):
            self._tree_log_counter = 0
        self._tree_log_counter += 1
        if self._tree_log_counter % 100 == 0:  # Every 10 seconds
            battery = self.blackboard.get("battery_level")
            self.get_logger().info(f'🔋 Battery: {battery:.1f}%')
            # Uncomment to see tree structure:
            # self.get_logger().info(f'\n{py_trees.display.unicode_tree(self.behavior_tree, show_status=True)}')

    def bfs_path_planner(self, start_cell, goal_cell):
        rows = len(self.grid_map)
        cols = len(self.grid_map[0])
        queue = [start_cell]
        came_from = {start_cell: None}
        found = False
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)] 
        while queue:
            current = queue.pop(0)
            if current == goal_cell:
                found = True
                break
            for d in directions:
                nr, nc = current[0] + d[0], current[1] + d[1]
                if 0 <= nr < rows and 0 <= nc < cols:
                    if self.grid_map[nr][nc] == 0 and (nr, nc) not in came_from:
                        queue.append((nr, nc))
                        came_from[(nr, nc)] = current
        if not found: return []
        path = []
        curr = goal_cell
        while curr is not None:
            path.append(curr)
            curr = came_from[curr]
        path.reverse()
        return path

def main(args=None):
    rclpy.init(args=args)
    node = CourierController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
