"""Behaviors per la navigazione - Versione Standalone."""

import math
import time
import py_trees
from py_trees import common


class RotateToTarget(py_trees.behaviour.Behaviour):
    """Ruota il robot verso l'angolo target."""
    
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="node", access=common.Access.READ)
        self.blackboard.register_key(key="target_yaw", access=common.Access.READ)
        self.rotation_start_time = None
        
    def initialise(self):
        """Chiamato quando il behavior inizia."""
        self.rotation_start_time = time.time()
    
    def update(self):
        """Esegue la logica di rotazione."""
        node = self.blackboard.get("node")
        target_yaw = self.blackboard.get("target_yaw")
        
        if target_yaw is None:
            return py_trees.common.Status.FAILURE
        
        # Calcola errore angolare
        angle_error = node.normalize_angle(target_yaw - node.robot_yaw)
        
        # Controlla se raggiunto
        if abs(angle_error) < node.angle_tolerance:
            node.stop_robot()
            node.log_info(f'✓ Rotazione completata! Yaw={math.degrees(node.robot_yaw):.1f}°')
            
            # Controlla ostacolo davanti
            if node.front_distance < node.obstacle_threshold:
                node.log_warn(f'⚠️  Ostacolo rilevato a {node.front_distance:.2f}m!')
                return py_trees.common.Status.FAILURE
            
            return py_trees.common.Status.SUCCESS
        
        # Ruota verso target
        direction = "Left" if angle_error > 0 else "Right"
        speed = min(node.rotation_speed, abs(angle_error) * 0.5)  # Proporzionale
        
        node.rover.moveTo(direction, speed)
        
        # Aggiorna odometria stimata (approssimazione)
        # Assumiamo rotazione di ~30°/s a velocità 0.3
        dt = 0.1  # Tick rate
        angular_velocity = speed * 1.0  # rad/s approssimato
        if direction == "Right":
            angular_velocity = -angular_velocity
        node.robot_yaw += angular_velocity * dt
        node.robot_yaw = node.normalize_angle(node.robot_yaw)
        
        return py_trees.common.Status.RUNNING


class MoveToTarget(py_trees.behaviour.Behaviour):
    """Muove il robot dritto verso la posizione target."""
    
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="node", access=common.Access.READ)
        self.blackboard.register_key(key="target_world_x", access=common.Access.READ)
        self.blackboard.register_key(key="target_world_y", access=common.Access.READ)
        self.blackboard.register_key(key="target_yaw", access=common.Access.READ)
        self.blackboard.register_key(key="current_target", access=common.Access.READ)
        self.movement_start_time = None
        self.last_log_time = 0
        
    def initialise(self):
        """Inizia il movimento."""
        self.movement_start_time = time.time()
        self.last_log_time = time.time()
    
    def update(self):
        """Esegue la logica di movimento."""
        node = self.blackboard.get("node")
        target_x = self.blackboard.get("target_world_x")
        target_y = self.blackboard.get("target_world_y")
        target_yaw = self.blackboard.get("target_yaw")
        current_target = self.blackboard.get("current_target")
        
        # Controlla ostacolo
        if node.front_distance < node.obstacle_threshold:
            node.stop_robot()
            node.log_warn(f'⚠️  Ostacolo a {node.front_distance:.2f}m!')
            return py_trees.common.Status.FAILURE
        
        # Calcola distanza
        dx = target_x - node.robot_x
        dy = target_y - node.robot_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Log periodico
        current_time = time.time()
        if current_time - self.last_log_time > 2.0:
            node.log_info(f'Movimento: distanza={distance:.2f}m, ultrasuono={node.front_distance:.2f}m')
            self.last_log_time = current_time
        
        # Controlla se raggiunto
        if distance < node.position_tolerance:
            node.stop_robot()
            
            # Decrementa batteria
            node.battery_level -= 10.0
            if node.battery_level < 0:
                node.battery_level = 0.0
            
            node.log_info(f'✓ Raggiunta cella {current_target}! 🔋 Batteria: {node.battery_level:.0f}%')
            return py_trees.common.Status.SUCCESS
        
        # Controlla drift angolare
        if target_yaw is not None:
            angle_error = node.normalize_angle(target_yaw - node.robot_yaw)
            if abs(angle_error) > 0.20:  # ~11 gradi
                node.log_warn(f'Drift rilevato: {math.degrees(angle_error):.1f}°')
                node.stop_robot()
                return py_trees.common.Status.FAILURE
        
        # Muovi in avanti
        speed = node.linear_speed
        if distance < 0.3:  # Rallenta vicino al target
            speed *= 0.6
        
        node.rover.moveTo("Forward", speed)
        
        # Aggiorna odometria stimata
        dt = 0.1
        linear_velocity = speed * 0.3  # m/s approssimato
        node.robot_x += linear_velocity * math.cos(node.robot_yaw) * dt
        node.robot_y += linear_velocity * math.sin(node.robot_yaw) * dt
        
        return py_trees.common.Status.RUNNING


class GetNextWaypoint(py_trees.behaviour.Behaviour):
    """Preleva il prossimo waypoint dalla coda del percorso."""
    
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
        """Preleva il prossimo waypoint e calcola direzione cardinale."""
        node = self.blackboard.get("node")
        path_queue = self.blackboard.get("path_queue")
        
        if not path_queue:
            return py_trees.common.Status.FAILURE
        
        # Preleva prossimo waypoint
        current_target = path_queue.popleft()
        self.blackboard.set("path_queue", path_queue)
        self.blackboard.set("current_target", current_target)
        
        row, col = current_target
        target_x, target_y = node.cell_to_world(row, col)
        self.blackboard.set("target_world_x", target_x)
        self.blackboard.set("target_world_y", target_y)
        
        # Calcola target yaw basato su delta griglia
        current_cell = node.world_to_cell(node.robot_x, node.robot_y)
        crow, ccol = current_cell
        drow = row - crow
        dcol = col - ccol
        
        # Determina direzione cardinale
        if dcol != 0:
            target_yaw = 0.0 if dcol > 0 else math.pi  # Est/Ovest
        elif drow != 0:
            target_yaw = math.pi / 2 if drow > 0 else -math.pi / 2  # Nord/Sud
        else:
            # Fallback
            dx = target_x - node.robot_x
            dy = target_y - node.robot_y
            target_yaw = math.atan2(dy, dx)
        
        target_yaw = node.normalize_angle(target_yaw)
        self.blackboard.set("target_yaw", target_yaw)
        
        node.log_info(f'🎯 Target: Cella{current_target} = ({target_x:.2f}, {target_y:.2f}), Yaw={math.degrees(target_yaw):.0f}°')
        
        return py_trees.common.Status.SUCCESS


class CenterOnCell(py_trees.behaviour.Behaviour):
    """Centra il robot sulla cella corrente."""
    
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="node", access=common.Access.READ)
        self.blackboard.register_key(key="current_target", access=common.Access.READ)
        self.blackboard.register_key(key="target_world_x", access=common.Access.READ)
        self.blackboard.register_key(key="target_world_y", access=common.Access.READ)
        self.blackboard.register_key(key="path_queue", access=common.Access.READ)
        self.blackboard.register_key(key="target_yaw", access=common.Access.WRITE)
        self.start_time = None
    
    def initialise(self):
        self.start_time = time.time()
    
    def update(self):
        """Centra il robot sulla cella."""
        node = self.blackboard.get("node")
        target_x = self.blackboard.get("target_world_x")
        target_y = self.blackboard.get("target_world_y")
        current_target = self.blackboard.get("current_target")
        
        if target_x is None or target_y is None:
            return py_trees.common.Status.FAILURE
        
        # Calcola errore
        ex = target_x - node.robot_x
        ey = target_y - node.robot_y
        distance = math.hypot(ex, ey)
        
        center_tolerance = 0.08  # 8cm
        
        # Timeout
        elapsed = time.time() - self.start_time
        if elapsed > 5.0:
            node.stop_robot()
            node.log_warn(f'Timeout centratura cella {current_target} (err={distance:.3f}m)')
            return py_trees.common.Status.SUCCESS
        
        # Controlla se centrato
        if distance < center_tolerance:
            node.stop_robot()
            node.log_info(f'✓ Centrato su cella {current_target}')
            
            # Imposta yaw per prossima cella (se esiste)
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
            
            return py_trees.common.Status.SUCCESS
        
        # Movimento correttivo semplice
        # Per semplicità, su robot fisico accettiamo la posizione corrente
        # (il centering preciso richiederebbe encoder accurati)
        node.stop_robot()
        return py_trees.common.Status.SUCCESS
