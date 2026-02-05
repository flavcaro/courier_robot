"""
Controller principale per il robot courier - Versione Standalone.
Adattato da ROS2 per funzionare con rover_API.py su Raspberry Pi.
"""

import math
import time
from collections import deque
import py_trees
from py_trees import common

from rover_API import RoverApi
from behaviors.navigation import RotateToTarget, MoveToTarget, GetNextWaypoint, CenterOnCell
from behaviors.mission import CollectObject, DeliverObject, PlanReturnPath, PlanPath
from behaviors.obstacle import HandleObstacle
from behaviors.battery import CheckBattery, ChargeBattery


class RobotController:
    """Controller principale con Behavior Tree per navigazione autonoma."""
    
    def __init__(self, serial_port='/dev/ttyUSB0'):
        """
        Inizializza il controller del robot.
        
        Args:
            serial_port: Porta seriale per comunicazione con Arduino
        """
        print('='*60)
        print('🤖 COURIER ROBOT - STANDALONE CONTROLLER')
        print('='*60)
        
        # === Connessione Robot ===
        self.rover = RoverApi(port=serial_port)
        
        # === Configurazione Griglia ===
        self.cell_size = 0.6  # 60cm × 60cm (griglia test più grande)
        self.grid_size = 3    # Griglia 3×3 = 1.8m × 1.8m (TEST)
        self.obstacles = {(1, 1)}  # Ostacolo singolo in (1,1)
        
        # === Parametri Missione ===
        self.start_cell = (0, 0)  # Angolo sud-ovest
        self.goal_cell = (1, 2)   # Destinazione pickup (riga 1, colonna 2)
        
        # === Stato Robot (odometria stimata) ===
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        # === Parametri Controllo ===
        self.rotation_speed = 0.3   # Velocità rotazione (ridotta per robot fisico)
        self.linear_speed = 0.2     # Velocità lineare (ridotta per robot fisico)
        self.angle_tolerance = 0.15  # ~8.6 gradi (più tollerante)
        self.position_tolerance = 0.15  # 15cm (più tollerante)
        
        # === Sensori ===
        self.front_distance = 5.0  # Distanza ostacolo (da ultrasuono)
        self.obstacle_threshold = 0.40  # 40cm threshold
        
        # === Batteria (simulata) ===
        self.battery_level = 100.0
        
        # === Timing ===
        self.start_time = time.time()
        self.last_sensor_read = 0
        
        # === Behavior Tree ===
        self.tree = None
        self.blackboard = py_trees.blackboard.Client(name="CourierRobot")
        self._setup_blackboard()
        
        print(f'📍 Griglia: {self.grid_size}×{self.grid_size} celle da {self.cell_size}m')
        print(f'🎯 Missione: {self.start_cell} → {self.goal_cell} → {self.start_cell}')
        print(f'🔋 Batteria: {self.battery_level:.0f}%')
        print('='*60)
    
    def _setup_blackboard(self):
        """Inizializza la blackboard per il Behavior Tree."""
        self.blackboard.register_key(key="node", access=common.Access.WRITE)
        self.blackboard.register_key(key="path_queue", access=common.Access.WRITE)
        self.blackboard.register_key(key="current_target", access=common.Access.WRITE)
        self.blackboard.register_key(key="target_world_x", access=common.Access.WRITE)
        self.blackboard.register_key(key="target_world_y", access=common.Access.WRITE)
        self.blackboard.register_key(key="target_yaw", access=common.Access.WRITE)
        self.blackboard.register_key(key="object_collected", access=common.Access.WRITE)
        self.blackboard.register_key(key="returning_home", access=common.Access.WRITE)
        
        # Inizializza valori
        self.blackboard.set("node", self)
        self.blackboard.set("path_queue", deque())
        self.blackboard.set("object_collected", False)
        self.blackboard.set("returning_home", False)
    
    def create_behavior_tree(self):
        """
        Crea il Behavior Tree gerarchico per la missione.
        
        Struttura:
        Root (Sequence)
        ├── Plan Path
        ├── Navigate To Pickup (Loop)
        ├── Collect Object
        ├── Plan Return Path
        ├── Navigate To Home (Loop)
        └── Deliver Object
        """
        root = py_trees.composites.Sequence(name="Mission", memory=True)
        
        # FASE 0: Pianifica percorso iniziale
        plan_path = PlanPath(name="Plan Path")
        
        # FASE 1: Naviga verso pickup
        nav_to_pickup = py_trees.decorators.FailureIsSuccess(
            name="Nav To Pickup Complete",
            child=py_trees.decorators.Repeat(
                name="Navigate Pickup Loop",
                child=py_trees.decorators.Retry(
                    name="Try Cell",
                    child=self.create_navigate_one_cell(),
                    num_failures=3
                ),
                num_success=999
            )
        )
        
        # FASE 2: Raccogli oggetto
        collect = CollectObject(name="Collect Object")
        
        # FASE 3: Pianifica ritorno
        plan_return = PlanReturnPath(name="Plan Return Path")
        
        # FASE 4: Naviga verso home
        nav_to_home = py_trees.decorators.FailureIsSuccess(
            name="Nav To Home Complete",
            child=py_trees.decorators.Repeat(
                name="Navigate Home Loop",
                child=py_trees.decorators.Retry(
                    name="Try Cell",
                    child=self.create_navigate_one_cell(),
                    num_failures=3
                ),
                num_success=999
            )
        )
        
        # FASE 5: Consegna oggetto
        deliver = DeliverObject(name="Deliver Object")
        
        # Assembla albero
        root.add_children([
            plan_path,
            nav_to_pickup,
            collect,
            plan_return,
            nav_to_home,
            deliver
        ])
        
        return root
    
    def create_navigate_one_cell(self):
        """
        Crea sotto-albero per navigare una singola cella.
        
        Sequenza: Check Battery → Get Waypoint → Rotate → Move → Center
        """
        nav_with_battery = py_trees.composites.Sequence(name="Navigate One Cell", memory=True)
        
        # Gestione batteria
        battery_check = py_trees.composites.Selector(name="Battery Management", memory=False)
        check_battery = CheckBattery(name="Check Battery")
        charge_battery = ChargeBattery(name="Charge Battery")
        battery_check.add_children([check_battery, charge_battery])
        
        # Sequenza navigazione
        nav_sequence = py_trees.composites.Sequence(name="Navigate", memory=True)
        get_waypoint = GetNextWaypoint(name="Get Next Waypoint")
        rotate = RotateToTarget(name="Rotate To Target")
        move = MoveToTarget(name="Move To Target")
        center = CenterOnCell(name="Center On Cell")
        
        # Gestione ostacoli
        move_with_fallback = py_trees.composites.Selector(name="Move Or Handle", memory=False)
        move_with_fallback.add_children([move, HandleObstacle(name="Handle Obstacle")])
        
        nav_sequence.add_children([get_waypoint, rotate, move_with_fallback, center])
        nav_with_battery.add_children([battery_check, nav_sequence])
        
        return nav_with_battery
    
    # ========================================================================
    # UTILITY METHODS (compatibili con behaviors)
    # ========================================================================
    
    def get_time(self):
        """Restituisce il tempo corrente (compatibilità con behaviors)."""
        return time.time()
    
    def log_info(self, message):
        """Log messaggio informativo."""
        timestamp = time.time() - self.start_time
        print(f"[{timestamp:7.2f}s] ℹ️  {message}")
    
    def log_warn(self, message):
        """Log warning."""
        timestamp = time.time() - self.start_time
        print(f"[{timestamp:7.2f}s] ⚠️  {message}")
    
    def log_error(self, message):
        """Log errore."""
        timestamp = time.time() - self.start_time
        print(f"[{timestamp:7.2f}s] ❌ {message}")
    
    def cell_to_world(self, row, col):
        """Converte cella griglia → coordinate mondo."""
        world_x = (col + 0.5) * self.cell_size
        world_y = (row + 0.5) * self.cell_size
        return world_x, world_y
    
    def world_to_cell(self, world_x, world_y):
        """Converte coordinate mondo → cella griglia."""
        col = int(world_x / self.cell_size)
        row = int(world_y / self.cell_size)
        return row, col
    
    def bfs_path(self, start, goal):
        """
        Pathfinding BFS - solo 4 direzioni (no diagonali).
        
        Args:
            start: Tupla (row, col) cella iniziale
            goal: Tupla (row, col) cella obiettivo
            
        Returns:
            Lista di celle [(row, col), ...] escludendo start
        """
        if start == goal:
            return []
        
        directions = [(1, 0), (-1, 0), (0, -1), (0, 1)]
        queue = deque([(start, [start])])
        visited = {start}
        
        while queue:
            (row, col), path = queue.popleft()
            
            for drow, dcol in directions:
                nrow, ncol = row + drow, col + dcol
                next_cell = (nrow, ncol)
                
                # Controlla limiti griglia
                if not (0 <= nrow < self.grid_size and 0 <= ncol < self.grid_size):
                    continue
                
                # Controlla ostacoli e visitati
                if next_cell in self.obstacles or next_cell in visited:
                    continue
                
                new_path = path + [next_cell]
                
                if next_cell == goal:
                    return new_path[1:]  # Escludi start
                
                visited.add(next_cell)
                queue.append((next_cell, new_path))
        
        return []  # Nessun percorso trovato
    
    def normalize_angle(self, angle):
        """Normalizza angolo a [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def update_sensors(self):
        """Aggiorna letture sensori (ultrasuono)."""
        current_time = time.time()
        
        # Leggi sensore ogni 0.2 secondi
        if current_time - self.last_sensor_read > 0.2:
            distance_cm = self.rover.getUltrasonicSensor()
            self.front_distance = distance_cm / 100.0  # Converti cm → metri
            self.last_sensor_read = current_time
    
    def stop_robot(self):
        """Ferma il robot."""
        self.rover.stop()
    
    def start_mission(self):
        """Avvia la missione."""
        self.log_info("🚀 Avvio missione...")
        
        # Crea e inizializza Behavior Tree
        self.tree = self.create_behavior_tree()
        self.tree.setup_with_descendants()
        
        self.log_info("✅ Behavior Tree creato")
        print("\n" + py_trees.display.unicode_tree(root=self.tree, show_status=True))
        print()
    
    def run(self):
        """Loop principale - tick del Behavior Tree a ~10Hz."""
        if self.tree is None:
            self.start_mission()
        
        tick_rate = 10  # Hz
        tick_interval = 1.0 / tick_rate
        
        self.log_info("▶️  Inizio esecuzione missione")
        
        try:
            while True:
                loop_start = time.time()
                
                # Aggiorna sensori
                self.update_sensors()
                
                # Tick Behavior Tree
                self.tree.tick_once()
                
                # Controlla se missione completata
                if self.tree.status == py_trees.common.Status.SUCCESS:
                    self.log_info("✅ MISSIONE COMPLETATA!")
                    self.stop_robot()
                    break
                
                elif self.tree.status == py_trees.common.Status.FAILURE:
                    self.log_error("❌ Missione fallita!")
                    self.stop_robot()
                    break
                
                # Mantieni tick rate costante
                elapsed = time.time() - loop_start
                if elapsed < tick_interval:
                    time.sleep(tick_interval - elapsed)
        
        except KeyboardInterrupt:
            self.log_warn("⏸️  Missione interrotta dall'utente")
            self.stop_robot()
        
        finally:
            self.rover.close()
            self.log_info("🏁 Controller terminato")


if __name__ == "__main__":
    # Crea e avvia controller
    controller = RobotController(serial_port='/dev/ttyUSB0')
    controller.run()
