import time
import math
from bt.imu_sensor import IMUSensor
 
class RobotState:
    def __init__(self):
        # Tempi calibrati per velocità ECO su superficie liscia (60% lineare, 75% rotazioni)
        self.base_rotation_90_time = 6.4  # 90° a 75% su superficie liscia
        self.base_cell_move_time = 3.8     # 60cm a 60%
        
        self.reference_voltage = 7.4
        self.current_voltage = 7.4
        self.min_voltage = 6.4
 
        # nuovi limiti "anti-escalation" (previene compensazioni eccessive)
        self.max_compensation = 1.25   # massimo +25% tempo
        self.min_valid_voltage = 6.0   # sotto: lettura/sag troppo basso, evita divisioni aggressive
        
        self.compensation_factor = 1.0
        self.rotation_90_time = self.base_rotation_90_time
        self.cell_move_time = self.base_cell_move_time
        
        # --- Navigation State ---
        self.cell_size = 0.60
        self.grid_size = 4
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_heading = 'N'
        self.robot_yaw = math.pi / 2
        
        self.obstacles = set()
        self.start_cell = (0, 0)
        self.goal_cell = (3, 3)
        
        self.front_distance = 100.0
        self.obstacle_threshold = 20.0
        
        # --- IMU ---
        self.imu = IMUSensor()
        if self.imu.is_available():
            print("🧭 IMU abilitato - calibrazione consigliata!")
        else:
            print("⚠️ IMU non disponibile - navigazione senza correzione heading")
 
    def world_to_cell(self, x, y):
        """Converte coordinate mondo (m) in cella (row, col)."""
        col = int(round(x / self.cell_size))
        row = int(round(y / self.cell_size))
        return row, col
 
    def cell_to_world(self, row, col):
        """Converte cella (row, col) in coordinate mondo (m)."""
        x = col * self.cell_size
        y = row * self.cell_size
        return x, y
 
    def update_sensors(self, rover_api=None):
        """Aggiorna sensori (distanza, IMU, ecc)."""
        # Aggiorna heading IMU
        if self.imu.is_available():
            self.imu.update_heading()
        
        # Aggiorna distanza ultrasuoni
        if rover_api:
            try:
                dist = rover_api.getDistance()
                if dist is not None:
                    self.front_distance = dist
            except Exception as e:
                print(f"❌ Errore lettura sensori: {e}")
        else:
            print("⚠️ update_sensors chiamato senza rover_api!")
 
    def update_battery_voltage(self, rover_api):
        """Lettura tensione disabilitata - voltage divider non presente."""
        # Pin A0 non collegato alla batteria → compensazione sempre 1.0
        # NON stampiamo warning (verrebbe chiamato ad ogni movimento)
        self.compensation_factor = 1.0
        self.rotation_90_time = self.base_rotation_90_time
        self.cell_move_time = self.base_cell_move_time
 
    def print_grid(self, show_path=None):
        """
        Stampa griglia ASCII con posizione robot, ostacoli, goal.
        
        Args:
            show_path: Lista opzionale di celle (row,col) da evidenziare come percorso
        """
        current_row, current_col = self.world_to_cell(self.robot_x, self.robot_y)
        
        # Simboli heading
        heading_symbols = {'N': '↑', 'S': '↓', 'E': '→', 'W': '←'}
        robot_symbol = heading_symbols.get(self.robot_heading, '●')
        
        print("\n" + "═" * (self.grid_size * 4 + 1))
        print("📍 MAPPA GRIGLIA")
        print("═" * (self.grid_size * 4 + 1))
        
        # Legenda
        print(f"  {robot_symbol} = Robot (heading: {self.robot_heading})")
        print(f"  🎯 = Goal {self.goal_cell}")
        print(f"  🚧 = Ostacolo")
        print(f"  · = Percorso pianificato")
        print(f"  □ = Cella vuota")
        print()
        
        # Header con numeri colonna
        print("    ", end="")
        for col in range(self.grid_size):
            print(f" {col}  ", end="")
        print()
        
        # Stampa griglia (dall'alto verso il basso)
        for row in range(self.grid_size - 1, -1, -1):  # Inverti per avere Nord in alto
            print(f"  {row} ", end="")
            
            for col in range(self.grid_size):
                cell = (row, col)
                
                # Determina simbolo cella
                if (row, col) == (current_row, current_col):
                    symbol = robot_symbol
                elif cell == self.goal_cell:
                    symbol = "🎯"
                elif cell in self.obstacles:
                    symbol = "🚧"
                elif show_path and cell in show_path:
                    symbol = "·"
                else:
                    symbol = "□"
                
                print(f"[{symbol}]", end="")
            
            print(f" {row}")  # Numero riga anche a destra
        
        # Footer con numeri colonna
        print("    ", end="")
        for col in range(self.grid_size):
            print(f" {col}  ", end="")
        print()
        print("═" * (self.grid_size * 4 + 1))
        print(f"Robot: ({current_row}, {current_col}) | Goal: {self.goal_cell}")
        print(f"Coordinate mondo: X={self.robot_x:.2f}m, Y={self.robot_y:.2f}m")
        print("═" * (self.grid_size * 4 + 1) + "\n")
 
# Instantiate the global state object
robot_state = RobotState()
 
