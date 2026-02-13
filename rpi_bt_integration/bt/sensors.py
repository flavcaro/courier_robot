import time
import math

class RobotState:
    def __init__(self):
        # Default base values (calibrati per velocità ECO: 35% rotazioni, 40% lineari)
        self.base_rotation_90_time = 16.0  # ~18.5s teorico, abbassato per compensare inerzia
        self.base_cell_move_time = 5.0     # 2.0s * (100/40) = 5.0s per 60cm
        
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
        """Aggiorna sensori (distanza, ecc)."""
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

# Instantiate the global state object
robot_state = RobotState()
