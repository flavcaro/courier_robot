import time
import math

class RobotState:
    def __init__(self):
        # Default base values (added to prevent AttributeError if missing)
        self.base_rotation_90_time = 6.5
        self.base_cell_move_time = 2.0
        
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
        """Legge tensione batteria e aggiorna fattore di compensazione (safe + media)."""
        try:
            # media di 3 letture per evitare sag istantaneo
            samples = []
            for _ in range(3):
                v = rover_api.getBatteryVoltage()
                samples.append(v)
                time.sleep(0.05)

            v_avg = sum(samples) / len(samples)
            self.current_voltage = v_avg

            if self.current_voltage < self.min_valid_voltage:
                # troppo basso o lettura sporca: non impazzire con la compensazione
                print(f"⚠️ Lettura tensione molto bassa ({self.current_voltage:.2f}V). Uso compensazione=1.0")
                self.compensation_factor = 1.0
            else:
                comp = self.reference_voltage / self.current_voltage
                # clamp massimo
                if comp > self.max_compensation:
                    comp = self.max_compensation
                if comp < 1.0:
                    comp = 1.0  # se batteria più alta del reference, non serve ridurre tempi

                self.compensation_factor = comp

            self.rotation_90_time = self.base_rotation_90_time * self.compensation_factor
            self.cell_move_time = self.base_cell_move_time * self.compensation_factor

            print(f"🔋 Batteria: {self.current_voltage:.2f}V | compensazione: {self.compensation_factor:.3f}")
            print(f"   rot90: {self.rotation_90_time:.2f}s | cella: {self.cell_move_time:.2f}s")

            if self.current_voltage < self.min_voltage:
                print(f"⚠️ Batteria bassa ({self.current_voltage:.2f}V)! Ricaricare presto.")

        except Exception as e:
            print(f"❌ Errore lettura tensione batteria: {e}")
            self.compensation_factor = 1.0
            self.rotation_90_time = self.base_rotation_90_time
            self.cell_move_time = self.base_cell_move_time

# Instantiate the global state object
robot_state = RobotState()
