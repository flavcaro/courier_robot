"""
Stato Robot
Traccia solo: distanza percorsa verso nord + offset laterale
 
Aggiornato:
- Boost extra sul cingolo che va in RETRO durante le rotazioni (attrito alto).
"""
 
class SimpleRobotState:
    def __init__(self):
        # === MISSIONE ===
        self.target_distance = 4  # metri da percorrere verso nord
        self.distance_traveled = 0.0  # metri già percorsi
        
        self.micro_turn_degrees = 30
        self.micro_turn_settle = 0.12
        self.scan_threshold = 50
 
 
        # === POSIZIONE ===
        self.lateral_offset = 0.0  # offset est(+) / ovest(-) dalla linea centrale
        self.heading = 'N'  # N/E/S/W
 
        # === CALIBRAZIONI MOVIMENTO ===
        self.meters_per_second_forward = 0.20  # ~20cm/s a 60%
        self.meters_per_second_lateral = 0.20  # ~20cm/s laterale
        self.rotation_90_time = 2.1  # secondi per 90° (calibrato con test 360°)
        
        # Tempi separati per sinistra/destra (per compensare asimmetrie)
        self.rotation_90_time_left = 1.8   # None = usa rotation_90_time
        self.rotation_90_time_right = 2.3  # None = usa rotation_90_time
 
        # Compensazione motori per movimento dritto
        self.left_factor = 1.0
        self.right_factor = 0.75  # default, sarà sovrascritto da calibrazione
 
        # === COMPENSAZIONI ROTAZIONE (base) ===
        # Bilanciamento tra cingolo sinistro e destro (es. sinistro più debole)
        self.rotation_left_factor = 1.5
        self.rotation_right_factor = 1.0
 
        # === BOOST RETRO DURANTE ROTAZIONI ===
        # Il cingolo che va in retro spesso si "pianta" per attrito -> boost extra
        self.rotation_reverse_boost = 1.30   # prova 1.25–1.45
        self.rotation_forward_boost = 1.00   # di solito 1.00
 
        # === PARAMETRI AGGIRAMENTO ===
        self.lateral_step = 0.5
        self.forward_clearance = 2.5
 
        # === SENSORI ===
        self.obstacle_threshold = 20.0  # cm
        self.lateral_clearance_threshold = 35.0  # cm
        self.bypass_verification_threshold = 35.0  # cm
        self.last_lidar_distance = 400.0
 
    # =========================
    # ROTAZIONE: FATTORI UTILI
    # =========================
    def get_rotation_factors(self, turn_dir):
        """
        Restituisce (left_factor, right_factor) per una rotazione sul posto.
 
        turn_dir:
          - 'L' = ruota a sinistra  -> sinistro indietro, destro avanti
          - 'R' = ruota a destra    -> sinistro avanti, destro indietro
 
        Applica:
        - fattori base (rotation_left_factor/right)
        - boost extra al cingolo in retro (rotation_reverse_boost)
        """
        if turn_dir == 'L':
            left = self.rotation_left_factor * self.rotation_reverse_boost    # sinistro in retro
            right = self.rotation_right_factor * self.rotation_forward_boost  # destro avanti
        elif turn_dir == 'R':
            left = self.rotation_left_factor * self.rotation_forward_boost    # sinistro avanti
            right = self.rotation_right_factor * self.rotation_reverse_boost  # destro in retro
        else:
            raise ValueError("turn_dir deve essere 'L' o 'R'")
 
        return left, right
 
    # =========================
    # STATO / UTILITIES
    # =========================
    def is_target_reached(self):
        return self.distance_traveled >= self.target_distance
 
    def is_on_center_line(self):
        return abs(self.lateral_offset) < 0.10
 
    def add_forward_distance(self, meters):
        if self.heading == 'N':
            self.distance_traveled += meters
            print(f"📊 Distanza percorsa: {self.distance_traveled:.2f}m / {self.target_distance:.2f}m")
        else:
            print(f"⚠️  add_forward_distance chiamato con heading {self.heading} (non Nord)")
 
    def add_lateral_offset(self, meters, direction):
        if direction == 'E':
            self.lateral_offset += meters
            print(f"📊 Offset laterale: {self.lateral_offset:+.2f}m (Est)")
        elif direction == 'W':
            self.lateral_offset -= meters
            print(f"📊 Offset laterale: {self.lateral_offset:+.2f}m (Ovest)")
 
    def print_status(self):
        print("\n" + "="*50)
        print("📍 STATO ROBOT")
        print("="*50)
        print(f"Distanza percorsa: {self.distance_traveled:.2f}m / {self.target_distance:.2f}m")
        print(f"Offset laterale: {self.lateral_offset:+.2f}m {'(sulla linea ✓)' if self.is_on_center_line() else '(fuori linea)'}")
        print(f"Heading: {self.heading}")
        print(f"Lidar: {self.last_lidar_distance:.1f}cm")
        print("="*50 + "\n")
 
 
# Istanza globale
simple_state = SimpleRobotState()
 
 