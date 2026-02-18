"""
Stato Robot per Navigazione Labirinto
Traccia posizione (x,y) su griglia, visited cells, path history
 
Aggiornato:
- Grid-based position tracking per maze navigation
- Wall-following algorithm support
- Integrazione IMU per rotazioni precise
"""
 
from bt.imu_sensor import IMUSensor
 
class SimpleRobotState:
    def __init__(self):
        # === MISSIONE ===
        self.target_distance = 4  # metri da percorrere (per maze: distanza dal punto iniziale)
        self.maze_step_size = 0.4  # metri per step nel labirinto
       
        self.micro_turn_degrees = 30
        self.micro_turn_settle = 0.12
        self.scan_threshold = 50
 
        # === POSIZIONE (MAZE) ===
        self.position_x = 0.0  # coordinata X (metri)
        self.position_y = 0.0  # coordinata Y (metri)
        self.heading = 'N'  # N/E/S/W
        self.total_distance = 0.0  # distanza totale percorsa
        
        # LEGACY (mantenuto per compatibilità)
        self.distance_traveled = 0.0  # distanza verso nord
        self.lateral_offset = 0.0  # offset est/ovest
 
        # === CALIBRAZIONI MOVIMENTO ===
        # ⚠️ CALIBRARE: Misura tempo per 1 metro e calcola velocità reale
        # Esempio: Se 1m richiede 30s → velocità = 1/30 = 0.033 m/s
        self.meters_per_second_forward = 0.033  # ~3.3cm/s (da calibrare!)
        self.meters_per_second_lateral = 0.033  # ~3.3cm/s laterale
        self.rotation_90_time = 2.1  # secondi per 90° (calibrato con test 360°)
       
        # Tempi separati per sinistra/destra (per compensare asimmetrie)
        self.rotation_90_time_left = 1.99   # Ridotto: faceva 95° invece di 90°
        self.rotation_90_time_right = 2.1   # Perfetto a 90°
 
        # Compensazione motori per movimento dritto
        # Cingolo sinistro più indietro → aumentiamo velocità per compensare attrito
        # Calibrato: 1.30=troppo poco (sinistra), 1.70=troppo (destra) → 1.50 ottimale
        self.left_factor = 1.34  # Valore intermedio calibrato
        self.right_factor = 1.00  # Base normale
 
        # === COMPENSAZIONI ROTAZIONE (base) ===
        # Bilanciamento tra cingolo sinistro e destro (es. sinistro più debole)
        self.rotation_left_factor = 1.5
        self.rotation_right_factor = 1.0
 
        # === BOOST RETRO DURANTE ROTAZIONI ===
        # Il cingolo che va in retro spesso si "pianta" per attrito -> boost extra
        self.rotation_reverse_boost = 1.30   # prova 1.25–1.45
        self.rotation_forward_boost = 1.00   # di solito 1.00
 
        # === PARAMETRI AGGIRAMENTO ===
        self.lateral_step = 0.5  # metri - spostamento laterale
        self.forward_clearance = 0.5  # ← AUMENTATO da 1.0m a 2.5m per superare ostacolo
 
        # === SENSORI ===
        self.obstacle_threshold = 20.0  # cm (margine sicurezza per variazioni sensore)
        self.lateral_clearance_threshold = 35.0  # cm
        self.bypass_verification_threshold = 35.0  # cm
        self.last_lidar_distance = 400.0
       
        # === IMU ===
        self.imu = IMUSensor()
        self.use_imu_rotation = False  # Abilitato dopo calibrazione
        self.use_imu_odometry = True  # Abilitato per odometria movimenti - accelerometro preciso!
        self.use_imu_heading_correction = True  # ✅ ABILITATO - Correzione automatica deriva durante movimento
        self.imu_heading_correction_gain = 0.15  # Guadagno correzione continua (0.1-0.3)
        self.imu_checkpoint_distance = 0.30  # Distanza tra checkpoint correzione attiva (metri)
        self.imu_checkpoint_threshold = 3.0  # Soglia deriva per attivare correzione (gradi)
        self.imu_reference_heading = None  # Heading di riferimento salvato in calibrazione
       
        if self.imu.is_available():
            print("🧭 IMU rilevato - calibrare per rotazioni e odometria precise")
        else:
            print("⚠️ IMU non disponibile - rotazioni e movimenti basati su tempo")
 
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
    # STATO / UTILITIES (MAZE)
    # =========================
    def get_distance_from_start(self):
        """Calcola distanza euclidea dal punto di partenza (0,0)"""
        import math
        return math.sqrt(self.position_x**2 + self.position_y**2)
    
    def is_target_reached(self):
        """Per maze: controlla se distanza dal punto iniziale >= target"""
        return self.get_distance_from_start() >= self.target_distance
    
    def update_position(self, meters):
        """Aggiorna posizione (x,y) in base a heading e distanza percorsa"""
        if self.heading == 'N':
            self.position_y += meters
        elif self.heading == 'S':
            self.position_y -= meters
        elif self.heading == 'E':
            self.position_x += meters
        elif self.heading == 'W':
            self.position_x -= meters
        
        self.total_distance += abs(meters)
        
        # LEGACY update per compatibilità
        self.distance_traveled = self.position_y
        self.lateral_offset = self.position_x
    

    
    def is_on_center_line(self):
        """Legacy - per compatibilità"""
        return abs(self.lateral_offset) < 0.10
    
    def add_forward_distance(self, meters):
        """Legacy wrapper - usa update_position invece"""
        self.update_position(meters)
    
    def add_lateral_offset(self, meters, direction):
        """Legacy - per compatibilità"""
        if direction == 'E':
            self.lateral_offset += meters
        elif direction == 'W':
            self.lateral_offset -= meters
 
    def print_status(self):
        print("\n" + "="*50)
        print("📍 STATO ROBOT (MAZE NAVIGATION)")
        print("="*50)
        print(f"Posizione: ({self.position_x:.2f}, {self.position_y:.2f})m")
        print(f"Heading: {self.heading}")
        print(f"Distanza da start: {self.get_distance_from_start():.2f}m / {self.target_distance:.2f}m")
        print(f"Distanza totale percorsa: {self.total_distance:.2f}m")
        print(f"Lidar: {self.last_lidar_distance:.1f}cm")
        print("="*50 + "\n")
 
 
# Istanza globale
simple_state = SimpleRobotState()
