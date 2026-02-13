"""
Sensori per il robot - Integrato con rover_API esistente.
"""
from rover_API import RoverApi
import time
import math

# Usa l'istanza rover globale da actions.py
from .actions import rover


def get_ultrasonic_distance():
    """
    Legge la distanza dal sensore ultrasuoni.
    
    Returns:
        float: Distanza in cm (400.0 se nessun ostacolo)
    """
    try:
        distance = rover.getUltrasonicSensor()
        return distance
    except Exception as e:
        print(f"Errore lettura ultrasuoni: {e}")
        return 400.0


def check_obstacle(threshold_cm=40.0):
    """
    Controlla se c'è un ostacolo entro la soglia.
    
    Args:
        threshold_cm: Distanza soglia in cm
        
    Returns:
        bool: True se ostacolo rilevato, False altrimenti
    """
    distance = get_ultrasonic_distance()
    return distance < threshold_cm


class RobotState:
    """
    Stato del robot per navigazione con griglia.
    Mantiene posizione stimata e parametri di configurazione.
    """
    
    def __init__(self):
        # === Configurazione Griglia ===
        self.cell_size = 0.6  # 60cm × 60cm
        self.grid_size = 3    # Griglia 3×3
        self.obstacles = {(1, 1)}  # Ostacoli predefiniti
        
        # === Parametri Missione ===
        self.start_cell = (0, 0)
        self.goal_cell = (1, 2)
        
        # === Stato Robot (odometria stimata) ===
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = math.pi / 2  # Radianti - Robot inizia orientato NORD (avanti)
        self.robot_heading = 'N'  # Direzione cardinale: 'N', 'E', 'S', 'W'
        
        # === Parametri Controllo ===
        self.rotation_speed = 1.0    # 100% - rotazione richiede potenza massima
        self.linear_speed = 1.0      # 100% - movimento fluido senza scatti
        self.angle_tolerance = 0.20  # ~11.5°
        self.position_tolerance = 0.25  # 25cm
        
        # === Calibrazione Movimento Temporizzato (valori base a tensione di riferimento) ===
        self.base_rotation_90_time = 5.9  # secondi per ruotare 90° al 100% con batteria carica
        self.base_cell_move_time = 4.5    # secondi per muoversi 0.6m (una cella) al 100%
        
        # === Compensazione Tensione Batteria ===
        self.reference_voltage = 7.4  # Tensione di riferimento (batteria LiPo 2S carica)
        self.current_voltage = 7.4    # Tensione corrente (aggiornata dinamicamente)
        self.min_voltage = 6.4        # Tensione minima consigliata
        self.compensation_factor = 1.0  # Fattore di compensazione calcolato
        
        # Tempi compensati (aggiornati automaticamente)
        self.rotation_90_time = self.base_rotation_90_time
        self.cell_move_time = self.base_cell_move_time
        
        # === Sensori ===
        self.front_distance = 400.0
        self.obstacle_threshold = 40.0  # cm
        
        # === Timing ===
        self.start_time = time.time()
        
    def update_battery_voltage(self):
        """Legge tensione batteria e aggiorna fattore di compensazione."""
        try:
            self.current_voltage = rover.getBatteryVoltage()
            
            # Calcola fattore di compensazione
            # Quando la batteria è scarica, i motori girano più lenti
            # quindi serve PIÙ tempo per fare la stessa rotazione
            self.compensation_factor = self.reference_voltage / self.current_voltage
            
            # Applica compensazione ai tempi
            self.rotation_90_time = self.base_rotation_90_time * self.compensation_factor
            self.cell_move_time = self.base_cell_move_time * self.compensation_factor
            
            print(f"🔋 Batteria: {self.current_voltage:.2f}V (riferimento: {self.reference_voltage}V)")
            print(f"   Fattore compensazione: {self.compensation_factor:.3f}")
            print(f"   Tempo rotazione 90°: {self.rotation_90_time:.2f}s (base: {self.base_rotation_90_time}s)")
            
            # Avviso se batteria bassa
            if self.current_voltage < self.min_voltage:
                print(f"⚠️  ATTENZIONE: Batteria bassa ({self.current_voltage:.2f}V)! Ricaricare presto.")
                
        except Exception as e:
            print(f"❌ Errore lettura tensione batteria: {e}")
            print(f"   Uso valori base senza compensazione")
            self.compensation_factor = 1.0
    
    def update_sensors(self):
        """Aggiorna letture sensori."""
        self.front_distance = get_ultrasonic_distance()
        
    def cell_to_world(self, row, col):
        """Converte coordinate cella in coordinate mondo (centro cella)."""
        x = (col + 0.5) * self.cell_size
        y = (row + 0.5) * self.cell_size
        return x, y
    
    def world_to_cell(self, x, y):
        """Converte coordinate mondo in coordinate cella."""
        col = int(x / self.cell_size)
        row = int(y / self.cell_size)
        return row, col


# Istanza globale dello stato robot
robot_state = RobotState()
