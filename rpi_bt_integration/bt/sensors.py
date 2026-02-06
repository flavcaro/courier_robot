"""
Sensori per il robot - Integrato con rover_API esistente.
"""
from rover_API import RoverApi
import time

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
        self.robot_yaw = 0.0  # Radianti
        
        # === Parametri Controllo ===
        self.rotation_speed = 0.6    # 60% - robot si muove bene
        self.linear_speed = 0.5      # 50% - velocità ottimale
        self.angle_tolerance = 0.20  # ~11.5°
        self.position_tolerance = 0.25  # 25cm
        
        # === Sensori ===
        self.front_distance = 400.0
        self.obstacle_threshold = 40.0  # cm
        
        # === Timing ===
        self.start_time = time.time()
        
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
