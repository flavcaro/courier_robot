"""
Actions estese per navigazione con griglia - Integrato con actions.py esistente.
"""
from rover_API import RoverApi
import time
import math

# Usa l'istanza rover globale
from .actions import rover, move_forward, move_back, move_left, move_right
from .actions import arm_up, arm_down, open_hand, close_hand
from .sensors import robot_state


def normalize_angle(angle):
    """Normalizza angolo in [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def rotate_to_angle(target_yaw, speed=0.3, tolerance=0.15):
    """
    Ruota il robot verso un angolo target.
    
    Args:
        target_yaw: Angolo target in radianti
        speed: Velocità rotazione (0-1)
        tolerance: Tolleranza angolare in radianti
        
    Returns:
        bool: True se rotazione completata, False se ostacolo
    """
    max_iterations = 50
    iteration = 0
    
    while iteration < max_iterations:
        # Calcola errore angolare
        angle_error = normalize_angle(target_yaw - robot_state.robot_yaw)
        
        # Controlla se raggiunto
        if abs(angle_error) < tolerance:
            rover.stop()
            print(f"✓ Rotazione completata! Yaw={math.degrees(robot_state.robot_yaw):.1f}°")
            
            # Controlla ostacolo
            robot_state.update_sensors()
            if robot_state.front_distance < robot_state.obstacle_threshold:
                print(f"⚠️ Ostacolo a {robot_state.front_distance:.1f}cm!")
                return False
            return True
        
        # Ruota
        if angle_error > 0:
            rover.moveTo('Left', speed)
            angular_velocity = speed * 1.0  # rad/s stimato
        else:
            rover.moveTo('Right', speed)
            angular_velocity = -speed * 1.0
        
        time.sleep(0.1)
        rover.stop()
        
        # Aggiorna odometria
        robot_state.robot_yaw += angular_velocity * 0.1
        robot_state.robot_yaw = normalize_angle(robot_state.robot_yaw)
        
        iteration += 1
    
    rover.stop()
    print("⚠️ Timeout rotazione")
    return False


def move_to_position(target_x, target_y, speed=0.2, tolerance=0.15):
    """
    Muove il robot verso una posizione target.
    
    Args:
        target_x, target_y: Coordinate target
        speed: Velocità lineare (0-1)
        tolerance: Tolleranza posizione in metri
        
    Returns:
        bool: True se posizione raggiunta, False se ostacolo
    """
    max_iterations = 100
    iteration = 0
    
    while iteration < max_iterations:
        # Calcola distanza
        dx = target_x - robot_state.robot_x
        dy = target_y - robot_state.robot_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Controlla se raggiunto
        if distance < tolerance:
            rover.stop()
            print(f"✓ Posizione raggiunta! ({robot_state.robot_x:.2f}, {robot_state.robot_y:.2f})")
            return True
        
        # Controlla ostacolo
        robot_state.update_sensors()
        if robot_state.front_distance < robot_state.obstacle_threshold:
            rover.stop()
            print(f"⚠️ Ostacolo a {robot_state.front_distance:.1f}cm!")
            return False
        
        # Muovi avanti
        actual_speed = speed if distance > 0.3 else speed * 0.6
        rover.moveTo('Forward', actual_speed)
        
        time.sleep(0.1)
        rover.stop()
        
        # Aggiorna odometria
        linear_velocity = actual_speed * 0.3  # m/s stimato
        robot_state.robot_x += linear_velocity * math.cos(robot_state.robot_yaw) * 0.1
        robot_state.robot_y += linear_velocity * math.sin(robot_state.robot_yaw) * 0.1
        
        iteration += 1
    
    rover.stop()
    print("⚠️ Timeout movimento")
    return False


def move_to_cell(target_row, target_col):
    """
    Muove il robot verso una cella della griglia.
    
    Args:
        target_row, target_col: Coordinate cella target
        
    Returns:
        bool: True se cella raggiunta, False altrimenti
    """
    # Converti cella in coordinate mondo
    target_x, target_y = robot_state.cell_to_world(target_row, target_col)
    
    # Calcola direzione
    current_row, current_col = robot_state.world_to_cell(robot_state.robot_x, robot_state.robot_y)
    drow = target_row - current_row
    dcol = target_col - current_col
    
    # Determina yaw target (direzione cardinale)
    if dcol != 0:
        target_yaw = 0.0 if dcol > 0 else math.pi  # Est/Ovest
    elif drow != 0:
        target_yaw = math.pi / 2 if drow > 0 else -math.pi / 2  # Nord/Sud
    else:
        target_yaw = robot_state.robot_yaw  # Già sulla cella
    
    target_yaw = normalize_angle(target_yaw)
    
    print(f"🎯 Movimento verso cella ({target_row},{target_col}) = ({target_x:.2f}, {target_y:.2f})")
    
    # Ruota verso target
    if not rotate_to_angle(target_yaw):
        return False
    
    time.sleep(0.2)
    
    # Muovi verso target
    if not move_to_position(target_x, target_y):
        return False
    
    return True


def bfs_path(start, goal, obstacles, grid_size):
    """
    Calcola percorso con BFS.
    
    Args:
        start: Tupla (row, col) partenza
        goal: Tupla (row, col) arrivo
        obstacles: Set di tuple (row, col) ostacoli
        grid_size: Dimensione griglia NxN
        
    Returns:
        list: Lista di celle (row, col) del percorso, o None
    """
    from collections import deque
    
    if start == goal:
        return [start]
    
    queue = deque([(start, [start])])
    visited = {start}
    
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # N, S, W, E
    
    while queue:
        (row, col), path = queue.popleft()
        
        for dr, dc in directions:
            new_row, new_col = row + dr, col + dc
            new_cell = (new_row, new_col)
            
            # Controlla validità
            if (0 <= new_row < grid_size and 
                0 <= new_col < grid_size and
                new_cell not in obstacles and
                new_cell not in visited):
                
                new_path = path + [new_cell]
                
                if new_cell == goal:
                    return new_path
                
                queue.append((new_cell, new_path))
                visited.add(new_cell)
    
    return None  # Nessun percorso trovato
