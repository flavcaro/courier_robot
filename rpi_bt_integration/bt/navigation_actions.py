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




# ============================================================================
# TIME-BASED NAVIGATION (Calibrated Movement)
# ============================================================================

def rotate_90_degrees(direction='right', compensate_drift=True):
    """
    Ruota esattamente 90° usando tempo calibrato.
    
    Args:
        direction: 'right' o 'left'
        compensate_drift: Se True, compensa lo slittamento all'indietro
    """
    print(f"🔄 Rotazione 90° a {'destra' if direction == 'right' else 'sinistra'}...")
    
    if direction == 'right':
        rover.moveTo('Right', 1.0)
    else:
        rover.moveTo('Left', 1.0)
    
    time.sleep(robot_state.rotation_90_time)
    rover.stop()
    time.sleep(0.2)  # Pausa per stabilizzazione
    
    # Compensazione drift: piccolo movimento avanti per recuperare slittamento
    if compensate_drift:
        rover.moveTo('Forward', 0.5)  # 50% velocità
        time.sleep(0.3)  # ~200ms di movimento per compensare
        rover.stop()
        time.sleep(0.1)
    
    print(f"✓ Rotazione completata!")


def rotate_180_degrees():
    """
    Ruota esattamente 180° usando tempo calibrato.
    Usa una singola rotazione continua invece di 2x90° per evitare accumulo errori.
    """
    print(f"🔄 Rotazione 180° (inversione direzione)...")
    
    # Rotazione continua di 180° = doppio tempo di 90°
    rover.moveTo('Right', 1.0)
    time.sleep(robot_state.rotation_90_time * 2.3)
    rover.stop()
    time.sleep(0.3)  # Pausa più lunga per stabilizzazione
    
    print(f"✓ Rotazione 180° completata!")



def move_one_cell_forward():
    """Muove avanti di esattamente una cella (60cm) usando tempo calibrato."""
    print(f"➡️  Movimento avanti di una cella (60cm)...")
    
    rover.moveTo('Forward', 1.0)
    time.sleep(robot_state.cell_move_time)
    rover.stop()
    time.sleep(0.2)  # Pausa per stabilizzazione
    
    print(f"✓ Cella raggiunta!")


def move_to_cell(target_row, target_col):
    """
    Muove verso cella usando movimenti temporizzati.
    IMPORTANTE: Può muoversi solo di 1 cella alla volta!
    
    Args:
        target_row, target_col: Coordinate cella target
        
    Returns:
        bool: True se cella raggiunta, False altrimenti
    """
    # Posizione corrente
    current_row, current_col = robot_state.world_to_cell(
        robot_state.robot_x, robot_state.robot_y
    )
    
    drow = target_row - current_row
    dcol = target_col - current_col
    
    # Verifica che sia movimento di 1 cella
    if abs(drow) + abs(dcol) != 1:
        print(f"❌ Errore: move_to_cell può muoversi solo di 1 cella alla volta!")
        print(f"   Da ({current_row},{current_col}) a ({target_row},{target_col})")
        return False
    
    print(f"🎯 Movimento da ({current_row},{current_col}) a ({target_row},{target_col})")
    
    # Determina direzione target
    if dcol == 1:
        target_heading = 'E'  # Vai a EST (destra)
    elif dcol == -1:
        target_heading = 'W'  # Vai a OVEST (sinistra)
    elif drow == 1:
        target_heading = 'N'  # Vai a NORD (avanti)
    elif drow == -1:
        target_heading = 'S'  # Vai a SUD (indietro)
    else:
        print(f"❌ Errore: movimento non valido")
        return False
    
    # Calcola rotazioni necessarie (sempre a destra)
    # N=0, E=1, S=2, W=3 rotazioni da N
    heading_to_rotations = {
        'N': {'N': 0, 'E': 1, 'S': 2, 'W': 3},
        'E': {'N': 3, 'E': 0, 'S': 1, 'W': 2},
        'S': {'N': 2, 'E': 3, 'S': 0, 'W': 1},
        'W': {'N': 1, 'E': 2, 'S': 3, 'W': 0}
    }
    
    current_heading = robot_state.robot_heading
    rotations_needed = heading_to_rotations[current_heading][target_heading]
    
    # Ottimizzazione: scegli il percorso più breve
    if rotations_needed == 3:
        print(f"📍 Heading: {current_heading} → {target_heading} (1 rotazione a sinistra)")
        rotate_90_degrees('left')
    elif rotations_needed == 2:
        # 180 gradi: 2 rotazioni separate con tempo aumentato (7s invece di 6.5s)
        print(f"📍 Heading: {current_heading} → {target_heading} (2 rotazioni da 90°, 7s ciascuna)")
        # Usa tempo custom per rotazioni di 180°
        original_time = robot_state.rotation_90_time
        robot_state.rotation_90_time = 7.0  # Aumenta a 7 secondi
        rotate_90_degrees('right', compensate_drift=False)
        rotate_90_degrees('right', compensate_drift=False)
        robot_state.rotation_90_time = original_time  # Ripristina
    elif rotations_needed == 1:
        print(f"📍 Heading: {current_heading} → {target_heading} (1 rotazione a destra)")
        rotate_90_degrees('right')
    else:
        print(f"📍 Heading: {current_heading} (già orientato correttamente)")
    
    # Muovi avanti di una cella
    move_one_cell_forward()
    
    # Aggiorna stato robot
    target_x, target_y = robot_state.cell_to_world(target_row, target_col)
    robot_state.robot_x = target_x
    robot_state.robot_y = target_y
    robot_state.robot_heading = target_heading
    
    # Aggiorna yaw
    heading_to_yaw = {'N': math.pi/2, 'E': 0.0, 'S': -math.pi/2, 'W': math.pi}
    robot_state.robot_yaw = heading_to_yaw[target_heading]
    
    print(f"✅ Posizione aggiornata: ({target_x:.2f}, {target_y:.2f}), Heading: {target_heading}")
    
    return True


# ============================================================================
# BFS PATHFINDING
# ============================================================================

def rotate_to_angle(target_yaw, speed=0.5, tolerance=math.radians(5)):
    """
    Ruota il robot verso un angolo target (yaw).
    
    Args:
        target_yaw: Angolo target in radianti (normalizzato in [-pi, pi])
        speed: Velocità di rotazione (0-1)
        tolerance: Tolleranza angolare in radianti
        
    Returns:
        bool: True se angolo raggiunto, False se timeout o ostacolo
    """
    max_iterations = 200  # Aumentato per sicurezza
    iteration = 0
    
    print(f"🔄 Rotazione: da {math.degrees(robot_state.robot_yaw):.1f}° a {math.degrees(target_yaw):.1f}°")
    
    while iteration < max_iterations:
        # Calcola errore angolare
        angle_error = normalize_angle(target_yaw - robot_state.robot_yaw)
        
        # Debug ogni 10 iterazioni
        if iteration % 10 == 0:
            print(f"   [Iter {iteration}] Yaw={math.degrees(robot_state.robot_yaw):.1f}°, Errore={math.degrees(angle_error):.1f}°")
        
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
            angular_velocity = speed * 0.5  # rad/s stimato (robot ruota più lento del previsto)
        else:
            rover.moveTo('Right', speed)
            angular_velocity = -speed * 0.5  # rad/s stimato (robot ruota più lento del previsto)
        
        time.sleep(0.1)
        rover.stop()
        
        # Aggiorna odometria
        robot_state.robot_yaw += angular_velocity * 0.1
        robot_state.robot_yaw = normalize_angle(robot_state.robot_yaw)
        
        iteration += 1
    
    rover.stop()
    print(f"⚠️ Timeout rotazione dopo {max_iterations} iterazioni")
    print(f"   Yaw finale: {math.degrees(robot_state.robot_yaw):.1f}°, Target: {math.degrees(target_yaw):.1f}°")
    return False


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
