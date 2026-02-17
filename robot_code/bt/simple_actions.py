"""
Azioni Semplici per Movimento Lineare - SENZA GRIGLIA
 
Include:
- Rotazioni 90° (N/E/S/W) con BOOST extra sul cingolo in RETRO (attrito alto).
- Micro-rotazioni a tempo (rotate_small) senza IMU.
- Scansione a ventaglio su UN lato (scan_fan_one_side): ritorna la PRIMA direzione libera.
- Movimento avanti (anche laterale se heading E/W), con controllo ostacoli.
- move_back_meters (compatibilità).
"""
import time
import math
from .actions import rover, DEFAULT_SPEED_LINEAR, DEFAULT_SPEED_TURN
from .simple_state import simple_state
 
 
 
# ============================================================================
# HELPERS
# ============================================================================
 
def _turn_speeds(direction_str: str):
    """
    Ritorna (speed_left, speed_right) per rotate_differential_compensated()
    applicando i fattori del simple_state (boost retro incluso).
    """
    if direction_str == 'left':
        lf, rf = simple_state.get_rotation_factors('L')
    elif direction_str == 'right':
        lf, rf = simple_state.get_rotation_factors('R')
    else:
        raise ValueError("direction_str deve essere 'left' o 'right'")
 
    return DEFAULT_SPEED_TURN * lf, DEFAULT_SPEED_TURN * rf
 
def _read_distance():
    d = rover.getUltrasonicSensor()
    # se torna 400 spesso è "lettura non valida / out of range"
    if d >= 390:
        # riprova una volta
        time.sleep(0.05)
        d2 = rover.getUltrasonicSensor()
        if d2 < d:
            d = d2
    simple_state.last_lidar_distance = d
    return d
 
# ============================================================================
# ROTAZIONI CARDINALI (90°)
# ============================================================================
 
def rotate_to_heading(target_heading):
    """
    Ruota verso heading target scegliendo percorso più breve (multipli di 90°).
    Aggiorna simple_state.heading.
    """
    if simple_state.heading == target_heading:
        print(f"✓ Già orientato verso {target_heading}")
        return True
 
    heading_order = ['N', 'E', 'S', 'W']
    current_idx = heading_order.index(simple_state.heading)
    target_idx = heading_order.index(target_heading)
    clockwise_rotations = (target_idx - current_idx) % 4
 
    print(f"🔄 Rotazione {simple_state.heading} → {target_heading}", end="")
 
    def do_turn(direction_str, t):
        speed_left, speed_right = _turn_speeds(direction_str)
        rover.rotate_differential_compensated(direction_str, speed_left, speed_right)
        time.sleep(t)
        rover.stop()
        time.sleep(0.30)
 
    # Usa tempi separati se calibrati, altrimenti usa il tempo base
    left_time = simple_state.rotation_90_time_left if simple_state.rotation_90_time_left else simple_state.rotation_90_time
    right_time = simple_state.rotation_90_time_right if simple_state.rotation_90_time_right else simple_state.rotation_90_time
 
    # percorso più breve
    if clockwise_rotations == 3:
        print(f" (90° sx, {left_time:.2f}s)")
        do_turn('left', left_time)
    elif clockwise_rotations == 2:
        print(f" (180°, 2 rotazioni dx)")
        for _ in range(2):
            do_turn('right', right_time)
            time.sleep(0.20)
    elif clockwise_rotations == 1:
        print(f" (90° dx, {right_time:.2f}s)")
        do_turn('right', right_time)
 
    simple_state.heading = target_heading
    print(f"✓ Heading aggiornato: {target_heading}")
    return True
 
 
# ============================================================================
# MICRO-ROTAZIONI (senza IMU)
# ============================================================================
 
def rotate_small(side, degrees):
    """
    Micro-rotazione stimata a tempo.
    side: 'left' o 'right'
    degrees: es. 15, 30
    NON aggiorna simple_state.heading (perché non è N/E/S/W).
    """
    if degrees <= 0:
        return True
 
    t = simple_state.rotation_90_time * (degrees / 90.0)
    speed_left, speed_right = _turn_speeds(side)
 
    rover.rotate_differential_compensated(side, speed_left, speed_right)
    time.sleep(t)
    rover.stop()
 
    settle = getattr(simple_state, "micro_turn_settle", 0.12)
    time.sleep(settle)
    return True
 
 
def rotate_to_micro_angle(side, angle, step=None):
    """
    Ruota di 'angle' gradi verso side usando micro-rotazioni.
    side: 'left' o 'right'
    """
    if angle <= 0:
        return True
 
    if step is None:
        step = getattr(simple_state, "micro_turn_degrees", 30)
 
    remaining = int(angle)
    while remaining > 0:
        a = step if remaining >= step else remaining
        rotate_small(side, a)
        remaining -= a
    return True
 
 
def rotate_back_from_micro_angle(side, angle, step=None):
    """
    Annulla una micro-rotazione precedente per tornare alla direzione iniziale (stimata).
    """
    back = 'right' if side == 'left' else 'left'
    return rotate_to_micro_angle(back, angle, step=step)
 
 
# ============================================================================
# SCANSIONE A VENTAGLIO (LEFT-FIRST la gestisce il BT)
# ============================================================================
 
def scan_fan_one_side(side='left', max_angle=90, step=None, threshold=None, strong_free=None):
    """
    Scansiona SOLO un lato e ritorna una direzione buona.
 
    - Se trova una direzione MOLTO libera (strong_free) si ferma subito (risparmia batteria).
    - Altrimenti arriva fino a max_angle e ritorna l'angolo con distanza migliore >= threshold.
    - Ignora letture "finte" tipo 400cm (out-of-range/fallback).
    """
    if step is None:
        step = getattr(simple_state, "micro_turn_degrees", 30)
    if threshold is None:
        threshold = getattr(simple_state, "scan_threshold", 50.0)
    if strong_free is None:
        strong_free = 120.0  # cm: se supera questo, è davvero libera -> stop subito
 
    best = None  # {"side":..., "angle":..., "dist":...}
    angle = 0
 
    for _ in range(int(max_angle / step)):
        angle += step
        rotate_small(side, step)
        d = _read_distance()
 
        # Filtra valori "finti"
        if d >= 390:
            continue
 
        # Se è super libero -> stop immediato
        if d >= strong_free:
            rotate_back_from_micro_angle(side, angle, step=step)
            return {"side": side, "angle": angle, "dist": d}
 
        # Se sopra soglia normale, tieni il migliore
        if d >= threshold:
            if (best is None) or (d > best["dist"]):
                best = {"side": side, "angle": angle, "dist": d}
 
    # torna allo zero stimato
    if angle > 0:
        rotate_back_from_micro_angle(side, angle, step=step)
 
    return best
 
 
# ============================================================================
# MOVIMENTO
# ============================================================================
 
def move_forward_meters(meters, check_obstacles=True, micro_side=None, micro_angle_deg=0):
    """
    Muove avanti nella direzione corrente.
 
    Se micro_side è 'left' o 'right' e micro_angle_deg>0:
      aggiorna lo stato con trigonometria (stima) invece che solo N/E/S/W.
    """
    if meters <= 0:
        return True
 
    # tempo movimento: usa forward come base
    # (anche se angolato, la velocità è quella lineare)
    time_needed = meters / simple_state.meters_per_second_forward
 
    # Se stai andando "laterale" cardinale E/W, usa m/s laterale
    if micro_side is None and simple_state.heading in ('E', 'W'):
        time_needed = meters / simple_state.meters_per_second_lateral
 
    print(f"➡️  Movimento {simple_state.heading} per {meters:.2f}m (~{time_needed:.1f}s)...")
 
    speed_left = DEFAULT_SPEED_LINEAR * simple_state.left_factor
    speed_right = DEFAULT_SPEED_LINEAR * simple_state.right_factor
 
    rover.moveTo('Forward', speed_left, speed_right)
 
    start_time = time.time()
    last_check = 0.0
 
    while time.time() - start_time < time_needed:
        elapsed = time.time() - start_time
 
        if check_obstacles and (elapsed - last_check) > 0.20:
            distance = _read_distance()
            print(f"   📡 {distance:.1f}cm", end='\r')
            last_check = elapsed
 
            if distance < simple_state.obstacle_threshold:
                rover.stop()
                print(f"\n🚧 OSTACOLO rilevato a {distance:.1f}cm!")
                return False
 
        time.sleep(0.05)
 
    rover.stop()
    time.sleep(0.15)
 
    # -------------------------
    # AGGIORNAMENTO STATO
    # -------------------------
    if micro_side in ('left', 'right') and micro_angle_deg > 0:
        # angolo rispetto al Nord
        theta = math.radians(float(micro_angle_deg))
 
        forward_component = meters * math.cos(theta)
        lateral_component = meters * math.sin(theta)
 
        # avanti “verso Nord”
        simple_state.distance_traveled += forward_component
 
        # laterale: left = Ovest (-), right = Est (+)
        if micro_side == 'right':
            simple_state.lateral_offset += lateral_component
        else:  # left
            simple_state.lateral_offset -= lateral_component
 
        print(f"\n✓ Step angolato: +{forward_component:.2f}m Nord, {('+' if micro_side=='right' else '-')}{lateral_component:.2f}m laterale")
        print(f"   Stato stimato: dist={simple_state.distance_traveled:.2f}m, offset={simple_state.lateral_offset:+.2f}m")
        return True
 
    # Cardinale classico
    if simple_state.heading == 'N':
        simple_state.distance_traveled += meters
        print(f"\n✓ Avanzato verso Nord: {simple_state.distance_traveled:.2f}m")
    elif simple_state.heading == 'S':
        simple_state.distance_traveled -= meters
        print(f"\n✓ Avanzato verso Sud: {simple_state.distance_traveled:.2f}m")
    elif simple_state.heading == 'E':
        simple_state.lateral_offset += meters
        print(f"\n✓ Spostato verso Est: offset {simple_state.lateral_offset:+.2f}m")
    elif simple_state.heading == 'W':
        simple_state.lateral_offset -= meters
        print(f"\n✓ Spostato verso Ovest: offset {simple_state.lateral_offset:+.2f}m")
 
    return True
 
 
def move_timed(seconds, check_obstacles=True):
    """
    Muove avanti per un tempo fisso in secondi.
    Utile quando l'odometria non è affidabile.
    NON aggiorna lo stato del robot (distanza/offset).
    """
    if seconds <= 0:
        return True
 
    print(f"➡️  Movimento per {seconds:.1f}s...")
 
    speed_left = DEFAULT_SPEED_LINEAR * simple_state.left_factor
    speed_right = DEFAULT_SPEED_LINEAR * simple_state.right_factor
 
    rover.moveTo('Forward', speed_left, speed_right)
 
    start_time = time.time()
    last_check = 0.0
 
    while time.time() - start_time < seconds:
        elapsed = time.time() - start_time
 
        if check_obstacles and (elapsed - last_check) > 0.20:
            distance = _read_distance()
            print(f"   📡 {distance:.1f}cm", end='\r')
            last_check = elapsed
 
            if distance < simple_state.obstacle_threshold:
                rover.stop()
                print(f"\n🚧 OSTACOLO rilevato a {distance:.1f}cm!")
                return False
 
        time.sleep(0.05)
 
    rover.stop()
    time.sleep(0.15)
    
    print(f"\n✓ Movimento completato ({seconds:.1f}s)")
    return True
 
 
# ============================================================================
# SENSORI
# ============================================================================
 
def check_obstacle_ahead(threshold=None):
    """
    Controlla ostacolo davanti (cioè nella direzione in cui il robot guarda ORA).
    """
    if threshold is None:
        threshold = simple_state.obstacle_threshold
 
    distance = _read_distance()
    return (distance < threshold), distance
 
 
def scan_directions():
    """
    Scansiona N/E/S/W (cardinali).
    """
    original_heading = simple_state.heading
    distances = {}
 
    print("\n🔍 Scansione direzioni...")
 
    for direction in ['N', 'E', 'S', 'W']:
        rotate_to_heading(direction)
        time.sleep(0.20)
        _, distance = check_obstacle_ahead()
        distances[direction] = distance
        status = "✅ LIBERO" if distance >= simple_state.obstacle_threshold else "❌ BLOCCATO"
        print(f"   {direction}: {distance:.1f}cm {status}")
 
    rotate_to_heading(original_heading)
    return distances
 
 
# ============================================================================
# UTILITY: INDIETRO
# ============================================================================
 
def move_back_meters(meters):
    """
    Indietreggia di X metri (compatibilità).
    """
    if meters <= 0:
        return True
 
    print(f"⬅️  Indietreggiamento {meters:.2f}m...")
 
    speed_left = DEFAULT_SPEED_LINEAR * simple_state.left_factor
    speed_right = DEFAULT_SPEED_LINEAR * simple_state.right_factor
    time_needed = meters / simple_state.meters_per_second_forward
 
    rover.moveTo('Back', speed_left, speed_right)
    time.sleep(time_needed)
    rover.stop()
    time.sleep(0.15)
 
    if simple_state.heading == 'N':
        simple_state.distance_traveled -= meters
    elif simple_state.heading == 'S':
        simple_state.distance_traveled += meters
    elif simple_state.heading == 'E':
        simple_state.lateral_offset -= meters
    elif simple_state.heading == 'W':
        simple_state.lateral_offset += meters
 
    print(f"✓ Indietreggiato {meters:.2f}m")
    return True
 
 
def move_backward_meters(meters):
    return move_back_meters(meters)
 
 