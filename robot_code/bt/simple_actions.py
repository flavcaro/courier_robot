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
 
def rotate_90_with_imu(direction_str, target_degrees=90.0, timeout=5.0):
    """
    Rotazione di 90° usando feedback IMU invece del tempo.
   
    Args:
        direction_str: 'left' o 'right'
        target_degrees: angolo target (default 90°)
        timeout: timeout sicurezza in secondi
       
    Returns:
        True se completata, False se timeout
    """
    if not simple_state.imu.is_available():
        print("⚠️ IMU non disponibile, usa rotate_to_heading tradizionale")
        return False
   
    # Salva heading iniziale per misurare la rotazione relativa
    simple_state.imu.update_heading()
    initial_heading = simple_state.imu.get_heading()
   
    # Calcola velocità motori
    speed_left, speed_right = _turn_speeds(direction_str)
   
    print(f"🧭 Rotazione IMU {direction_str} target={target_degrees}°", end="")
   
    # Inizia rotazione
    rover.rotate_differential_compensated(direction_str, speed_left, speed_right)
   
    start_time = time.time()
    angle_reached = False
   
    try:
        while time.time() - start_time < timeout:
            # Aggiorna heading
            simple_state.imu.update_heading()
            current_heading = simple_state.imu.get_heading()
           
            # Calcola angolo ruotato rispetto al punto iniziale
            if direction_str == 'left':
                angle_rotated = (current_heading - initial_heading) % 360
            else:  # right
                angle_rotated = (initial_heading - current_heading) % 360
           
            # Tolleranza ±2°
            if abs(angle_rotated - target_degrees) <= 2.0:
                angle_reached = True
                break
           
            time.sleep(0.02)  # 50Hz
           
    finally:
        rover.stop()
        time.sleep(0.30)
   
    if angle_reached:
        final_heading = simple_state.imu.get_heading()
        print(f" ✓ {final_heading:.1f}° in {time.time()-start_time:.2f}s")
        return True
    else:
        print(f" ⚠️ TIMEOUT dopo {timeout}s")
        return False
 
 
def rotate_to_heading(target_heading):
    """
    Ruota verso heading target scegliendo percorso più breve (multipli di 90°).
    Aggiorna simple_state.heading.
    Usa IMU se abilitato, altrimenti tempo.
    """
    if simple_state.heading == target_heading:
        print(f"✓ Già orientato verso {target_heading}")
        return True
 
    heading_order = ['N', 'E', 'S', 'W']
    current_idx = heading_order.index(simple_state.heading)
    target_idx = heading_order.index(target_heading)
    clockwise_rotations = (target_idx - current_idx) % 4
 
    print(f"🔄 Rotazione {simple_state.heading} → {target_heading}", end="")
 
    def do_turn_timed(direction_str, t):
        speed_left, speed_right = _turn_speeds(direction_str)
        rover.rotate_differential_compensated(direction_str, speed_left, speed_right)
        time.sleep(t)
        rover.stop()
        time.sleep(0.30)
   
    def do_turn_imu(direction_str):
        return rotate_90_with_imu(direction_str, target_degrees=90.0)
 
    # Usa tempi separati se calibrati, altrimenti usa il tempo base
    left_time = simple_state.rotation_90_time_left if simple_state.rotation_90_time_left else simple_state.rotation_90_time
    right_time = simple_state.rotation_90_time_right if simple_state.rotation_90_time_right else simple_state.rotation_90_time
 
    # Scegli metodo: IMU o tempo
    use_imu = simple_state.use_imu_rotation and simple_state.imu.is_available()
   
    # percorso più breve
    if clockwise_rotations == 3:
        if use_imu:
            print(f" (90° sx IMU)")
            do_turn_imu('left')
        else:
            print(f" (90° sx, {left_time:.2f}s)")
            do_turn_timed('left', left_time)
    elif clockwise_rotations == 2:
        print(f" (180°, 2 rotazioni dx)")
        for _ in range(2):
            if use_imu:
                do_turn_imu('right')
            else:
                do_turn_timed('right', right_time)
            time.sleep(0.20)
    elif clockwise_rotations == 1:
        if use_imu:
            print(f" (90° dx IMU)")
            do_turn_imu('right')
        else:
            print(f" (90° dx, {right_time:.2f}s)")
            do_turn_timed('right', right_time)
 
    simple_state.heading = target_heading
    print(f"✓ Heading aggiornato: {target_heading}")
   
    return True
 
 
def realign_to_north_with_imu():
    """
    Riallinea il robot a Nord (0° rispetto al reference heading) usando IMU.
    Chiamata DOPO aggiramento ostacolo per correggere eventuali derive.
    """
    # Controlla se IMU è disponibile e abilitato
    if not simple_state.imu.is_available():
        return
   
    if not getattr(simple_state, 'use_imu_rotation', False):
        return
   
    if simple_state.heading != 'N':
        return  # Funziona solo se già orientato a Nord
   
    reference_heading = getattr(simple_state, 'imu_reference_heading', None)
    if reference_heading is None:
        return
   
    # Leggi heading corrente
    simple_state.imu.update_heading()
    current_heading = simple_state.imu.get_heading()
   
    # Calcola errore rispetto a Nord (reference_heading)
    error = current_heading - reference_heading
   
    # Normalizza (-180 a +180)
    if error > 180:
        error -= 360
    elif error < -180:
        error += 360
   
    # Se errore piccolo, non fare nulla
    if abs(error) < 2.0:
        print(f"✓ Robot già dritto: {current_heading:.1f}° (errore: {error:+.1f}°)")
        return
   
    # Correggi con rotazione closed-loop usando IMU
    print(f"🔧 Riallineamento a Nord: errore {error:+.1f}° - Correzione IMU...")
   
    # Determina direzione
    if error > 0:  # Deviato verso sinistra (heading maggiore del reference)
        direction = 'right'  # Ruota a destra per tornare indietro
        target_angle = error  # Angolo da recuperare
    else:  # Deviato verso destra (heading minore del reference)
        direction = 'left'  # Ruota a sinistra per tornare indietro
        target_angle = abs(error)  # Angolo da recuperare
   
    # Usa feedback IMU per correzione precisa
    speed_left, speed_right = _turn_speeds(direction)
    rover.rotate_differential_compensated(direction, speed_left, speed_right)
   
    start_time = time.time()
    timeout = 3.0  # Timeout per sicurezza
   
    try:
        while time.time() - start_time < timeout:
            simple_state.imu.update_heading()
            current_heading = simple_state.imu.get_heading()
           
            # Ricalcola errore
            error_now = current_heading - reference_heading
            if error_now > 180:
                error_now -= 360
            elif error_now < -180:
                error_now += 360
           
            # Se raggiunto target (entro ±1°), stop
            if abs(error_now) <= 1.0:
                break
           
            time.sleep(0.02)  # 50Hz
    finally:
        rover.stop()
        time.sleep(0.3)
   
    # Verifica risultato finale
    simple_state.imu.update_heading()
    final_heading = simple_state.imu.get_heading()
    final_error = final_heading - reference_heading
    if final_error > 180:
        final_error -= 360
    elif final_error < -180:
        final_error += 360
   
    print(f"✓ Riallineamento completato: {final_heading:.1f}° (errore residuo: {final_error:+.1f}°)")
 
 
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
# MOVIMENTO CON ODOMETRIA IMU
# ============================================================================
 
def move_forward_meters_with_imu(meters, check_obstacles=True, micro_side=None, micro_angle_deg=0):
    """
    Muove avanti usando l'accelerometro IMU per stimare la distanza percorsa.
    Include correzione automatica heading per mantenere traiettoria dritta.
   
    Args:
        meters: distanza target in metri
        check_obstacles: controlla ostacoli durante movimento
        micro_side: 'left'/'right' se movimento angolato
        micro_angle_deg: angolo del movimento rispetto alla direzione principale
   
    Returns:
        True se completato, False se ostacolo
    """
    if not simple_state.imu.is_available():
        # Fallback a movimento normale
        return move_forward_meters(meters, check_obstacles, micro_side, micro_angle_deg)
   
    if meters <= 0:
        return True
   
    print(f"➡️  Movimento IMU {simple_state.heading} per {meters:.2f}m...")
   
    # Velocità motori base
    base_speed_left = DEFAULT_SPEED_LINEAR * simple_state.left_factor
    base_speed_right = DEFAULT_SPEED_LINEAR * simple_state.right_factor
   
    # Reset odometria IMU
    distance_traveled_imu = 0.0
    velocity_x = 0.0
    velocity_y = 0.0
   
    # Inizia movimento
    rover.moveTo('Forward', base_speed_left, base_speed_right)
   
    start_time = time.time()
    last_time = start_time
    last_check = 0.0
    timeout = meters / simple_state.meters_per_second_forward * 8.0  # Timeout aumentato per movimenti lenti
   
    # Costante di filtraggio per velocità
    alpha_vel = 0.8
   
    while time.time() - start_time < timeout:
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time
       
        if dt < 0.01:
            time.sleep(0.01)
            continue
       
        # Leggi accelerazioni per odometria
        ax, ay = simple_state.imu.get_accel_xy()
       
        # Filtra e integra velocità (filtro esponenziale per ridurre rumore)
        velocity_x = alpha_vel * velocity_x + (1 - alpha_vel) * (ax * 9.81 * dt)
        velocity_y = alpha_vel * velocity_y + (1 - alpha_vel) * (ay * 9.81 * dt)
       
        # Calcola modulo velocità
        velocity = math.sqrt(velocity_x**2 + velocity_y**2)
       
        # Integra velocità → distanza
        distance_traveled_imu += velocity * dt
       
        # Controlla ostacoli
        elapsed = time.time() - start_time
        if check_obstacles and (elapsed - last_check) > 0.20:
            distance = _read_distance()
            print(f"   📡 {distance:.1f}cm | IMU: {distance_traveled_imu:.2f}m/{meters:.2f}m", end='\r')
            last_check = elapsed
           
            if distance < simple_state.obstacle_threshold:
                rover.stop()
                print(f"\n🚧 OSTACOLO rilevato a {distance:.1f}cm!")
                return False
       
        # Controlla se target raggiunto
        if distance_traveled_imu >= meters:
            break
       
        time.sleep(0.02)  # 50Hz
   
    rover.stop()
    time.sleep(0.15)
   
    print(f"\n✓ Completato: {distance_traveled_imu:.2f}m in {time.time()-start_time:.2f}s")
   
    # Aggiorna stato (maze-aware)
    if micro_side in ('left', 'right') and micro_angle_deg > 0:
        theta = math.radians(float(micro_angle_deg))
        forward_component = distance_traveled_imu * math.cos(theta)
        lateral_component = distance_traveled_imu * math.sin(theta)
       
        simple_state.distance_traveled += forward_component
       
        if micro_side == 'right':
            simple_state.lateral_offset += lateral_component
        else:
            simple_state.lateral_offset -= lateral_component
       
        print(f"   Step angolato: +{forward_component:.2f}m Nord, {('+' if micro_side=='right' else '-')}{lateral_component:.2f}m")
        print(f"   Stato: dist={simple_state.distance_traveled:.2f}m, offset={simple_state.lateral_offset:+.2f}m")
    else:
        # Movimento cardinale - usa nuova funzione update_position
        simple_state.update_position(distance_traveled_imu)
        print(f"   Posizione: ({simple_state.position_x:.2f}, {simple_state.position_y:.2f})m")
        print(f"   Distanza da start: {simple_state.get_distance_from_start():.2f}m / {simple_state.target_distance:.2f}m")
   
    return True
 
 
# ============================================================================
# MOVIMENTO (FALLBACK SENZA IMU)
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
 
    # Cardinale classico - usa nuova funzione update_position
    simple_state.update_position(meters)
    print(f"\n✓ Movimento completato: Posizione ({simple_state.position_x:.2f}, {simple_state.position_y:.2f})m")
    print(f"   Distanza da start: {simple_state.get_distance_from_start():.2f}m")
 
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
 
 
def scan_all_directions_maze():
    """
    Scansiona tutte le 4 direzioni cardinali (N/E/S/W) e ritorna info per maze navigation.
   
    Returns:
        dict: {
            'N': {'distance': float, 'blocked': bool, 'free': bool},
            'E': {...},
            'S': {...},
            'W': {...}
        }
    """
    original_heading = simple_state.heading
    results = {}
   
    print("\n🔍 Scansione maze (4 direzioni)...")
   
    for direction in ['N', 'E', 'S', 'W']:
        rotate_to_heading(direction)
        time.sleep(0.15)
       
        # Controlla ostacolo fisico
        blocked, distance = check_obstacle_ahead(threshold=simple_state.obstacle_threshold)
       
        results[direction] = {
            'distance': distance,
            'blocked': blocked,
            'free': not blocked  # libero se non bloccato
        }
       
        # Status
        status = "✅ LIBERO" if not blocked else "🚧 BLOCCATO"
        print(f"   {direction}: {distance:.1f}cm - {status}")
   
    rotate_to_heading(original_heading)
    return results
 
 
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
 
    # Aggiorna posizione (movimento all'indietro = negativo)
    simple_state.update_position(-meters)
 
    print(f"✓ Indietreggiato {meters:.2f}m")
    return True
 
 
def move_backward_meters(meters):
    return move_back_meters(meters)
 
 
 
 
 
 
 