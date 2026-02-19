 
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
 
 
def update_imu_reference_after_rotation():
    """
    Aggiorna il riferimento IMU dopo una rotazione per compensare drift accumulato.
    Questo previene che l'errore si accumuli dopo molte rotazioni.
    """
    if not simple_state.imu.is_available():
        return
   
    if not hasattr(simple_state, 'imu_reference_heading') or simple_state.imu_reference_heading is None:
        return
   
    # Mappa direzioni cardinali agli angoli IMU
    heading_map = {'N': 0, 'W': 90, 'S': 180, 'E': 270}
    if simple_state.heading not in heading_map:
        return
   
    # Leggi heading attuale con piccola media per ridurre rumore
    readings = []
    for _ in range(3):
        simple_state.imu.update_heading()
        readings.append(simple_state.imu.get_heading())
        time.sleep(0.02)
    current_heading = sum(readings) / len(readings)
   
    # Calcola nuovo riferimento: sottrai l'offset atteso per questa direzione
    # Es: Se siamo a W (90°) e heading IMU è 91.5°, nuovo ref = 91.5 - 90 = 1.5°
    expected_offset = heading_map[simple_state.heading]
    new_reference = (current_heading - expected_offset) % 360
   
    # Normalizza a range ±180° per evitare valori strani
    if new_reference > 180:
        new_reference -= 360
   
    # Aggiorna il riferimento per i prossimi movimenti
    old_ref = simple_state.imu_reference_heading
    simple_state.imu_reference_heading = new_reference
   
    # Debug: mostra aggiornamento riferimento
    if getattr(simple_state, 'imu_heading_correction_debug', False):
        print(f"   🔄 Rif IMU: {old_ref:.1f}° → {new_reference:.1f}° (heading: {current_heading:.1f}°)")
 
 
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
            simple_state.heading = target_heading
            # DISABILITATO: Aggiornare riferimento accumula errori invece di compensarli
            # update_imu_reference_after_rotation()
        else:
            print(f" (90° sx, {left_time:.2f}s)")
            do_turn_timed('left', left_time)
            simple_state.heading = target_heading
    elif clockwise_rotations == 2:
        print(f" (180°, 2 rotazioni dx)")
        for i in range(2):
            if use_imu:
                do_turn_imu('right')
                # Aggiorna heading per tracking
                heading_order = ['N', 'E', 'S', 'W']
                current_idx_loop = heading_order.index(simple_state.heading)
                simple_state.heading = heading_order[(current_idx_loop + 1) % 4]
                # DISABILITATO: Aggiornare riferimento accumula errori
                # update_imu_reference_after_rotation()
            else:
                do_turn_timed('right', right_time)
            time.sleep(0.20)
    elif clockwise_rotations == 1:
        if use_imu:
            print(f" (90° dx IMU)")
            do_turn_imu('right')
            simple_state.heading = target_heading
            # DISABILITATO: Aggiornare riferimento accumula errori invece di compensarli
            # update_imu_reference_after_rotation()
        else:
            print(f" (90° dx, {right_time:.2f}s)")
            do_turn_timed('right', right_time)
            simple_state.heading = target_heading
 
    # Conferma (già aggiornato sopra per IMU)
    if not use_imu:
        simple_state.heading = target_heading
    print(f"✓ Heading aggiornato: {target_heading}")
   
    # DISABILITATO: Riallineamento causava più errori che benefici (IMU rumoroso vicino ai motori)
    # if use_imu:
    #     realign_to_north_with_imu()
   
    return True
 
 
def realign_to_north_with_imu():
    """
    Riallinea il robot alla direzione cardinale corrente usando IMU.
    Funziona per tutte le direzioni (giroscopio misura in senso ANTIORARIO):
    N=0°, W=90° (sinistra), S=180°, E=270° (destra)
    """
    # Controlla se IMU è disponibile e abilitato
    if not simple_state.imu.is_available():
        return
   
    if not getattr(simple_state, 'use_imu_rotation', False):
        return
   
    reference_heading = getattr(simple_state, 'imu_reference_heading', None)
    if reference_heading is None:
        return
   
    # Mappa heading corrente all'angolo target assoluto
    # IMPORTANTE: Il giroscopio misura in senso ANTIORARIO!
    # N=0°, W=90° (sinistra), S=180°, E=270° (destra)
    heading_map = {'N': 0, 'W': 90, 'S': 180, 'E': 270}
    if simple_state.heading not in heading_map:
        return
   
    # Calcola angolo target per la direzione corrente
    target_angle = (reference_heading + heading_map[simple_state.heading]) % 360
   
    # Stabilizzazione IMU dopo rotazione (riduce rumore motori)
    time.sleep(0.3)  # Aspetta che vibrazioni si attenuino
   
    # Leggi heading corrente con media di più campioni per ridurre rumore
    readings = []
    for _ in range(5):
        simple_state.imu.update_heading()
        readings.append(simple_state.imu.get_heading())
        time.sleep(0.05)
    current_heading = sum(readings) / len(readings)
   
    # Calcola errore rispetto al target
    error = current_heading - target_angle
   
    # Normalizza (-180 a +180)
    if error > 180:
        error -= 360
    elif error < -180:
        error += 360
   
    # Se errore piccolo, non fare nulla
    if abs(error) < 4.0:  # Aumentato a 5° per evitare micro-correzioni inutili
        print(f"✓ Robot già allineato a {simple_state.heading}: {current_heading:.1f}° (target: {target_angle:.1f}°, errore: {error:+.1f}°)")
        return
   
    # Correggi con rotazione closed-loop usando IMU
    print(f"🔧 Riallineamento a {simple_state.heading} ({target_angle:.1f}°): errore {error:+.1f}° - Correzione IMU...")
   
    # Determina direzione
    # error = current - target (normalizzato -180 a +180)
    # Se error < 0: current è "prima" del target → ruota RIGHT (verso target)
    # Se error > 0: current è "dopo" il target → ruota LEFT (verso target)
    if error < 0:
        direction = 'right'  # Diminuisce angolo verso target
    else:
        direction = 'left'   # Aumenta angolo verso target
   
    # Usa feedback IMU per correzione precisa
    speed_left, speed_right = _turn_speeds(direction)
    rover.rotate_differential_compensated(direction, speed_left, speed_right)
   
    start_time = time.time()
    timeout = 3.0  # Timeout per sicurezza
   
    try:
        while time.time() - start_time < timeout:
            simple_state.imu.update_heading()
            current_heading = simple_state.imu.get_heading()
           
            # Ricalcola errore rispetto al target
            error_now = current_heading - target_angle
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
    final_error = final_heading - target_angle
    if final_error > 180:
        final_error -= 360
    elif final_error < -180:
        final_error += 360
   
    print(f"✓ Riallineamento completato: {final_heading:.1f}° (target: {target_angle:.1f}°, errore residuo: {final_error:+.1f}°)")
 
 
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
    Include correzione heading CONTINUA durante il movimento (senza fermarsi).
   
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
   
    correction_status = "🔧 correzione continua" if simple_state.use_imu_heading_correction else "standard"
    print(f"➡️  Movimento {simple_state.heading}: {meters:.2f}m ({correction_status})")
   
    # Esegui movimento continuo con correzione integrata
    if not _move_segment_imu(meters, check_obstacles):
        return False  # Ostacolo rilevato
   
    print(f"\n✓ Completato: {meters:.2f}m")
   
    # Aggiorna posizione finale
    distance_traveled_imu = meters * simple_state.imu_odometry_scale
   
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
        simple_state.update_position(distance_traveled_imu)
        print(f"   Posizione: ({simple_state.position_x:.2f}, {simple_state.position_y:.2f})m")
        print(f"   Distanza da start: {simple_state.get_distance_from_start():.2f}m / {simple_state.target_distance:.2f}m")
   
    return True
 
 
def _move_segment_imu(meters, check_obstacles=True):
    """
    Muove un singolo segmento usando IMU per odometria.
    CORREZIONE CONTINUA: Aggiusta heading in tempo reale regolando velocità motori.
    Ritorna True se completato, False se ostacolo.
    """
    if not simple_state.imu.is_available():
        # Fallback senza IMU
        time_needed = meters / simple_state.meters_per_second_forward
        rover.moveTo('Forward', DEFAULT_SPEED_LINEAR * simple_state.left_factor,
                     DEFAULT_SPEED_LINEAR * simple_state.right_factor)
       
        start = time.time()
        while time.time() - start < time_needed:
            if check_obstacles:
                dist = _read_distance()
                if dist < simple_state.obstacle_threshold:
                    rover.stop()
                    return False
            time.sleep(0.1)
       
        rover.stop()
        return True
   
    # Velocità motori base
    base_speed_left = DEFAULT_SPEED_LINEAR * simple_state.left_factor
    base_speed_right = DEFAULT_SPEED_LINEAR * simple_state.right_factor
   
    # Reset odometria IMU
    distance_traveled_imu = 0.0
    velocity_x = 0.0
    velocity_y = 0.0
   
    # Heading iniziale per correzione RELATIVA (misura solo deriva durante questo movimento)
    # ✅ Non usa più heading assoluto che accumula drift, ma solo la deviazione rispetto all'inizio
    use_heading_correction = simple_state.use_imu_heading_correction and simple_state.imu.is_available()
    initial_heading = None
   
    if use_heading_correction:
        # Salva heading di partenza - misureremo solo la deriva rispetto a questo
        simple_state.imu.update_heading()
        initial_heading = simple_state.imu.get_heading()
        print(f"   🔧 Correzione RELATIVA attiva: heading_start={initial_heading:.1f}° (misura solo deriva durante movimento)")
    else:
        print(f"   ⚠️  Correzione heading disabilitata")
   
    # Inizia movimento
    rover.moveTo('Forward', base_speed_left, base_speed_right)
   
    start_time = time.time()
    last_time = start_time
    last_check = 0.0
    last_correction = 0.0
    last_debug_print = 0.0  # Per stampare stato heading
    timeout = meters / simple_state.meters_per_second_forward * 8.0
   
    # Costante di filtraggio per velocità
    alpha_vel = 0.95
   
    # Guadagno correzione heading (quanto aggressivo)
    correction_gain = getattr(simple_state, 'imu_heading_correction_gain', 0.25)
   
    try:
        while time.time() - start_time < timeout:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
           
            if dt < 0.01:
                time.sleep(0.01)
                continue
           
            # Leggi accelerazioni per odometria
            ax, ay = simple_state.imu.get_accel_xy()
           
            # Dead-zone per rumore accelerometro
            ACCEL_THRESHOLD = 0.05
            if abs(ax) < ACCEL_THRESHOLD:
                ax = 0.0
            if abs(ay) < ACCEL_THRESHOLD:
                ay = 0.0
           
            # Filtra e integra velocità
            velocity_x = alpha_vel * velocity_x + (1 - alpha_vel) * (ax * 9.81 * dt)
            velocity_y = alpha_vel * velocity_y + (1 - alpha_vel) * (ay * 9.81 * dt)
           
            # Calcola modulo velocità
            velocity = math.sqrt(velocity_x**2 + velocity_y**2)
           
            # Integra velocità → distanza
            distance_traveled_imu += velocity * dt
           
            # ===== CORREZIONE HEADING CONTINUA =====
            # Ogni 100ms controlla heading e aggiusta velocità motori
            if use_heading_correction and (current_time - last_correction) > 0.10:
                simple_state.imu.update_heading()
                current_heading = simple_state.imu.get_heading()
               
                # Calcola DERIVA RELATIVA: quanto sto deviando rispetto all'inizio del movimento
                # ✅ Funziona anche se heading assoluto è impreciso per drift accumulato
                error = current_heading - initial_heading
                if error > 180:
                    error -= 360
                elif error < -180:
                    error += 360
               
                # Debug periodico: stampa heading ogni secondo
                if (current_time - last_debug_print) > 1.0:
                    print(f"\n   🧭 Deriva: {error:+.1f}° (start: {initial_heading:.1f}°, now: {current_heading:.1f}°)", end='')
                    last_debug_print = current_time
               
                # 🔧 CORREZIONE per compensare deriva
                # Soglia minima per evitare micro-oscillazioni
                if abs(error) > 0.5:
                    # Calcola correzione proporzionale
                    correction = correction_gain * error
                   
                    # Applica correzione: limita a ±25% velocità base
                    correction = max(-0.25, min(0.25, correction))
                   
                    # Se error > 0 (deriva destra): aumenta left, riduci right
                    # Se error < 0 (deriva sinistra): riduci left, aumenta right
                    speed_left = base_speed_left * (1 + correction)
                    speed_right = base_speed_right * (1 - correction)
                   
                    # Limita velocità [0.3, 1.0]
                    speed_left = max(0.30, min(1.0, speed_left))
                    speed_right = max(0.30, min(1.0, speed_right))
                   
                    # Debug per ogni correzione
                    if getattr(simple_state, 'imu_heading_correction_debug', False):
                        print(f" → CORR: L:{speed_left:.2f} R:{speed_right:.2f}", end='')
                   
                    # Applica nuove velocità senza fermarsi
                    rover.moveTo('Forward', speed_left, speed_right)
                else:
                    # Errore minimo (<0.5°) → ritorna a velocità base bilanciate
                    rover.moveTo('Forward', base_speed_left, base_speed_right)
                   
                last_correction = current_time
           
            # Controlla ostacoli
            elapsed = time.time() - start_time
            if check_obstacles and (elapsed - last_check) > 0.10:  # 100ms - più frequente!
                distance = _read_distance()
                print(f"   📡 {distance:.1f}cm | IMU: {distance_traveled_imu:.2f}m/{meters:.2f}m", end='\r')
                last_check = elapsed
               
                if distance < simple_state.obstacle_threshold:
                    print(f"\n🚧 OSTACOLO rilevato a {distance:.1f}cm!")
                    # Se MOLTO vicino, indietreggia un po' per sicurezza
                    if distance < 20.0:
                        print(f"   ⚠️  Troppo vicino! Piccolo backup...")
                        rover.moveTo('Back', DEFAULT_SPEED_LINEAR * 0.5, DEFAULT_SPEED_LINEAR * 0.5)
                        time.sleep(0.3)
                        rover.stop()
                        time.sleep(0.1)
                    return False
           
            # Controlla se target raggiunto
            if distance_traveled_imu >= meters:
                break
           
            time.sleep(0.02)
    finally:
        rover.stop()
        time.sleep(0.15)  # Aumentato per compensare inerzia
   
    return True
 
 
def _correct_heading_during_movement():
    """
    Correzione heading durante movimento dritto.
    Più affidabile che dopo rotazioni (motori stabili, no torsioni).
    """
    if not simple_state.imu.is_available():
        return
   
    reference_heading = getattr(simple_state, 'imu_reference_heading', None)
    if reference_heading is None:
        return
   
    # Mappa heading corrente all'angolo target
    heading_map = {'N': 0, 'W': 90, 'S': 180, 'E': 270}
    if simple_state.heading not in heading_map:
        return
   
    target_angle = (reference_heading + heading_map[simple_state.heading]) % 360
   
    # Leggi heading con stabilizzazione breve
    time.sleep(0.15)
    readings = []
    for _ in range(3):
        simple_state.imu.update_heading()
        readings.append(simple_state.imu.get_heading())
        time.sleep(0.03)
    current_heading = sum(readings) / len(readings)
   
    # Calcola errore
    error = current_heading - target_angle
    if error > 180:
        error -= 360
    elif error < -180:
        error += 360
   
    # Soglia più permissiva durante movimento
    if abs(error) < 4.0:
        return
   
    # Correzione rapida
    print(f"\n   🔧 Correzione heading: {error:+.1f}°...", end="")
   
    direction = 'right' if error < 0 else 'left'
    speed_left, speed_right = _turn_speeds(direction)
    rover.rotate_differential_compensated(direction, speed_left, speed_right)
   
    start_time = time.time()
    try:
        while time.time() - start_time < 1.0:
            simple_state.imu.update_heading()
            current_heading = simple_state.imu.get_heading()
           
            error_now = current_heading - target_angle
            if error_now > 180:
                error_now -= 360
            elif error_now < -180:
                error_now += 360
           
            if abs(error_now) <= 3.0:
                break
           
            time.sleep(0.02)
    finally:
        rover.stop()
        time.sleep(0.1)
   
    print(f" ✓")
 
 
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
 
        if check_obstacles and (elapsed - last_check) > 0.10:  # 100ms - più frequente!
 
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
 
 
 