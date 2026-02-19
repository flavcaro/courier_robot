 
"""
Behavior Tree per Navigazione Labirinto
Movimento continuo con priorità fissa: N > W > E > S
 
Features:
- Muove avanti fino a rilevamento ostacolo (NON colpisce muri)
- Sensore controlla continuamente durante movimento
- Quando ostacolo rilevato, controlla priorità finché trova prima libera
- Priorità fissa: Nord > Ovest > Est > Sud
- Prima direzione libera = direzione scelta (non controlla altre)
- Supporto IMU per movimenti precisi
"""
 
import time
import py_trees
 
from .simple_state import simple_state
from .simple_actions import (
    rotate_to_heading,
    move_forward_meters,
    move_forward_meters_with_imu,
    check_obstacle_ahead,
    move_back_meters,
    realign_to_north_with_imu,
    scan_all_directions_maze,
)
from .actions import arm_up, arm_down, open_hand, close_hand, rover
 
 
# =============================================================================
# Helper movimento: IMU odometry se abilitata, altrimenti tempo
# =============================================================================
 
def _move_forward(meters, check_obstacles=True, micro_side=None, micro_angle_deg=0):
    """
    Wrapper che usa IMU se disponibile e abilitato, altrimenti fallback a tempo.
    """
    if simple_state.imu.is_available() and getattr(simple_state, "use_imu_odometry", False):
        return move_forward_meters_with_imu(meters, check_obstacles, micro_side, micro_angle_deg)
    return move_forward_meters(meters, check_obstacles, micro_side, micro_angle_deg)
 
 
def _rotate_to_north_and_realign():
    """
    Gira verso Nord usando la bussola e riallinea con IMU al Nord di riferimento.
    """
    rotate_to_heading("N")
    realign_to_north_with_imu()
 
 
# =============================================================================
# NAVIGAZIONE LABIRINTO (WALL-FOLLOWING)
# =============================================================================

def _get_left_direction(current_heading):
    """Ritorna la direzione a sinistra di current_heading"""
    directions = ['N', 'E', 'S', 'W']
    idx = directions.index(current_heading)
    return directions[(idx - 1) % 4]  # -1 = sinistra (antiorario)

def _get_right_direction(current_heading):
    """Ritorna la direzione a destra di current_heading"""
    directions = ['N', 'E', 'S', 'W']
    idx = directions.index(current_heading)
    return directions[(idx + 1) % 4]  # +1 = destra (orario)

def _get_back_direction(current_heading):
    """Ritorna la direzione opposta a current_heading"""
    directions = ['N', 'E', 'S', 'W']
    idx = directions.index(current_heading)
    return directions[(idx + 2) % 4]  # +2 = dietro


class MazeNavigator(py_trees.behaviour.Behaviour):
    """
    Navigazione labirinto con WALL-FOLLOWING EFFICIENTE (risparmio batteria).
    
    Strategia LEFT-HAND RULE (o RIGHT-HAND - configurabile):
    1. Controlla sensore davanti (direzione corrente)
    2. Se libero (>threshold) → muovi avanti step
    3. Se ostacolo rilevato (≤threshold) → FERMA e scansione EFFICIENTE:
       ⚡ Controlla SOLO le laterali (non avanti/indietro inutilmente)
       ⚡ Prima laterale libera → VAI (stop scansione)
       ⚡ Entrambe bloccate → Gira 180° indietro
    4. Left-hand: Sinistra > Destra > Indietro
       Right-hand: Destra > Sinistra > Indietro
    5. Il robot segue il percorso del labirinto tenendo la mano sul muro
    
    Il robot NON colpisce i muri - si ferma quando il sensore rileva ostacolo.
    """

    def __init__(self, name="MazeNavigator"):
        super().__init__(name)
        self.iteration = 0
        self.forward_step = 0.5  # metri - step quando muovi avanti

    def initialise(self):
        self.iteration = 0
        rule_name = "Left-Hand Rule" if simple_state.wall_following_rule == 'left' else "Right-Hand Rule"
        print("\n" + "="*60)
        print(f"🧭 MAZE NAVIGATION - Wall-Following ({rule_name})")
        print("="*60)
        print(f"Target: {simple_state.target_distance:.2f}m dal punto iniziale")
        print(f"Strategia: Priorità RELATIVA (⚡ EFFICIENTE - risparmio batteria)")
        print(f"           1️⃣ Controlla SOLO laterali (non avanti già bloccato)")
        print(f"           2️⃣ {'Sinistra' if simple_state.wall_following_rule == 'left' else 'Destra'} (priorità 1 - prima libera = vai!)")
        print(f"           3️⃣ {'Destra' if simple_state.wall_following_rule == 'left' else 'Sinistra'} (priorità 2)")
        print(f"           4️⃣ Indietro (se entrambe laterali bloccate)")
        print(f"Soglia ostacolo: {simple_state.obstacle_threshold:.0f}cm")
        print("="*60 + "\n")

    def update(self):
        # Check successo (con tolleranza per arrotondamenti)
        distance_from_start = simple_state.get_distance_from_start()
        remaining = simple_state.target_distance - distance_from_start
        
        if remaining <= 0.05:  # Tolleranza 5cm per arrotondamenti e precisione IMU
            print("\n" + "="*60)
            print("✅ TARGET RAGGIUNTO!")
            print(f"Distanza da start: {distance_from_start:.2f}m / {simple_state.target_distance:.2f}m")
            print(f"Distanza percorsa totale: {simple_state.total_distance:.2f}m")
            print(f"Posizione finale: ({simple_state.position_x:.2f}, {simple_state.position_y:.2f})m")
            print("="*60)
            return py_trees.common.Status.SUCCESS

        self.iteration += 1
        
        print(f"\n{'='*60}")
        print(f"🔄 Iterazione {self.iteration}")
        print(f"📍 Posizione: ({simple_state.position_x:.2f}, {simple_state.position_y:.2f})m")
        print(f"🧭 Heading: {simple_state.heading}")
        print(f"📏 Distanza da start: {distance_from_start:.2f}m / {simple_state.target_distance:.2f}m")
        print(f"{'='*60}")

        # Passo 1: Controlla sensore nella direzione corrente
        time.sleep(0.2)
        obstacle_ahead, dist_ahead = check_obstacle_ahead(threshold=simple_state.obstacle_threshold)
        
        # 🎯 EARLY OBJECT DETECTION: Se siamo vicini al target E vediamo un oggetto vicino
        # → dichiara successo anche se non siamo esattamente a 3m
        OBJECT_DETECTION_DISTANCE = 60.0  # cm - considera "oggetto vicino"
        APPROACH_THRESHOLD = 0.5  # m - inizia a cercare oggetto quando manca <0.5m al target
        
        if remaining <= APPROACH_THRESHOLD and dist_ahead <= OBJECT_DETECTION_DISTANCE:
            print(f"\n🎯 OGGETTO RILEVATO VICINO AL TARGET!")
            print(f"   📍 Distanza da start: {distance_from_start:.2f}m / {simple_state.target_distance:.2f}m")
            print(f"   📡 Oggetto a {dist_ahead:.1f}cm davanti")
            print(f"   ✅ TARGET RAGGIUNTO (early detection)")
            print("="*60)
            return py_trees.common.Status.SUCCESS
        
        if not obstacle_ahead:
            # Sensore indica strada libera → muovi avanti (con check continuo durante movimento)
            print(f"🟢 SENSORE LIBERO ({dist_ahead:.1f}cm) - Avanzo in direzione {simple_state.heading}...")
            
            # Calcola quanto manca al target
            step = min(self.forward_step, remaining)
            
            # 🎯 Se siamo già al target o manca meno di 1cm, dichiara successo
            if remaining <= 0.01:  # 1cm tolleranza
                print(f"\n🎯 Distanza rimanente: {remaining*100:.1f}cm - TARGET RAGGIUNTO!")
                return py_trees.common.Status.SUCCESS
            
            # check_obstacles=True garantisce che si fermi se rileva ostacolo DURANTE movimento
            if _move_forward(step, check_obstacles=True):
                self.stuck_count = 0
                return py_trees.common.Status.RUNNING
            else:
                # Ostacolo rilevato DURANTE movimento (sensore ha rilevato muro mentre avanzava)
                print("⚠️  Sensore ha rilevato ostacolo durante movimento - Mi fermo")
                # Continua con scansione direzioni
        
        # Passo 2: Ostacolo rilevato davanti → il robot si è fermato PRIMA del muro
        # Wall-following con priorità relativa alla direzione corrente
        print(f"\n🔴 OSTACOLO RILEVATO ({dist_ahead:.1f}cm) - Wall-following...")
        print(f"   📍 Posizione attuale: {simple_state.heading} ({simple_state.position_x:.2f}, {simple_state.position_y:.2f})m")
        
        # DISABILITATO: Backup automatico non necessario - la soglia configurabile gestisce la distanza
        # Se troppo vicino, aumenta obstacle_threshold in simple_state.py invece di indietreggiare
        # SAFE_DISTANCE_FOR_ROTATION = 25.0
        # if dist_ahead < SAFE_DISTANCE_FOR_ROTATION:
        #     backup_distance = 0.15
        #     print(f"   ⚠️  Troppo vicino ({dist_ahead:.1f}cm) - Indietreggio {backup_distance*100:.0f}cm...")
        #     move_back_meters(backup_distance)
        #     time.sleep(0.3)
        
        # 🧠 WALL-FOLLOWING EFFICIENTE: Controlla SOLO le laterali (risparmia batteria)
        # Avanti è già bloccato (per questo siamo qui), non ri-controllarlo
        # Calcola direzioni relative
        current = simple_state.heading
        left_dir = _get_left_direction(current)
        right_dir = _get_right_direction(current)
        back_dir = _get_back_direction(current)
        
        # Costruisci ordine priorità SOLO per laterali
        if simple_state.wall_following_rule == 'left':
            # LEFT-HAND RULE: Sinistra > Destra
            lateral_order = [(left_dir, f"SINISTRA ({left_dir})"), (right_dir, f"DESTRA ({right_dir})")]
            print(f"   🤚 Left-Hand: Provo {left_dir} (sx) poi {right_dir} (dx)")
        else:
            # RIGHT-HAND RULE: Destra > Sinistra
            lateral_order = [(right_dir, f"DESTRA ({right_dir})"), (left_dir, f"SINISTRA ({left_dir})")]
            print(f"   🤚 Right-Hand: Provo {right_dir} (dx) poi {left_dir} (sx)")
        
        chosen_direction = None
        chosen_label = None
        
        # ⚡ SCANSIONE EFFICIENTE: Controlla SOLO le laterali, ferma alla prima libera
        for direction, label in lateral_order:
            # Ruota verso direzione da controllare
            rotate_to_heading(direction)
            time.sleep(0.15)
            
            # Leggi sensore con soglia PIÙ ALTA per decidere se è percorribile
            # ⚠️ IMPORTANTE: Serve spazio almeno per uno step (50cm) + margine sicurezza
            distance = rover.getUltrasonicSensor()
            blocked = distance < simple_state.direction_clearance_threshold  # 60cm invece di 15cm!
            
            status = "✅ LIBERO" if not blocked else "🚧 BLOCCATO"
            print(f"   {direction} ({label:15s}): {distance:.1f}cm - {status}")
            
            if not blocked:
                # Prima laterale libera → SCEGLI E FERMA (non controllare l'altra!)
                chosen_direction = direction
                chosen_label = label
                print(f"   → {direction} libera! Vado lì ✓")
                break  # ⚡ STOP: risparmio batteria!
        
        # Se entrambe le laterali sono bloccate → torna indietro (se possibile)
        if chosen_direction is None:
            print(f"   ⚠️  Entrambe le laterali bloccate → Controllo indietro...")
            rotate_to_heading(back_dir)
            time.sleep(0.15)
            
            distance_back = rover.getUltrasonicSensor()
            if distance_back >= simple_state.direction_clearance_threshold:
                chosen_direction = back_dir
                chosen_label = f"INDIETRO ({back_dir})"
                print(f"   {back_dir} (INDIETRO): {distance_back:.1f}cm - ✅ LIBERO")
            else:
                # COMPLETAMENTE BLOCCATO!
                print(f"   {back_dir} (INDIETRO): {distance_back:.1f}cm - 🚧 BLOCCATO")
                print(f"\n❌ ROBOT COMPLETAMENTE BLOCCATO - Tutte le direzioni chiuse!")
                self.stuck_count += 1
                if self.stuck_count >= self.max_stuck_attempts:
                    print("💀 Troppi tentativi falliti - Missione impossibile")
                    return py_trees.common.Status.FAILURE
                # Prova a indietreggiare un po'
                print("   → Provo piccolo backup...")
                move_back_meters(0.3)
                return py_trees.common.Status.RUNNING
        
        # Reset stuck counter
        self.stuck_count = 0
        # Altrimenti il robot è già orientato verso chosen_direction dall'ultimo rotate_to_heading
        
        print(f"\n➡️  Direzione scelta: {chosen_direction} ({chosen_label})")
        
        # Muovi nella direzione scelta
        step = self.forward_step
        print(f"🚶 Movimento: {step:.2f}m → {chosen_direction}")
        print(f"   📍 Da ({simple_state.position_x:.2f}, {simple_state.position_y:.2f})m...")
        
        if _move_forward(step, check_obstacles=True):
            print(f"   ✅ Arrivato a ({simple_state.position_x:.2f}, {simple_state.position_y:.2f})m")
        else:
            print(f"   ⚠️  Sensore ha rilevato ostacolo durante movimento")
        
        return py_trees.common.Status.RUNNING


# =============================================================================
# GRAB OBJECT
# =============================================================================
 
class GrabObject(py_trees.behaviour.Behaviour):
    def __init__(self, name="GrabObject"):
        super().__init__(name)
        self.start_time = None
        self.step = 0
        self.object_detected = False

    def initialise(self):
        # 🔍 CONTROLLO OGGETTO: Verifica se c'è un ostacolo davanti (= oggetto da prendere)
        print("\n" + "=" * 60)
        print("🔍 CONTROLLO PRESENZA OGGETTO")
        print("=" * 60)
        
        distance = rover.getUltrasonicSensor()
        print(f"📡 Sensore: {distance:.1f}cm")
        
        # Se ostacolo entro 40cm = oggetto presente
        if distance < 40.0:
            print(f"✅ OGGETTO RILEVATO a {distance:.1f}cm!")
            print("🤖 Inizio sequenza grab...")
            print("=" * 60)
            self.object_detected = True
            self.start_time = time.time()
            self.step = 0
        else:
            print(f"❌ NESSUN OGGETTO TROVATO (distanza: {distance:.1f}cm)")
            print("🔊 Emetto segnale acustico...")
            rover.beep()  # 3 beep di errore
            print("=" * 60)
            self.object_detected = False

    def update(self):
        # Se non c'è oggetto, termina con FAILURE
        if not self.object_detected:
            return py_trees.common.Status.FAILURE
        
        elapsed = time.time() - self.start_time

        if elapsed < 1.5 and self.step == 0:
            print("   1️⃣ Apertura pinza...")
            open_hand(2000)  # ⬆️ AUMENTATO a 2000ms per apertura completa
            self.step = 1

        elif 1.5 <= elapsed < 3.5 and self.step == 1:
            print("   2️⃣ Abbassamento braccio...")
            arm_down()
            self.step = 2

        elif 3.5 <= elapsed < 5.5 and self.step == 2:
            print("   3️⃣ Chiusura pinza...")
            close_hand(1750)
            self.step = 3

        elif 5.5 <= elapsed < 9.5 and self.step == 3:
            print("   4️⃣ Sollevamento braccio...")
            arm_up()  # ✅ Eseguito correttamente (3.5s wait interno)
            self.step = 4

        elif elapsed >= 9.5 and self.step == 4:
            print("✅ Oggetto raccolto!")
            print("=" * 60)
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING


# =============================================================================
# TREE FACTORY
# =============================================================================
 
def create_main_mission_tree(target_distance=2.4):
    """
    Crea Behavior Tree per navigazione labirinto.
 
    Struttura:
      MainMission (Sequence)
        - MazeNavigator (wall-following)
        - GrabObject
    """
    simple_state.target_distance = target_distance
 
    root = py_trees.composites.Sequence(
        name="MainMission",
        memory=True,  # ✅ CRITICAL: memory=True mantiene progresso tra i children!
        children=[MazeNavigator(), GrabObject()],
    )
 
    return py_trees.trees.BehaviourTree(root)