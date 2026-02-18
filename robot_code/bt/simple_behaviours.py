 
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
from .actions import arm_up, arm_down, open_hand, close_hand
 
 
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
    Navigazione labirinto con priorità fissa: N > W > E > S
    
    Strategia:
    1. Controlla sensore davanti (direzione corrente)
    2. Se libero (>threshold) → muovi avanti 0.5m
    3. Se ostacolo rilevato (≤threshold) → FERMA e controlla priorità
    4. Controlla direzioni IN ORDINE finché trova la prima libera:
       - NORD (uscita maze) 
       - OVEST
       - EST
       - SUD
    5. Prende la PRIMA direzione libera trovata (non controlla le altre)
    6. Ruota e continua in quella direzione
    
    Il robot NON colpisce i muri - si ferma quando il sensore rileva ostacolo.
    """

    def __init__(self, name="MazeNavigator"):
        super().__init__(name)
        self.iteration = 0
        self.stuck_count = 0
        self.max_stuck_attempts = 3
        self.forward_step = 0.5  # metri - step quando muovi avanti

    def initialise(self):
        self.iteration = 0
        self.stuck_count = 0
        print("\n" + "="*60)
        print("🧭 MAZE NAVIGATION - Priority: N > W > E > S")
        print("="*60)
        print(f"Target: {simple_state.target_distance:.2f}m dal punto iniziale")
        print(f"Strategia: Avanza fino a sensore rileva muro ({simple_state.obstacle_threshold:.0f}cm)")
        print(f"           Quando bloccato controlla: N → W → E → S")
        print(f"           Prima direzione LIBERA = scelta finale ✓")
        print(f"Soglia ostacolo: {simple_state.obstacle_threshold:.0f}cm")
        print("="*60 + "\n")

    def update(self):
        # Check successo
        if simple_state.is_target_reached():
            print("\n" + "="*60)
            print("✅ TARGET RAGGIUNTO!")
            print(f"Distanza percorsa: {simple_state.total_distance:.2f}m")
            print(f"Posizione finale: ({simple_state.position_x:.2f}, {simple_state.position_y:.2f})m")
            print("="*60)
            return py_trees.common.Status.SUCCESS

        self.iteration += 1
        distance_from_start = simple_state.get_distance_from_start()
        remaining = simple_state.target_distance - distance_from_start
        
        print(f"\n{'='*60}")
        print(f"🔄 Iterazione {self.iteration}")
        print(f"📍 Posizione: ({simple_state.position_x:.2f}, {simple_state.position_y:.2f})m")
        print(f"🧭 Heading: {simple_state.heading}")
        print(f"📏 Distanza da start: {distance_from_start:.2f}m / {simple_state.target_distance:.2f}m")
        print(f"{'='*60}")

        # Passo 1: Controlla sensore nella direzione corrente
        time.sleep(0.2)
        obstacle_ahead, dist_ahead = check_obstacle_ahead(threshold=simple_state.obstacle_threshold)
        
        if not obstacle_ahead:
            # Sensore indica strada libera → muovi avanti (con check continuo durante movimento)
            print(f"🟢 SENSORE LIBERO ({dist_ahead:.1f}cm) - Avanzo in direzione {simple_state.heading}...")
            step = min(self.forward_step, remaining)
            
            # check_obstacles=True garantisce che si fermi se rileva ostacolo DURANTE movimento
            if _move_forward(step, check_obstacles=True):
                self.stuck_count = 0
                return py_trees.common.Status.RUNNING
            else:
                # Ostacolo rilevato DURANTE movimento (sensore ha rilevato muro mentre avanzava)
                print("⚠️  Sensore ha rilevato ostacolo durante movimento - Mi fermo")
                # Continua con scansione direzioni
        
        # Passo 2: Ostacolo rilevato davanti → il robot si è fermato PRIMA del muro
        # Ora controlla direzioni in ordine di priorità: N > W > E > S
        # Prende la PRIMA direzione libera (non controlla tutte)
        print(f"\n🔴 OSTACOLO RILEVATO ({dist_ahead:.1f}cm) - Controllo priorità...")
        
        priority_directions = [
            ('N', "NORD (uscita)"),
            ('W', "OVEST"),
            ('E', "EST"),
            ('S', "SUD")
        ]
        
        chosen_direction = None
        chosen_label = None
        
        # Controlla direzioni in ordine finché non ne trova una libera
        for direction, label in priority_directions:
            # Ruota verso direzione da controllare
            rotate_to_heading(direction)
            time.sleep(0.15)
            
            # Leggi sensore
            blocked, distance = check_obstacle_ahead(threshold=simple_state.obstacle_threshold)
            
            status = "✅ LIBERO" if not blocked else "🚧 BLOCCATO"
            print(f"   {direction} ({label:12s}): {distance:.1f}cm - {status}")
            
            if not blocked:
                # Prima direzione libera trovata → scegliamo questa
                chosen_direction = direction
                chosen_label = label
                print(f"   → Prima direzione libera trovata: {direction} ✓")
                break
            # Altrimenti continua a controllare prossima priorità
        
        # Se tutte bloccate, dichiara stuck
        if chosen_direction is None:
            self.stuck_count += 1
            print(f"\n❌ TUTTE LE DIREZIONI BLOCCATE! (tentativo {self.stuck_count}/{self.max_stuck_attempts})")
            
            if self.stuck_count >= self.max_stuck_attempts:
                print("\n💀 Robot bloccato senza vie d'uscita - Missione fallita")
                return py_trees.common.Status.FAILURE
            
            # Prova a indietreggiare
            print("   → Provo a indietreggiare...")
            move_back_meters(0.3)
            return py_trees.common.Status.RUNNING
        
        # Reset stuck counter
        self.stuck_count = 0
        
        # Robot già orientato verso direzione scelta (dall'ultimo rotate_to_heading nel loop)
        print(f"\n➡️  Direzione scelta: {chosen_direction} ({chosen_label})")
        
        # Muovi nella direzione scelta
        step = self.forward_step
        print(f"🚶 Muovo {step:.2f}m verso {chosen_direction}...")
        
        if _move_forward(step, check_obstacles=True):
            print(f"✅ Step completato")
        else:
            print(f"⚠️  Sensore ha rilevato ostacolo durante movimento")
        
        return py_trees.common.Status.RUNNING


# =============================================================================
# GRAB OBJECT
# =============================================================================
 
class GrabObject(py_trees.behaviour.Behaviour):
    def __init__(self, name="GrabObject"):
        super().__init__(name)
        self.start_time = None
        self.step = 0
 
    def initialise(self):
        self.start_time = time.time()
        self.step = 0
        print("\n" + "=" * 60)
        print("🤖 GRAB OGGETTO - Sequenza braccio robotico")
        print("=" * 60)
 
    def update(self):
        elapsed = time.time() - self.start_time
 
        if elapsed < 1.0 and self.step == 0:
            print("   1️⃣ Apertura pinza...")
            open_hand(1000)
            self.step = 1
 
        elif 1.0 <= elapsed < 2.5 and self.step == 1:
            print("   2️⃣ Abbassamento braccio...")
            arm_down()
            time.sleep(1.5)
            self.step = 2
 
        elif 2.5 <= elapsed < 4.0 and self.step == 2:
            print("   3️⃣ Chiusura pinza...")
            close_hand(1500)
            self.step = 3
 
        elif 4.0 <= elapsed < 5.5 and self.step == 3:
            print("   4️⃣ Sollevamento braccio...")
            arm_up()
            time.sleep(1.5)
            self.step = 4
 
        elif elapsed >= 5.5:
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
        memory=False,
        children=[MazeNavigator(), GrabObject()],
    )
 
    return py_trees.trees.BehaviourTree(root)
 
 
 