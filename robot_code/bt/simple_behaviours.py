"""
Behavior Tree per Missione Lineare con Aggiramento Ostacoli
Sistema SENZA GRIGLIA - solo movimento lineare + aggiramento
 
Logica:
- Avanza a Nord a step.
- Se ostacolo:
    1) Scansione a ventaglio SOLO SINISTRA. Se trova un angolo libero, usa quello.
    2) Solo se SINISTRA non trova nulla, scansiona DESTRA.
- Aggiramento a step “micro”: mantiene l'angolo, ogni tot step torna a Nord per verificare.
- Quando Nord è libero in modo stabile: avanza un po' a Nord e poi prova a rientrare sulla linea.
- Rientro intelligente: prova direzione di rientro, se bloccata avanza un po' a Nord e riprova (senza ventaglio).
"""
import py_trees
import time
from .simple_actions import (
    rotate_to_heading,
    rotate_to_micro_angle,
    rotate_back_from_micro_angle,
    rotate_small,
    scan_fan_one_side,
    move_forward_meters,
    check_obstacle_ahead,
)
from .simple_state import simple_state
from .actions import arm_up, arm_down, open_hand, close_hand
 
 
class NavigateToTarget(py_trees.behaviour.Behaviour):
    def __init__(self, name="NavigateToTarget"):
        super().__init__(name)
        self.iteration = 0
        self.consecutive_bypass_failures = 0  # Contatore per fallimenti consecutivi
 
        # step di avanzamento
        self.forward_step = 0.5
 
        # bypass params
        self.lateral_step = simple_state.lateral_step
        self.max_lateral_steps = 10
 
        # micro scan params
        self.micro_step_deg = getattr(simple_state, "micro_turn_degrees", 30)
        self.max_scan_angle = 90
        self.scan_threshold = getattr(simple_state, "scan_threshold", 50.0)
 
    def initialise(self):
        self.iteration = 0
        self.consecutive_bypass_failures = 0
 
    def update(self):
        if simple_state.distance_traveled >= simple_state.target_distance:
            print(f"\n✅ TARGET RAGGIUNTO! ({simple_state.distance_traveled:.2f}m)")
            return py_trees.common.Status.SUCCESS
 
        self.iteration += 1
        remaining = simple_state.target_distance - simple_state.distance_traveled
        print(f"\n🔄 Iterazione {self.iteration} - Mancano {remaining:.2f}m al target")
 
        # Step avanti verso Nord
        rotate_to_heading('N')
        step = min(self.forward_step, remaining)
        print(f"🚶 Step NORD {step:.2f}m")
 
        if move_forward_meters(step, check_obstacles=True):
            # Movimento riuscito - reset del contatore
            self.consecutive_bypass_failures = 0
            return py_trees.common.Status.RUNNING
 
        # Ostacolo
        print("\n🚧 OSTACOLO DAVANTI - Aggiramento rettangolare")
        rotate_to_heading('N')
 
        # Ignora la scansione - usa aggiramento rettangolare fisso
        rotate_to_heading('N')
        
        # Prova prima aggiramento a sinistra
        if self._bypass_rectangle(try_right=False):
            # Aggiramento riuscito - reset del contatore
            self.consecutive_bypass_failures = 0
            return py_trees.common.Status.RUNNING
        
        # Se sinistra fallisce perché sinistra E centro bloccati, prova destra
        print("   🔄 Tentativo aggiramento a DESTRA...")
        if self._bypass_rectangle(try_right=True):
            # Aggiramento riuscito - reset del contatore
            self.consecutive_bypass_failures = 0
            return py_trees.common.Status.RUNNING
 
        # Se fallisce anche destra, incrementa contatore
        self.consecutive_bypass_failures += 1
        print(f"   ❌ Entrambi i bypass falliti ({self.consecutive_bypass_failures} tentativi consecutivi).")
        
        # Se troppi fallimenti consecutivi, prova strategia alternativa
        if self.consecutive_bypass_failures >= 3:
            print("   ⚠️  Troppi fallimenti di aggiramento consecutivi!")
            print("   🔄 Strategia alternativa: indietreggio e riprovo...")
            from .simple_actions import move_back_meters
            rotate_to_heading('N')
            move_back_meters(0.30)
            self.consecutive_bypass_failures = 0  # Reset dopo strategia alternativa
        
        rotate_to_heading('N')
        return py_trees.common.Status.RUNNING
 
    def _bypass_rectangle(self, try_right=False):
        """
        Aggiramento a rettangolo con rotazioni fisse di 90° e tempi fissi:
        
        SINISTRA (default):
        1. Gira 90° SINISTRA → controlla → avanza lateralmente 5 secondi
        2. Gira 90° DESTRA → controlla → avanza 10 secondi per superare ostacolo  
        3. Gira 90° DESTRA → avanza 5 secondi per rientrare (stesso tempo laterale)
        4. Gira 90° SINISTRA → torna orientato a Nord e continua
        
        DESTRA (se try_right=True):
        1. Gira 90° DESTRA → controlla → avanza lateralmente 5 secondi
        2. Gira 90° SINISTRA → controlla → avanza 10 secondi per superare ostacolo  
        3. Gira 90° SINISTRA → avanza 5 secondi per rientrare (stesso tempo laterale)
        4. Gira 90° DESTRA → torna orientato a Nord e continua
        """
        lateral_time = 5.0  # Secondi per movimento laterale
        forward_time = 15.0  # Secondi per superare ostacolo
        
        if try_right:
            print("\n🔄 Aggiramento rettangolare (DESTRA):")
            
            # PASSO 1: Gira 90° DESTRA
            print("   1️⃣ Gira 90° DESTRA...")
            rotate_to_heading('E')  # Da Nord a Est
            
            time.sleep(0.2)
            blocked, dist = check_obstacle_ahead(threshold=simple_state.obstacle_threshold)
            print(f"   📡 Est: {dist:.1f}cm {'❌ BLOCCATO' if blocked else '✅ LIBERO'}")
            
            if blocked:
                print("   ❌ Est bloccato, non posso aggirare a destra")
                rotate_to_heading('N')
                return False
        else:
            print("\n🔄 Aggiramento rettangolare (SINISTRA):")
            
            # PASSO 1: Gira 90° SINISTRA
            print("   1️⃣ Gira 90° SINISTRA...")
            rotate_to_heading('W')  # Da Nord a Ovest
            
            time.sleep(0.2)
            blocked, dist = check_obstacle_ahead(threshold=simple_state.obstacle_threshold)
            print(f"   📡 Ovest: {dist:.1f}cm {'❌ BLOCCATO' if blocked else '✅ LIBERO'}")
            
            if blocked:
                print("   ❌ Ovest bloccato")
                # Controlla anche al centro prima di dichiarare fallimento totale
                rotate_to_heading('N')
                time.sleep(0.2)
                blocked_center, dist_center = check_obstacle_ahead(threshold=simple_state.obstacle_threshold)
                print(f"   📡 Centro (Nord): {dist_center:.1f}cm")
                
                if blocked_center:
                    print("   ⚠️  Sia SINISTRA che CENTRO bloccati → provo DESTRA")
                    return False
                else:
                    print("   ❌ Ovest bloccato, non posso aggirare a sinistra")
                    return False
        
        # Avanza lateralmente per tempo fisso
        if try_right:
            print(f"   ➡️  Avanza {lateral_time:.1f}s a EST (laterale)...")
        else:
            print(f"   ➡️  Avanza {lateral_time:.1f}s a OVEST (laterale)...")
        
        from .simple_actions import move_timed
        if not move_timed(lateral_time, check_obstacles=True):
            print("   ❌ Ostacolo durante movimento laterale")
            rotate_to_heading('N')
            return False
        
        # PASSO 2: Gira verso Nord
        if try_right:
            print("   2️⃣ Gira 90° SINISTRA...")
        else:
            print("   2️⃣ Gira 90° DESTRA...")
        rotate_to_heading('N')
        
        time.sleep(0.2)
        blocked, dist = check_obstacle_ahead(threshold=simple_state.obstacle_threshold)
        print(f"   📡 Nord: {dist:.1f}cm {'❌ BLOCCATO' if blocked else '✅ LIBERO'}")
        
        if blocked:
            print("   ❌ Nord ancora bloccato")
            rotate_to_heading('N')
            return False
        
        # Avanza a Nord per tempo fisso (supera ostacolo)
        print(f"   ➡️  Avanza {forward_time:.1f}s a NORD (supera ostacolo)...")
        if not move_timed(forward_time, check_obstacles=True):
            print("   ❌ Ostacolo durante superamento")
            rotate_to_heading('N')
            return False
        
        # PASSO 3: Gira per rientrare
        if try_right:
            print("   3️⃣ Gira 90° SINISTRA...")
            rotate_to_heading('W')
        else:
            print("   3️⃣ Gira 90° DESTRA...")
            rotate_to_heading('E')
        
        time.sleep(0.2)
        blocked, dist = check_obstacle_ahead(threshold=simple_state.obstacle_threshold)
        if try_right:
            print(f"   📡 Ovest: {dist:.1f}cm {'❌ BLOCCATO' if blocked else '✅ LIBERO'}")
        else:
            print(f"   📡 Est: {dist:.1f}cm {'❌ BLOCCATO' if blocked else '✅ LIBERO'}")
        
        if blocked:
            print("   ❌ Lato rientro bloccato")
            rotate_to_heading('N')
            return False
        
        # Rientra per stesso tempo laterale
        if try_right:
            print(f"   ↩️  Rientra {lateral_time:.1f}s a OVEST (torna in linea)...")
        else:
            print(f"   ↩️  Rientra {lateral_time:.1f}s a EST (torna in linea)...")
        
        if not move_timed(lateral_time, check_obstacles=True):
            print("   ❌ Ostacolo durante rientro")
            rotate_to_heading('N')
            return False
        
        # PASSO 4: Torna a Nord
        if try_right:
            print("   4️⃣ Gira 90° DESTRA...")
        else:
            print("   4️⃣ Gira 90° SINISTRA...")
        rotate_to_heading('N')
        
        time.sleep(0.2)
        blocked, dist = check_obstacle_ahead(threshold=simple_state.obstacle_threshold)
        print(f"   📡 Nord finale: {dist:.1f}cm {'❌ BLOCCATO' if blocked else '✅ LIBERO'}")
        
        if try_right:
            print("✅ Aggiramento rettangolare DESTRA completato!\n")
        else:
            print("✅ Aggiramento rettangolare SINISTRA completato!\n")
        return True
 
    def _bypass_with_micro_OLD(self, side, angle):
        """
        side: 'left' o 'right'
        angle: 0..90
        """
        lateral_progress = 0.0
 
        # sicurezza: prima di controllare N, fai un minimo laterale
        min_lateral_before_n_check = 0.60
 
        # Nord libero deve essere stabile
        north_clear_needed = 3
        north_clear_count = 0
 
        # Ruota all'angolo UNA VOLTA
        rotate_to_heading('N')
        rotate_to_micro_angle(side, angle, step=self.micro_step_deg)
 
        for i in range(self.max_lateral_steps):
            print(f"   ↔️  Step angolato {side} {angle}° : {self.lateral_step:.2f}m (#{i+1})")
 
            # Step avanti mantenendo l'angolo
            if not move_forward_meters(
                self.lateral_step,
                check_obstacles=True,
                micro_side=side,
                micro_angle_deg=angle
            ):
                print("   🚧 Ostacolo durante step angolato - aumento angolo")
                
                # Se l'angolo è già massimo, fallisci
                if angle >= self.max_scan_angle:
                    print("   ❌ Angolo già al massimo, aggiramento fallito")
                    rotate_back_from_micro_angle(side, angle, step=self.micro_step_deg)
                    rotate_to_heading('N')
                    return False
                
                # Altrimenti aumenta l'angolo e continua
                rotate_back_from_micro_angle(side, angle, step=self.micro_step_deg)
                rotate_to_heading('N')
                
                # Cerca un nuovo angolo più ampio
                new_angle = angle + self.micro_step_deg
                print(f"   🔄 Provo angolo maggiore: {new_angle}°")
                
                # Scansiona dal nuovo angolo in poi
                rotate_to_micro_angle(side, new_angle, step=self.micro_step_deg)
                
                # Prova qualche angolo più ampio
                found_angle = None
                for test_offset in range(0, self.max_scan_angle - new_angle + 1, self.micro_step_deg):
                    test_angle = new_angle + test_offset
                    if test_offset > 0:
                        rotate_small(side, self.micro_step_deg)
                    
                    time.sleep(0.15)
                    blocked, dist = check_obstacle_ahead(threshold=self.scan_threshold)
                    print(f"      Test {side} {test_angle}°: {dist:.1f}cm {'❌' if blocked else '✅'}")
                    
                    if not blocked:
                        found_angle = test_angle
                        break
                    
                    if test_angle >= self.max_scan_angle:
                        break
                
                # Torna a N
                if found_angle:
                    current_angle = new_angle if test_offset == 0 else new_angle + test_offset
                    rotate_back_from_micro_angle(side, current_angle, step=self.micro_step_deg)
                else:
                    # Torna indietro dal punto di test
                    current_test = new_angle + (test_offset if 'test_offset' in locals() else 0)
                    rotate_back_from_micro_angle(side, current_test, step=self.micro_step_deg)
                
                rotate_to_heading('N')
                
                if found_angle is None:
                    print("   ❌ Nessun angolo maggiore trovato libero")
                    return False
                
                # Aggiorna angolo e riparti
                angle = found_angle
                print(f"   ✅ Nuovo angolo di aggiramento: {angle}°")
                rotate_to_micro_angle(side, angle, step=self.micro_step_deg)
                continue
 
            lateral_progress += self.lateral_step
 
            # Non controllare rientro troppo presto
            if lateral_progress < min_lateral_before_n_check:
                continue
 
            # Controlla la direzione OPPOSTA (simmetrica) per il rientro
            rotate_back_from_micro_angle(side, angle, step=self.micro_step_deg)
            rotate_to_heading('N')
            
            opposite_side = 'right' if side == 'left' else 'left'
            print(f"   🔍 Check rientro simmetrico: {opposite_side} {angle}°...")
            
            # Ruota nella direzione opposta per controllare
            rotate_to_micro_angle(opposite_side, angle, step=self.micro_step_deg)
            time.sleep(0.15)
            blocked, dist = check_obstacle_ahead(threshold=self.scan_threshold)
            print(f"      {opposite_side} {angle}°: {dist:.1f}cm {'❌' if blocked else '✅'}")
            
            # Torna a Nord
            rotate_back_from_micro_angle(opposite_side, angle, step=self.micro_step_deg)
            rotate_to_heading('N')
            
            if not blocked:
                north_clear_count += 1
                print(f"   ✅ Direzione rientro libera ({north_clear_count}/{north_clear_needed})")
                
                # Se trovata libera, fare controlli ravvicinati SENZA allontanarsi ulteriormente
                if north_clear_count < north_clear_needed:
                    print(f"   ⏸️  Pausa per verifica stabilità (aspetto 1s)...")
                    time.sleep(1.0)
                    # Ricontrolla subito senza muoversi
                    rotate_to_micro_angle(opposite_side, angle, step=self.micro_step_deg)
                    time.sleep(0.15)
                    blocked_recheck, dist_recheck = check_obstacle_ahead(threshold=self.scan_threshold)
                    print(f"      Ricontrollo {opposite_side} {angle}°: {dist_recheck:.1f}cm {'❌' if blocked_recheck else '✅'}")
                    rotate_back_from_micro_angle(opposite_side, angle, step=self.micro_step_deg)
                    rotate_to_heading('N')
                    
                    if blocked_recheck:
                        # Era un falso positivo
                        north_clear_count = 0
                        print(f"   ❌ Ricontrollo fallito, era temporaneo - continuo aggiramento")
                        rotate_to_micro_angle(side, angle, step=self.micro_step_deg)
                        continue
                    else:
                        # Ancora libero - non allontanarsi, ricontrolla al prossimo ciclo
                        continue
            else:
                north_clear_count = 0
                # Torna all'angolo di aggiramento e continua ad allontanarsi
                rotate_to_micro_angle(side, angle, step=self.micro_step_deg)
                continue
 
            # Direzione opposta davvero libera → RIENTRO SIMMETRICO
            print(f"   ✅ Rientro simmetrico {opposite_side} {angle}° per ~{lateral_progress:.2f}m")
            
            # Ruota nella direzione di rientro
            rotate_to_micro_angle(opposite_side, angle, step=self.micro_step_deg)
            
            # Rientra percorrendo circa la stessa distanza laterale fatta
            rientro_distance = abs(simple_state.lateral_offset)
            steps_rientro = int(round(rientro_distance / self.lateral_step))
            if steps_rientro < 1:
                steps_rientro = int(round(lateral_progress / self.lateral_step))
            
            print(f"   ↩️  Rientro in {steps_rientro} step da {self.lateral_step:.2f}m")
            
            rientro_success = True
            for step_num in range(steps_rientro):
                if not move_forward_meters(
                    self.lateral_step,
                    check_obstacles=True,
                    micro_side=opposite_side,
                    micro_angle_deg=angle
                ):
                    print(f"   🚧 Ostacolo durante rientro (step {step_num+1}/{steps_rientro})")
                    rientro_success = False
                    break
            
            # Torna a Nord
            rotate_back_from_micro_angle(opposite_side, angle, step=self.micro_step_deg)
            rotate_to_heading('N')
            
            if rientro_success:
                print("✅ Aggiramento e rientro simmetrico completati!")
                return True
            else:
                # Rientro bloccato, prova ad avanzare un po' a Nord
                print("   ⚠️  Rientro parziale, avanzo un po' a Nord...")
                if move_forward_meters(0.50, check_obstacles=True):
                    print("✅ Aggiramento completato (rientro parziale)")
                    return True
                else:
                    print("   ❌ Anche Nord bloccato, riprovo aggiramento")
                    return False
 
        # Troppi step -> fallito
        rotate_back_from_micro_angle(side, angle, step=self.micro_step_deg)
        rotate_to_heading('N')
        print("⚠️  Ostacolo troppo lungo: micro bypass fallito")
        return False
 
 
class GrabObject(py_trees.behaviour.Behaviour):
    def __init__(self, name="GrabObject"):
        super().__init__(name)
        self.start_time = None
        self.step = 0
 
    def initialise(self):
        self.step = 0
        self.start_time = time.time()
        print("\n" + "="*60)
        print("🤖 GRAB OGGETTO - Sequenza braccio robotico")
        print("="*60)
 
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
            print("="*60)
            return py_trees.common.Status.SUCCESS
 
        return py_trees.common.Status.RUNNING
 
 
def create_main_mission_tree(target_distance=2.4):
    simple_state.target_distance = target_distance
    root = py_trees.composites.Sequence(name="MainMission", memory=True)
    root.add_children([NavigateToTarget(), GrabObject()])
    return root
 
 