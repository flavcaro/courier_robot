"""
Behaviours estesi per navigazione con griglia - Integrato con behaviours.py esistente.
"""
import py_trees
from .navigation_actions import move_to_cell, bfs_path, normalize_angle
from .sensors import robot_state
from .actions import arm_up, arm_down, open_hand, close_hand
import time
import math


# ============================================================================
# NAVIGATION BEHAVIORS
# ============================================================================

class NavigateToCell(py_trees.behaviour.Behaviour):
    """Naviga verso una cella specifica della griglia."""
    
    def __init__(self, target_row, target_col, name="NavigateToCell"):
        super().__init__(name)
        self.target_row = target_row
        self.target_col = target_col
        
    def update(self):
        print(f"🧭 Navigazione verso cella ({self.target_row}, {self.target_col})")
        
        if move_to_cell(self.target_row, self.target_col):
            return py_trees.common.Status.SUCCESS
        else:
            print(f"❌ Impossibile raggiungere cella ({self.target_row}, {self.target_col})")
            return py_trees.common.Status.FAILURE


class FollowPath(py_trees.behaviour.Behaviour):
    """Segue un percorso di celle calcolato con BFS."""
    
    def __init__(self, path, name="FollowPath"):
        super().__init__(name)
        self.path = path
        self.current_index = 0
        
    def initialise(self):
        self.current_index = 0
        print(f"📍 Percorso: {self.path}")
        
    def update(self):
        if self.current_index >= len(self.path):
            print("✅ Percorso completato!")
            return py_trees.common.Status.SUCCESS
        
        target_row, target_col = self.path[self.current_index]
        print(f"🎯 Cella {self.current_index + 1}/{len(self.path)}: ({target_row}, {target_col})")
        
        if move_to_cell(target_row, target_col):
            self.current_index += 1
            return py_trees.common.Status.RUNNING
        else:
            print(f"❌ Fallimento navigazione verso ({target_row}, {target_col})")
            return py_trees.common.Status.FAILURE


class PlanPath(py_trees.behaviour.Behaviour):
    """Pianifica percorso da posizione corrente a goal."""
    
    def __init__(self, goal_cell, name="PlanPath"):
        super().__init__(name)
        self.goal_cell = goal_cell
        self.path = None
        
    def update(self):
        # Ottieni posizione corrente
        current_cell = robot_state.world_to_cell(robot_state.robot_x, robot_state.robot_y)
        
        print(f"📍 Pianificazione: {current_cell} → {self.goal_cell}")
        
        # Calcola percorso
        self.path = bfs_path(
            current_cell, 
            self.goal_cell,
            robot_state.obstacles,
            robot_state.grid_size
        )
        
        if self.path:
            print(f"✓ Percorso trovato: {len(self.path)} celle")
            # Salva percorso nella blackboard per altri nodi
            self.blackboard = self.attach_blackboard_client()
            self.blackboard.register_key(key="path", access=py_trees.common.Access.WRITE)
            self.blackboard.set("path", self.path)
            return py_trees.common.Status.SUCCESS
        else:
            print("❌ Nessun percorso trovato!")
            return py_trees.common.Status.FAILURE


# ============================================================================
# MISSION BEHAVIORS (Riutilizzano GrabObject e DropObject esistenti)
# ============================================================================

class CollectObject(py_trees.behaviour.Behaviour):
    """Sequenza completa di raccolta oggetto."""
    
    def __init__(self, name="CollectObject"):
        super().__init__(name)
        self.step = 0
        self.start_time = None
        
    def initialise(self):
        self.step = 0
        self.start_time = time.time()
        print("="*50)
        print("🎯 RACCOLTA OGGETTO")
        print("="*50)
        
    def update(self):
        elapsed = time.time() - self.start_time
        
        if elapsed < 1.0 and self.step == 0:
            print("🤖 Apertura pinza...")
            open_hand(1000)
            self.step = 1
            
        elif 1.0 <= elapsed < 2.5 and self.step == 1:
            print("🤖 Abbassamento braccio...")
            arm_down()
            time.sleep(1.5)
            self.step = 2
            
        elif 2.5 <= elapsed < 4.0 and self.step == 2:
            print("🤖 Chiusura pinza...")
            close_hand(1500)
            self.step = 3
            
        elif 4.0 <= elapsed < 5.5 and self.step == 3:
            print("🤖 Sollevamento braccio...")
            arm_up()
            time.sleep(1.5)
            self.step = 4
            
        elif elapsed >= 5.5:
            print("✅ Oggetto raccolto!")
            return py_trees.common.Status.SUCCESS
        
        return py_trees.common.Status.RUNNING


class DeliverObject(py_trees.behaviour.Behaviour):
    """Sequenza completa di consegna oggetto."""
    
    def __init__(self, name="DeliverObject"):
        super().__init__(name)
        self.step = 0
        self.start_time = None
        
    def initialise(self):
        self.step = 0
        self.start_time = time.time()
        print("="*50)
        print("🏠 CONSEGNA OGGETTO")
        print("="*50)
        
    def update(self):
        elapsed = time.time() - self.start_time
        
        if elapsed < 1.5 and self.step == 0:
            print("📦 Abbassamento braccio...")
            arm_down()
            time.sleep(1.5)
            self.step = 1
            
        elif 1.5 <= elapsed < 3.0 and self.step == 1:
            print("📦 Apertura pinza...")
            open_hand(1000)
            self.step = 2
            
        elif 3.0 <= elapsed < 3.5 and self.step == 2:
            print("📦 Rilascio oggetto...")
            time.sleep(0.5)
            self.step = 3
            
        elif 3.5 <= elapsed < 5.0 and self.step == 3:
            print("📦 Sollevamento braccio...")
            arm_up()
            time.sleep(1.5)
            self.step = 4
            
        elif elapsed >= 5.0:
            print("✅ Oggetto consegnato!")
            print("="*50)
            print("🎉 MISSIONE COMPLETATA!")
            print("="*50)
            return py_trees.common.Status.SUCCESS
        
        return py_trees.common.Status.RUNNING


# ============================================================================
# COMPLETE MISSION TREE
# ============================================================================

def create_delivery_mission():
    """
    Crea Behavior Tree completo per missione pickup/delivery con navigazione.
    
    Struttura:
    - Pianifica percorso verso GOAL
    - Naviga verso GOAL
    - Raccogli oggetto
    - Pianifica percorso verso START
    - Naviga verso START
    - Consegna oggetto
    """
    root = py_trees.composites.Sequence(name="Delivery Mission", memory=True)
    
    # Fase 1: Vai al pickup
    plan_to_goal = PlanPath(goal_cell=robot_state.goal_cell, name="Plan to Pickup")
    
    # Fase 2: Raccogli
    collect = CollectObject(name="Collect Object")
    
    # Fase 3: Torna a casa
    plan_to_start = PlanPath(goal_cell=robot_state.start_cell, name="Plan to Home")
    
    # Fase 4: Consegna
    deliver = DeliverObject(name="Deliver Object")
    
    # Assembla albero
    root.add_children([
        plan_to_goal,
        collect,
        plan_to_start,
        deliver
    ])
    
    return root


def create_simple_navigation_test():
    """
    Crea un semplice test di navigazione verso una cella.
    Utile per testare il sistema prima della missione completa.
    """
    root = py_trees.composites.Sequence(name="Navigation Test", memory=False)
    
    # Test: vai a (1, 2) e torna a (0, 0)
    go_to_goal = NavigateToCell(1, 2, name="Go to (1,2)")
    go_to_start = NavigateToCell(0, 0, name="Return to (0,0)")
    
    root.add_children([go_to_goal, go_to_start])
    
    return root
