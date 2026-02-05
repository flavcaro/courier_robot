"""Behaviors per la missione - Versione Standalone."""

import time
import math
import py_trees
from py_trees import common
from collections import deque


class CollectObject(py_trees.behaviour.Behaviour):
    """Sequenza di raccolta oggetto con braccio robotico."""
    
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="node", access=common.Access.READ)
        self.blackboard.register_key(key="object_collected", access=common.Access.WRITE)
        self.animation_start = None
        self.animation_step = 0
        
    def initialise(self):
        """Inizia la sequenza di raccolta."""
        node = self.blackboard.get("node")
        self.animation_start = time.time()
        self.animation_step = 0
        node.log_info('='*50)
        node.log_info('🎯 RAGGIUNTO OBIETTIVO! Raccolta oggetto...')
        node.log_info('='*50)
        node.stop_robot()
        
    def update(self):
        """Esegue la sequenza di raccolta."""
        node = self.blackboard.get("node")
        
        if self.animation_start is None:
            return py_trees.common.Status.FAILURE
        
        elapsed = time.time() - self.animation_start
        
        # Sequenza temporizzata
        if elapsed < 1.0 and self.animation_step == 0:
            node.log_info('🤖 Apertura pinza...')
            node.rover.openHand(1000)
            self.animation_step = 1
            
        elif 1.0 <= elapsed < 2.5 and self.animation_step == 1:
            node.log_info('🤖 Abbassamento braccio...')
            node.rover.armDown()
            time.sleep(1.5)
            self.animation_step = 2
            
        elif 2.5 <= elapsed < 4.0 and self.animation_step == 2:
            node.log_info('🤖 Chiusura pinza...')
            node.rover.closeHand(1500)
            self.animation_step = 3
            
        elif 4.0 <= elapsed < 5.5 and self.animation_step == 3:
            node.log_info('🤖 Sollevamento braccio...')
            node.rover.armUP()
            time.sleep(1.5)
            self.animation_step = 4
            
        elif elapsed >= 5.5 and self.animation_step == 4:
            node.log_info('✅ Oggetto raccolto!')
            self.blackboard.set("object_collected", True)
            return py_trees.common.Status.SUCCESS
        
        return py_trees.common.Status.RUNNING


class DeliverObject(py_trees.behaviour.Behaviour):
    """Sequenza di consegna oggetto."""
    
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="node", access=common.Access.READ)
        self.animation_start = None
        self.animation_step = 0
        
    def initialise(self):
        """Inizia la sequenza di consegna."""
        node = self.blackboard.get("node")
        self.animation_start = time.time()
        self.animation_step = 0
        node.log_info('='*50)
        node.log_info('🏠 TORNATO A CASA! Consegna oggetto...')
        node.log_info('='*50)
        node.stop_robot()
        
    def update(self):
        """Esegue la sequenza di consegna."""
        node = self.blackboard.get("node")
        
        if self.animation_start is None:
            return py_trees.common.Status.FAILURE
        
        elapsed = time.time() - self.animation_start
        
        # Sequenza temporizzata
        if elapsed < 1.5 and self.animation_step == 0:
            node.log_info('📦 Abbassamento braccio...')
            node.rover.armDown()
            time.sleep(1.5)
            self.animation_step = 1
            
        elif 1.5 <= elapsed < 3.0 and self.animation_step == 1:
            node.log_info('📦 Apertura pinza...')
            node.rover.openHand(1000)
            self.animation_step = 2
            
        elif 3.0 <= elapsed < 3.5 and self.animation_step == 2:
            node.log_info('📦 Rilascio oggetto...')
            time.sleep(0.5)
            self.animation_step = 3
            
        elif 3.5 <= elapsed < 5.0 and self.animation_step == 3:
            node.log_info('📦 Sollevamento braccio...')
            node.rover.armUP()
            time.sleep(1.5)
            self.animation_step = 4
            
        elif elapsed >= 5.0 and self.animation_step == 4:
            node.log_info('✅ Oggetto consegnato!')
            node.log_info('='*50)
            node.log_info('🎉 MISSIONE COMPLETATA!')
            node.log_info('='*50)
            return py_trees.common.Status.SUCCESS
        
        return py_trees.common.Status.RUNNING


class PlanReturnPath(py_trees.behaviour.Behaviour):
    """Pianifica il percorso di ritorno verso casa."""
    
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="node", access=common.Access.READ)
        self.blackboard.register_key(key="path_queue", access=common.Access.WRITE)
        self.blackboard.register_key(key="returning_home", access=common.Access.WRITE)
        
    def update(self):
        """Calcola il percorso di ritorno."""
        node = self.blackboard.get("node")
        
        node.log_info('='*50)
        node.log_info('📍 PIANIFICAZIONE PERCORSO DI RITORNO')
        node.log_info('='*50)
        
        current_cell = node.world_to_cell(node.robot_x, node.robot_y)
        node.log_info(f'Posizione: {current_cell} → Casa: {node.start_cell}')
        
        path = node.bfs_path(current_cell, node.start_cell)
        
        if path:
            self.blackboard.set("path_queue", deque(path))
            self.blackboard.set("returning_home", True)
            node.log_info(f'✓ Percorso trovato con {len(path)} waypoints')
            for i, cell in enumerate(path):
                wx, wy = node.cell_to_world(cell[0], cell[1])
                node.log_info(f'  {i+1}. Cella{cell} → ({wx:.2f}, {wy:.2f})')
            return py_trees.common.Status.SUCCESS
        else:
            node.log_error('❌ NESSUN PERCORSO TROVATO!')
            return py_trees.common.Status.FAILURE


class PlanPath(py_trees.behaviour.Behaviour):
    """Pianifica il percorso iniziale verso l'obiettivo."""
    
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="node", access=common.Access.READ)
        self.blackboard.register_key(key="path_queue", access=common.Access.WRITE)
        self.blackboard.register_key(key="returning_home", access=common.Access.WRITE)
    
    def update(self):
        """Calcola il percorso da start a goal."""
        node = self.blackboard.get("node")
        
        node.log_info('='*50)
        node.log_info('📍 PIANIFICAZIONE PERCORSO INIZIALE')
        node.log_info('='*50)
        
        node.log_info(f'Partenza: {node.start_cell} → Obiettivo: {node.goal_cell}')
        
        path = node.bfs_path(node.start_cell, node.goal_cell)
        
        if path:
            self.blackboard.set("path_queue", deque(path))
            self.blackboard.set("returning_home", False)
            node.log_info(f'✓ Percorso trovato con {len(path)} waypoints')
            for i, cell in enumerate(path):
                wx, wy = node.cell_to_world(cell[0], cell[1])
                node.log_info(f'  {i+1}. Cella{cell} → ({wx:.2f}, {wy:.2f})')
            return py_trees.common.Status.SUCCESS
        else:
            node.log_error('❌ NESSUN PERCORSO TROVATO!')
            return py_trees.common.Status.FAILURE
