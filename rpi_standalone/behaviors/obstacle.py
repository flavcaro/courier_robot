"""Behavior per gestione ostacoli - Versione Standalone."""

import time
import py_trees
from py_trees import common
from collections import deque


class HandleObstacle(py_trees.behaviour.Behaviour):
    """Gestisce rilevamento ostacoli: indietreggia e ripianifica."""
    
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="node", access=common.Access.READ)
        self.blackboard.register_key(key="path_queue", access=common.Access.WRITE)
        self.blackboard.register_key(key="current_target", access=common.Access.READ)
        self.blackboard.register_key(key="returning_home", access=common.Access.READ)
    
    def update(self):
        """Gestisce ostacolo: indietreggia e ripianifica."""
        node = self.blackboard.get("node")
        current_target = self.blackboard.get("current_target")
        returning_home = self.blackboard.get("returning_home")
        
        node.stop_robot()
        node.log_warn('🚧 Ostacolo rilevato! Indietreggiamento...')
        
        # Indietreggia
        backup_duration = 1.0  # secondi
        node.rover.moveTo("Back", 0.2)
        time.sleep(backup_duration)
        node.stop_robot()
        
        # Aggiorna odometria stimata (indietreggiamento)
        backup_distance = 0.2  # metri stimati
        node.robot_x -= backup_distance * math.cos(node.robot_yaw)
        node.robot_y -= backup_distance * math.sin(node.robot_yaw)
        
        # Ottieni posizione corrente
        current_cell = node.world_to_cell(node.robot_x, node.robot_y)
        
        # Determina target per ripianificazione
        if returning_home:
            target = node.start_cell
            node.log_info(f'🔄 Ripianificazione RITORNO da {current_cell} a {target}')
        else:
            target = node.goal_cell
            node.log_info(f'🔄 Ripianificazione da {current_cell} a {target}')
        
        # Controlla se già al target
        if current_cell == target:
            node.log_info(f'✓ Già al target {target}! Continuo missione.')
            self.blackboard.set("path_queue", deque())
            return py_trees.common.Status.SUCCESS
        
        # Ripianifica usando BFS
        new_path = node.bfs_path(current_cell, target)
        
        if new_path:
            self.blackboard.set("path_queue", deque(new_path))
            node.log_info(f'✓ NUOVO PERCORSO trovato: {new_path}')
            return py_trees.common.Status.SUCCESS
        else:
            node.log_error(f'❌ NESSUN PERCORSO disponibile verso {target}!')
            return py_trees.common.Status.FAILURE


import math  # Aggiunto import mancante
