"""Behaviors per gestione batteria - Versione Standalone."""

import time
import py_trees


class CheckBattery(py_trees.behaviour.Behaviour):
    """Controlla se il livello di batteria è sufficiente."""
    
    def __init__(self, name="Check Battery"):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="node", access=py_trees.common.Access.READ)
        self.low_battery_warned = False
    
    def update(self):
        """Controlla il livello di batteria."""
        node = self.blackboard.get("node")
        
        if node.battery_level <= 20.0:
            if not self.low_battery_warned:
                node.log_warn(f'⚠️  BATTERIA BASSA: {node.battery_level:.0f}%')
                self.low_battery_warned = True
            return py_trees.common.Status.FAILURE
        else:
            # Reset warning quando batteria sopra soglia
            self.low_battery_warned = False
        
        return py_trees.common.Status.SUCCESS


class ChargeBattery(py_trees.behaviour.Behaviour):
    """Ricarica la batteria quando bassa."""
    
    def __init__(self, name="Charge Battery"):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="node", access=py_trees.common.Access.READ)
        self.charging_start_time = None
        self.last_increment_time = None
    
    def initialise(self):
        """Inizia il processo di ricarica."""
        node = self.blackboard.get("node")
        node.stop_robot()
        self.charging_start_time = time.time()
        self.last_increment_time = time.time()
        node.log_info('='*50)
        node.log_warn('⚠️  BATTERIA BASSA! Inizio ricarica...')
        node.log_info(f'🔋 Batteria: {node.battery_level:.0f}% → Ricarica...')
        node.log_info('='*50)
    
    def update(self):
        """Ricarica batteria del 30% ogni 5 secondi."""
        node = self.blackboard.get("node")
        current_time = time.time()
        elapsed = current_time - self.last_increment_time
        
        # Incrementa batteria ogni 5 secondi
        if elapsed >= 5.0:
            node.battery_level += 30.0
            if node.battery_level > 100.0:
                node.battery_level = 100.0
            
            self.last_increment_time = current_time
            node.log_info(f'🔌 Ricarica... Batteria: {node.battery_level:.0f}%')
            
            # Controlla se carica all'80%
            if node.battery_level >= 80.0:
                total_charge_time = current_time - self.charging_start_time
                node.log_info('='*50)
                node.log_info(f'✅ Batteria carica! ({total_charge_time:.1f}s)')
                node.log_info(f'🔋 Batteria: {node.battery_level:.0f}% → Riprendo missione...')
                node.log_info('='*50)
                return py_trees.common.Status.SUCCESS
        
        return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status):
        """Pulizia quando ricarica completata."""
        pass
