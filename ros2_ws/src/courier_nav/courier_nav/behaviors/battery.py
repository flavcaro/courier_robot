"""Battery management behaviors for the courier robot."""

import py_trees
import time


class CheckBattery(py_trees.behaviour.Behaviour):
    """Check if battery level is sufficient to continue mission."""
    
    def __init__(self, name="Check Battery"):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="node", access=py_trees.common.Access.READ)
    
    def update(self):
        """Check battery level."""
        node = self.blackboard.get("node")
        
        if node.battery_level <= 20.0:
            node.get_logger().warn(f'⚠️  LOW BATTERY: {node.battery_level:.0f}%')
            return py_trees.common.Status.FAILURE
        
        return py_trees.common.Status.SUCCESS


class ChargeBattery(py_trees.behaviour.Behaviour):
    """Charge the battery when low."""
    
    def __init__(self, name="Charge Battery"):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="node", access=py_trees.common.Access.READ)
        self.charging_start_time = None
        self.last_increment_time = None
    
    def initialise(self):
        """Start charging process."""
        node = self.blackboard.get("node")
        node.stop_robot()
        self.charging_start_time = time.time()
        self.last_increment_time = time.time()
        node.get_logger().info('='*50)
        node.get_logger().info('⚠️  LOW BATTERY! Starting recharge...')
        node.get_logger().info(f'🔋 Battery: {node.battery_level:.0f}% → Charging...')
        node.get_logger().info('='*50)
    
    def update(self):
        """Charge battery by 30% every 5 seconds."""
        node = self.blackboard.get("node")
        current_time = time.time()
        elapsed = current_time - self.last_increment_time
        
        # Increment battery every 5 seconds
        if elapsed >= 5.0:
            node.battery_level += 30.0
            if node.battery_level > 100.0:
                node.battery_level = 100.0
            
            self.last_increment_time = current_time
            node.get_logger().info(f'🔌 Charging... Battery: {node.battery_level:.0f}%')
            
            # Check if fully charged (>20% is enough to continue)
            if node.battery_level > 20.0:
                total_charge_time = current_time - self.charging_start_time
                node.get_logger().info('='*50)
                node.get_logger().info(f'✅ Battery charged! ({total_charge_time:.1f}s)')
                node.get_logger().info(f'🔋 Battery: {node.battery_level:.0f}% → Resuming mission...')
                node.get_logger().info('='*50)
                return py_trees.common.Status.SUCCESS
        
        return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status):
        """Clean up when charging is done."""
        pass
