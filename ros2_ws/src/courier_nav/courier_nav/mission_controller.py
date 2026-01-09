#!/usr/bin/env python3
"""
Cell-to-Cell Navigation Controller - BEHAVIOR TREE ARCHITECTURE

Main entry point for the courier robot mission controller.
The implementation is now modularized:
- behaviors/navigation.py: Movement and rotation behaviors
- behaviors/mission.py: Object collection and delivery
- behaviors/conditions.py: State checking behaviors
- behaviors/obstacle.py: Obstacle handling
- controller.py: Main behavior tree controller node
"""

import rclpy
from .controller import BehaviorTreeController


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorTreeController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
