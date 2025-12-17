#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math


def euler_from_quaternion(x, y, z, w):
    """Converte quaternione in angolo yaw."""
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)


class PIDController(Node):
    """
    Controllore PID per il movimento del robot.
    Riceve target pose su /target_pose e pubblica velocità su /cmd_vel.
    """
    
    def __init__(self):
        super().__init__('pid_controller')
        
        # PID Gains (Linear)
        self.kp_linear = 0.5
        self.ki_linear = 0.0
        self.kd_linear = 0.1
        
        # PID Gains (Angular)
        self.kp_angular = 1.0
        self.ki_angular = 0.0
        self.kd_angular = 0.2
        
        # PID State
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0
        self.integral_linear = 0.0
        self.integral_angular = 0.0
        
        # Robot State
        self.current_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.target_pose = None
        self.target_reached_threshold = 0.15  # 15 cm
        
        # Publishers & Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_reached_pub = self.create_publisher(PoseStamped, '/target_reached', 10)
        
        self.create_subscription(PoseStamped, '/target_pose', self.target_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Control Loop (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('🎮 PID Controller avviato')
    
    def odom_callback(self, msg):
        """Aggiorna posizione corrente del robot."""
        self.current_pose['x'] = msg.pose.pose.position.x
        self.current_pose['y'] = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        self.current_pose['theta'] = euler_from_quaternion(q.x, q.y, q.z, q.w)
    
    def target_callback(self, msg):
        """Riceve un nuovo target."""
        self.target_pose = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f'🎯 Nuovo target: ({self.target_pose[0]:.2f}, {self.target_pose[1]:.2f})')
        
        # Reset PID
        self.integral_linear = 0.0
        self.integral_angular = 0.0
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0
    
    def control_loop(self):
        """Ciclo di controllo PID."""
        if self.target_pose is None:
            # Nessun target, ferma il robot
            self.publish_velocity(0.0, 0.0)
            return
        
        # Calcola errore
        tx, ty = self.target_pose
        dx = tx - self.current_pose['x']
        dy = ty - self.current_pose['y']
        distance = math.sqrt(dx**2 + dy**2)
        
        # 🆕 LOG DEBUG
        if not hasattr(self, '_last_debug_time'):
            import time
            self._last_debug_time = time.time()
        
        import time
        if time.time() - self._last_debug_time > 1.0:  # Ogni secondo
            self.get_logger().info(
                f"🤖 Pos: ({self.current_pose['x']:.2f}, {self.current_pose['y']:.2f}) | "
                f"Target: ({tx:.2f}, {ty:.2f}) | "
                f"Dist: {distance:.2f}m"
            )
            self._last_debug_time = time.time()
        
        # Target raggiunto?
        if distance < self.target_reached_threshold:
            self.get_logger().info(f'✅ Target raggiunto: ({tx:.2f}, {ty:.2f})')
            
            # Pubblica notifica
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'odom'
            msg.pose.position.x = tx
            msg.pose.position.y = ty
            self.target_reached_pub.publish(msg)
            
            # Ferma il robot
            self.target_pose = None
            self.publish_velocity(0.0, 0.0)
            return
        
        # Calcola angolo target
        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - self.current_pose['theta']
        
        # Normalizza angolo [-π, π]
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
        
        # PID Lineare
        self.integral_linear += distance
        derivative_linear = distance - self.prev_linear_error
        linear_vel = (
            self.kp_linear * distance +
            self.ki_linear * self.integral_linear +
            self.kd_linear * derivative_linear
        )
        self.prev_linear_error = distance
        
        # Limita velocità lineare
        linear_vel = max(0.0, min(linear_vel, 0.5))  # Max 0.5 m/s
        
        # PID Angolare
        self.integral_angular += angle_error
        derivative_angular = angle_error - self.prev_angular_error
        angular_vel = (
            self.kp_angular * angle_error +
            self.ki_angular * self.integral_angular +
            self.kd_angular * derivative_angular
        )
        self.prev_angular_error = angle_error
        
        # Limita velocità angolare
        angular_vel = max(-1.0, min(angular_vel, 1.0))  # Max ±1.0 rad/s
        
        # Se non è allineato, ruota sul posto
        if abs(angle_error) > 0.2:  # 11 gradi
            linear_vel = 0.0
        
        self.publish_velocity(linear_vel, angular_vel)
    
    def publish_velocity(self, linear, angular):
        """Pubblica comando di velocità."""
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()