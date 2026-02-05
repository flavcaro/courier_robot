import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import socket
import time

# CONFIGURAZIONE RETE
# Se il ping raspberrypi.local funzionava, usa questo. 
# Altrimenti usa l'IP numerico (es. '192.168.137.2')
ROBOT_IP = 'pi.local' 
ROBOT_PORT = 65432

class EthernetDriver(Node):
    def __init__(self):
        super().__init__('ethernet_driver')
        
        self.sock = None
        self.connect_to_robot()

        # Iscrizione ai comandi di movimento (dal Behavior Tree)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Iscrizione ai comandi del braccio (se li hai implementati)
        # Esempio: Stringa "pickup", "drop"
        self.create_subscription(String, '/arm_cmd', self.arm_callback, 10)

        self.get_logger().info("Driver Ethernet Pronto! In attesa di comandi...")
        self.last_cmd = "Stop"

    def connect_to_robot(self):
        """Tenta di connettersi al Raspberry Pi via cavo"""
        while self.sock is None:
            try:
                self.get_logger().info(f"Tentativo connessione a {ROBOT_IP}...")
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(5) # Timeout di 5 secondi
                self.sock.connect((ROBOT_IP, ROBOT_PORT))
                self.get_logger().info("✅ CONNESSO AL ROBOT!")
            except Exception as e:
                self.get_logger().error(f"Connessione fallita: {e}. Riprovo tra 3s...")
                self.sock = None
                time.sleep(3)

    def send_command(self, cmd_string):
        """Invia la stringa grezza al Raspberry"""
        if self.sock:
            try:
                self.sock.sendall(cmd_string.encode('utf-8'))
            except BrokenPipeError:
                self.get_logger().error("Connessione persa! Tento riconnessione...")
                self.sock = None
                self.connect_to_robot()

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        
        # Logica Semplificata per Makeblock (Forward, Back, Left, Right)
        # Poiché l'API Arduino non supporta curve miste, diamo priorità alla rotazione
        
        cmd = "Stop"
        speed = 0.0

        if abs(angular) > 0.2: # Se sta sterzando
            speed = abs(angular)
            if angular > 0:
                cmd = "Left"
            else:
                cmd = "Right"
        elif abs(linear) > 0.1: # Se sta andando dritto
            speed = abs(linear)
            if linear > 0:
                cmd = "Forward"
            else:
                cmd = "Back"
        
        # Invia solo se il comando cambia (per non intasare la rete)
        if cmd != self.last_cmd or (cmd != "Stop" and speed > 0):
            # Limita la velocità a max 1.0
            speed = min(speed, 1.0)
            
            if cmd == "Stop":
                message = "Stop"
            else:
                message = f"{cmd}:{speed:.2f}" # Es: "Forward:0.50"
            
            self.send_command(message)
            self.last_cmd = cmd

    def arm_callback(self, msg):
        # Gestione Braccio
        command = msg.data # Es: "armUP", "openHand"
        self.send_command(command)

def main(args=None):
    rclpy.init(args=args)
    node = EthernetDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.send_command("Stop")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()