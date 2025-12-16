import rclpy
from rclpy.node import Node
import subprocess
import time

class WorldSpawner(Node):
    def __init__(self):
        super().__init__('world_spawner')
        
        # --- CONFIGURAZIONE GRIGLIA ---
        self.cell_size = 1.0
        self.grid_map = [
            [0, 0, 0, 0, 0],
            [0, 1, 1, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 1, 0, 1, 0],
            [0, 0, 0, 0, 0]
        ]
        self.start_cell = (0, 0)
        self.goal_cell = (4, 2)

        self.get_logger().info('Inizio a popolare il mondo (Gazebo Harmonic)...')
        self.spawn_world()

    def get_sdf_box(self, color, size_x, size_y, size_z, with_collision=True):
        """Genera l'SDF per un cubo colorato"""
        # Nota: I colori in Harmonic usano <ambient> e <diffuse>
        rgb = "0.8 0.8 0.8 1"
        if color == "Red": rgb = "1 0 0 1"
        elif color == "Green": rgb = "0 1 0 1"
        elif color == "Blue": rgb = "0 0 1 1"
        
        # Collision block solo se richiesto
        collision_block = ""
        if with_collision:
            collision_block = f"""
              <collision name='collision'>
                <geometry><box><size>{size_x} {size_y} {size_z}</size></box></geometry>
              </collision>"""
        
        return f"""<?xml version='1.0'?>
        <sdf version='1.6'>
          <model name='box_model'>
            <static>true</static>
            <link name='link'>
              <visual name='visual'>
                <geometry><box><size>{size_x} {size_y} {size_z}</size></box></geometry>
                <material>
                  <ambient>{rgb}</ambient>
                  <diffuse>{rgb}</diffuse>
                </material>
              </visual>{collision_block}
            </link>
          </model>
        </sdf>"""

    def spawn_object(self, name, x, y, z, color, sx, sy, sz, with_collision=True):
        sdf_content = self.get_sdf_box(color, sx, sy, sz, with_collision)
        
        # Comando per spawnare in Harmonic: ros2 run ros_gz_sim create ...
        cmd = [
            "ros2", "run", "ros_gz_sim", "create",
            "-world", "empty",
            "-name", name,
            "-string", sdf_content,
            "-x", str(x), "-y", str(y), "-z", str(z)
        ]
        
        # Esegue il comando senza bloccare troppo
        subprocess.Popen(cmd)
        time.sleep(0.1) # Piccola pausa per non intasare Gazebo

    def spawn_world(self):
        rows = len(self.grid_map)
        cols = len(self.grid_map[0])
        
        for r in range(rows):
            for c in range(cols):
                x = (r + 0.5) * self.cell_size
                y = (c + 0.5) * self.cell_size
                
                if self.grid_map[r][c] == 1:
                    # Ostacoli CON collisioni ma PIÙ PICCOLI per lasciare spazio al robot
                    # Robot è 0.4m x 0.4m, celle sono 1.0m, quindi ostacoli di 0.6m lasciano 0.4m di corridoio
                    self.spawn_object(f"obs_{r}_{c}", x, y, 0.3, "Red", 0.6, 0.6, 0.6, with_collision=True)
                elif (r, c) == self.start_cell:
                    self.spawn_object("start", x, y, 0.01, "Green", self.cell_size*0.9, self.cell_size*0.9, 0.01, with_collision=False)
                elif (r, c) == self.goal_cell:
                    self.spawn_object("goal", x, y, 0.01, "Blue", self.cell_size*0.9, self.cell_size*0.9, 0.01, with_collision=False)

        self.get_logger().info("Comandi di spawn inviati!")

def main(args=None):
    rclpy.init(args=args)
    node = WorldSpawner()
    # Non serve spinare perché i comandi sono lanciati subito
    node.destroy_node()
    rclpy.shutdown()

