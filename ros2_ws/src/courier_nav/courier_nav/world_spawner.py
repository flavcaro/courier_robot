import rclpy
from rclpy.node import Node
import subprocess
import time

class WorldSpawner(Node):
    def __init__(self):
        super().__init__('world_spawner')
        
        # --- GRID CONFIGURATION ---
        # Coordinate system: cell (x, y) maps to world position (x+0.5, y+0.5)
        # x = column (0-4 left to right)
        # y = row (0-4 bottom to top)
        #
        # Visual grid (y increases upward):
        #   y=4: [ ][ ][ ][ ][ ]
        #   y=3: [ ][X][ ][X][ ]   X = Obstacle
        #   y=2: [ ][ ][ ][ ][ ]
        #   y=1: [ ][X][X][ ][ ]   X = Obstacle
        #   y=0: [S][ ][ ][ ][ ]   S = Start
        #        x=0 x=1 x=2 x=3 x=4
        #
        # Goal is at (2, 4) - column 2, row 4 (top middle area)
        
        self.cell_size = 1.0
        
        # Obstacles at: (1,1), (2,1), (1,3), (3,3)
        self.obstacles = [(1, 1), (2, 1), (1, 3), (3, 3)]
        
        self.start_cell = (0, 0)  # Bottom-left corner
        self.goal_cell = (2, 4)   # Top middle area

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
        
        self.get_logger().info(f"Spawning {name} at ({x:.2f}, {y:.2f}, {z:.2f}) - color: {color}")
        
        # Comando per spawnare in Harmonic: ros2 run ros_gz_sim create ...
        cmd = [
            "ros2", "run", "ros_gz_sim", "create",
            "-world", "empty",
            "-name", name,
            "-string", sdf_content,
            "-x", str(x), "-y", str(y), "-z", str(z)
        ]
        
        # Esegue il comando e attende il completamento
        try:
            subprocess.run(cmd, timeout=5, capture_output=True)
        except subprocess.TimeoutExpired:
            self.get_logger().warn(f"Timeout spawning {name}")
        time.sleep(0.5)  # Pausa per Gazebo

    def spawn_world(self):
        self.get_logger().info(f"Spawning world: start={self.start_cell}, goal={self.goal_cell}")
        self.get_logger().info(f"Obstacles at: {self.obstacles}")
        
        # Spawn start cell (green) - robot starts here
        start_x = (self.start_cell[0] + 0.5) * self.cell_size
        start_y = (self.start_cell[1] + 0.5) * self.cell_size
        self.get_logger().info(f"Start cell at world pos ({start_x}, {start_y})")
        self.spawn_object("start_cell", start_x, start_y, 0.01, "Green", self.cell_size*0.9, self.cell_size*0.9, 0.02, with_collision=False)
        
        # Spawn goal cell (blue)
        goal_x = (self.goal_cell[0] + 0.5) * self.cell_size
        goal_y = (self.goal_cell[1] + 0.5) * self.cell_size
        self.get_logger().info(f"Goal cell at world pos ({goal_x}, {goal_y})")
        self.spawn_object("goal_cell", goal_x, goal_y, 0.01, "Blue", self.cell_size*0.9, self.cell_size*0.9, 0.02, with_collision=False)
        
        # Spawn obstacles (red cubes) - size 0.95 to block entire cell
        for obs_x, obs_y in self.obstacles:
            world_x = (obs_x + 0.5) * self.cell_size
            world_y = (obs_y + 0.5) * self.cell_size
            self.get_logger().info(f"Obstacle at cell ({obs_x},{obs_y}) -> world ({world_x}, {world_y})")
            # Obstacles are 0.95x0.95m to block the entire cell, preventing diagonal paths
            self.spawn_object(f"obs_{obs_x}_{obs_y}", world_x, world_y, 0.3, "Red", 0.95, 0.95, 0.6, with_collision=True)

        # Spawn AprilTag markers at strategic locations
        self.get_logger().info("Spawning AprilTag markers...")
        self.spawn_all_apriltags()
        
        self.get_logger().info("All spawn commands completed!")
    
    def spawn_apriltag(self, tag_id, x, y, z, yaw, color="White"):
        """
        Spawn an AprilTag marker at the specified position.
        
        Args:
            tag_id: Unique ID for this tag (0, 1, 2, ...)
            x, y, z: Position in world coordinates
            yaw: Rotation around Z axis (radians). 0=facing +X, pi/2=facing +Y
            color: Background color of the tag board
        """
        # Color mapping
        color_rgb = {
            "White": "1 1 1 1",
            "Yellow": "1 1 0 1",
            "Cyan": "0 1 1 1",
            "Magenta": "1 0 1 1",
        }.get(color, "1 1 1 1")
        
        sdf_content = f"""<?xml version='1.0'?>
        <sdf version='1.6'>
          <model name='apriltag_{tag_id}'>
            <static>true</static>
            <pose>0 0 0 0 0 {yaw}</pose>
            <link name='link'>
              <visual name='board_visual'>
                <geometry>
                  <box><size>0.25 0.02 0.25</size></box>
                </geometry>
                <material>
                  <ambient>{color_rgb}</ambient>
                  <diffuse>{color_rgb}</diffuse>
                </material>
              </visual>
              <visual name='tag_visual'>
                <pose>0.011 0 0 0 0 0</pose>
                <geometry>
                  <box><size>0.001 0.15 0.15</size></box>
                </geometry>
                <material>
                  <ambient>0 0 0 1</ambient>
                  <diffuse>0 0 0 1</diffuse>
                </material>
              </visual>
              <visual name='id_marker'>
                <pose>0.012 0 0.08 0 0 0</pose>
                <geometry>
                  <box><size>0.001 0.03 0.03</size></box>
                </geometry>
                <material>
                  <ambient>1 0 0 1</ambient>
                  <diffuse>1 0 0 1</diffuse>
                </material>
              </visual>
            </link>
          </model>
        </sdf>"""
        
        cmd = [
            "ros2", "run", "ros_gz_sim", "create",
            "-world", "empty",
            "-name", f"apriltag_{tag_id}",
            "-string", sdf_content,
            "-x", str(x), "-y", str(y), "-z", str(z)
        ]
        
        try:
            subprocess.run(cmd, timeout=5, capture_output=True)
        except subprocess.TimeoutExpired:
            self.get_logger().warn(f"Timeout spawning apriltag_{tag_id}")
        time.sleep(0.3)
        self.get_logger().info(f"AprilTag {tag_id} spawned at ({x:.2f}, {y:.2f}, {z:.2f})")
    
    def spawn_all_apriltags(self):
        """
        Spawn AprilTags at strategic locations in the grid.
        
        Grid layout (x=column, y=row):
           y=4: [ ][ ][G][ ][ ]   G = Goal (2,4)
           y=3: [ ][X][ ][X][ ]   X = Obstacle
           y=2: [ ][ ][ ][ ][ ]
           y=1: [ ][X][X][ ][ ]   X = Obstacle  
           y=0: [S][ ][ ][ ][ ]   S = Start (0,0)
                x=0 x=1 x=2 x=3 x=4
        
        Tags are placed at cell edges, facing the direction where robot will approach.
        """
        import math
        
        # Tag positions: (tag_id, cell_x, cell_y, position_in_cell, facing_direction, color)
        # position_in_cell: where in cell to place tag
        # facing_direction: direction the tag faces (where robot should be to see it)
        
        tags = [
            # Tag 0: At start cell (0,0) - robot can see when starting
            (0, 0, 0, 'center', 'north', "White"),
            
            # Tag 1: At goal cell (2,4) - robot sees when arriving at goal
            (1, 2, 4, 'south', 'south', "Yellow"),
            
            # Tag 2: At cell (0,2) - on the safe path corridor
            (2, 0, 2, 'east', 'east', "Cyan"),
            
            # Tag 3: At cell (0,4) - corner near goal
            (3, 0, 4, 'east', 'east', "White"),
            
            # Tag 4: At cell (2,2) - center corridor
            (4, 2, 2, 'south', 'south', "Magenta"),
            
            # Tag 5: At cell (4,2) - right side of grid
            (5, 4, 2, 'west', 'west', "Cyan"),
        ]
        
        for tag_id, cell_x, cell_y, position, facing, color in tags:
            # Base cell center in world coordinates
            cx = (cell_x + 0.5) * self.cell_size
            cy = (cell_y + 0.5) * self.cell_size
            
            # Offset based on position in cell
            offset = 0.35  # Distance from cell center to edge
            if position == 'north':
                y = cy + offset
                x = cx
            elif position == 'south':
                y = cy - offset
                x = cx
            elif position == 'east':
                x = cx + offset
                y = cy
            elif position == 'west':
                x = cx - offset
                y = cy
            else:  # center
                x = cx
                y = cy
            
            # Yaw based on facing direction
            # 0 = facing +X (east), pi/2 = facing +Y (north), pi = facing -X (west), -pi/2 = facing -Y (south)
            yaw_map = {
                'east': 0,
                'north': math.pi / 2,
                'west': math.pi,
                'south': -math.pi / 2,
            }
            yaw = yaw_map.get(facing, 0)
            
            z = 0.4  # Height above ground
            
            self.spawn_apriltag(tag_id, x, y, z, yaw, color)


def main(args=None):
    rclpy.init(args=args)
    node = WorldSpawner()
    # Non serve spinare perché i comandi sono lanciati subito
    node.destroy_node()
    rclpy.shutdown()

