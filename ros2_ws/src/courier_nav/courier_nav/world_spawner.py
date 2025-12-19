#!/usr/bin/env python3
"""
World Spawner usando gz service direttamente (Gazebo Harmonic)
- Griglia 5x5 con pavimento a scacchiera
- Muri di confine
- Ostacoli che riempiono le celle
- AprilTag sui bordi delle celle
"""

import rclpy
from rclpy.node import Node
import subprocess
import time
import math


class WorldSpawner(Node):
    def __init__(self):
        super().__init__('world_spawner')
        
        self.grid_size = 5
        self.cell_size = 1.0
        # Obstacles as (row, col) - same as controller
        # row=1: col=1,2 | row=3: col=1,3
        self.obstacles = [(1, 1), (1, 2), (3, 1), (3, 3)]
        self.start_cell = (0, 0)   # row=0, col=0
        self.goal_cell = (4, 2)    # row=4, col=2
        
        self.spawn_counter = 0
        
        self.get_logger().info('World Spawner starting...')
        
        # Wait for Gazebo to be ready
        time.sleep(2.0)
        
        self.spawn_world()
    
    def spawn_sdf(self, name: str, sdf: str, x: float, y: float, z: float = 0.0, yaw: float = 0.0):
        """Spawn an entity using gz service command."""
        # Convert yaw to quaternion
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        
        # Escape SDF for command line - replace newlines and quotes
        sdf_escaped = sdf.replace('\n', ' ').replace('"', '\\"').replace("'", "\\'")
        
        req = f'sdf: "{sdf_escaped}", name: "{name}", pose: {{position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{z: {qz}, w: {qw}}}}}'
        
        cmd = [
            'gz', 'service', '-s', '/world/empty/create',
            '--reqtype', 'gz.msgs.EntityFactory',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '3000',
            '--req', req
        ]
        
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
            # Success: don't log every spawn to avoid flooding the console
            return True
            else:
                self.get_logger().warn(f'Failed to spawn {name}: {result.stderr}')
                return False
        except subprocess.TimeoutExpired:
            self.get_logger().warn(f'Timeout spawning {name}')
            return False
        except Exception as e:
            self.get_logger().error(f'Error spawning {name}: {e}')
            return False
    
    def get_box_sdf(self, size_x: float, size_y: float, size_z: float, 
                   r: float, g: float, b: float, collision: bool = False) -> str:
        """Generate SDF for a box."""
        collision_str = f'''<collision name="collision">
          <geometry><box><size>{size_x} {size_y} {size_z}</size></box></geometry>
        </collision>''' if collision else ''
        
        return f'''<?xml version="1.0"?>
<sdf version="1.8">
  <model name="box">
    <static>true</static>
    <link name="link">
      {collision_str}
      <visual name="visual">
        <geometry><box><size>{size_x} {size_y} {size_z}</size></box></geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>'''
    
    def get_cylinder_sdf(self, radius: float, length: float,
                        r: float, g: float, b: float) -> str:
        """Generate SDF for a cylinder."""
        return f'''<?xml version="1.0"?>
<sdf version="1.8">
  <model name="cylinder">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>'''
    
    def spawn_world(self):
        """Spawn all world elements."""
        self.get_logger().info('='*50)
        self.get_logger().info('SPAWNING WORLD ELEMENTS')
        self.get_logger().info('='*50)
        
        grid_length = self.grid_size * self.cell_size
        
        # 1. FLOOR TILES (checkered pattern)
        self.get_logger().info('Spawning floor...')
        for i in range(self.grid_size):
            for j in range(self.grid_size):
                is_white = (i + j) % 2 == 0
                color = (0.9, 0.9, 0.9) if is_white else (0.6, 0.6, 0.6)
                x = i * self.cell_size + self.cell_size / 2
                y = j * self.cell_size + self.cell_size / 2
                
                sdf = self.get_box_sdf(0.98, 0.98, 0.01, *color)
                self.spawn_sdf(f'floor_{i}_{j}', sdf, x, y, -0.005)
        
        # 2. GRID LINES
        self.get_logger().info('Spawning grid lines...')
        for i in range(self.grid_size + 1):
            # Horizontal lines
            sdf_h = self.get_box_sdf(grid_length, 0.02, 0.01, 0, 0, 0)
            self.spawn_sdf(f'hline_{i}', sdf_h, grid_length/2, i * self.cell_size, 0.01)
            
            # Vertical lines
            sdf_v = self.get_box_sdf(0.02, grid_length, 0.01, 0, 0, 0)
            self.spawn_sdf(f'vline_{i}', sdf_v, i * self.cell_size, grid_length/2, 0.01)
        
        # 3. BOUNDARY WALLS (blue)
        self.get_logger().info('Spawning boundary walls...')
        wall_height = 0.4
        wall_thickness = 0.1
        
        # South wall (y = 0)
        sdf = self.get_box_sdf(grid_length + 0.2, wall_thickness, wall_height, 0.2, 0.2, 0.8, True)
        self.spawn_sdf('wall_south', sdf, grid_length/2, -wall_thickness/2, wall_height/2)
        
        # North wall (y = grid_length)
        self.spawn_sdf('wall_north', sdf, grid_length/2, grid_length + wall_thickness/2, wall_height/2)
        
        # West wall (x = 0)
        sdf = self.get_box_sdf(wall_thickness, grid_length + 0.2, wall_height, 0.2, 0.2, 0.8, True)
        self.spawn_sdf('wall_west', sdf, -wall_thickness/2, grid_length/2, wall_height/2)
        
        # East wall (x = grid_length)
        self.spawn_sdf('wall_east', sdf, grid_length + wall_thickness/2, grid_length/2, wall_height/2)
        
        # 4. OBSTACLES (red boxes filling cells)
        self.get_logger().info('Spawning obstacles...')
        obstacle_sdf = self.get_box_sdf(0.9, 0.9, 0.5, 0.8, 0.1, 0.1, True)
        
        for (row, col) in self.obstacles:
            # Convert (row, col) to world coordinates
            # col -> x, row -> y
            x = col * self.cell_size + self.cell_size / 2
            y = row * self.cell_size + self.cell_size / 2
            
            self.spawn_sdf(f'obstacle_{row}_{col}', obstacle_sdf, x, y, 0.25)
            
            # X marker on top (orange)
            x_sdf = self.get_box_sdf(0.6, 0.08, 0.05, 1.0, 0.5, 0.0)
            self.spawn_sdf(f'x1_{row}_{col}', x_sdf, x, y, 0.55, 0.785)
            self.spawn_sdf(f'x2_{row}_{col}', x_sdf, x, y, 0.55, -0.785)
        
        # 5. APRILTAG MARKERS (mounted ON walls - flat against surface)
        self.get_logger().info('Spawning AprilTag markers...')
        # Wall-mounted tags: flat against wall surfaces, facing INTO the room
        # Use different box dimensions based on wall orientation (no yaw rotation needed)
        
        tag_id = 0
        
        # South wall (y = 0.01) - tags face NORTH, panel in XZ plane
        for x_pos in [0.5, 2.5, 4.5]:
            panel_sdf = self.get_box_sdf(0.20, 0.005, 0.20, 1.0, 1.0, 1.0)  # wide in X, thin in Y
            self.spawn_sdf(f'tag_bg_{tag_id}', panel_sdf, x_pos, 0.01, 0.15)
            pattern_sdf = self.get_box_sdf(0.14, 0.008, 0.14, 0.1, 0.1, 0.1)
            self.spawn_sdf(f'tag_pattern_{tag_id}', pattern_sdf, x_pos, 0.02, 0.15)
            tag_id += 1
        
        # North wall (y = 4.99) - tags face SOUTH, panel in XZ plane
        for x_pos in [0.5, 2.5, 4.5]:
            panel_sdf = self.get_box_sdf(0.20, 0.005, 0.20, 1.0, 1.0, 1.0)
            self.spawn_sdf(f'tag_bg_{tag_id}', panel_sdf, x_pos, 4.99, 0.15)
            pattern_sdf = self.get_box_sdf(0.14, 0.008, 0.14, 0.1, 0.1, 0.1)
            self.spawn_sdf(f'tag_pattern_{tag_id}', pattern_sdf, x_pos, 4.98, 0.15)
            tag_id += 1
        
        # West wall (x = 0.01) - tags face EAST, panel in YZ plane
        for y_pos in [0.5, 2.5, 4.5]:
            panel_sdf = self.get_box_sdf(0.005, 0.20, 0.20, 1.0, 1.0, 1.0)  # thin in X, wide in Y
            self.spawn_sdf(f'tag_bg_{tag_id}', panel_sdf, 0.01, y_pos, 0.15)
            pattern_sdf = self.get_box_sdf(0.008, 0.14, 0.14, 0.1, 0.1, 0.1)
            self.spawn_sdf(f'tag_pattern_{tag_id}', pattern_sdf, 0.02, y_pos, 0.15)
            tag_id += 1
        
        # East wall (x = 4.99) - tags face WEST, panel in YZ plane
        for y_pos in [0.5, 2.5, 4.5]:
            panel_sdf = self.get_box_sdf(0.005, 0.20, 0.20, 1.0, 1.0, 1.0)
            self.spawn_sdf(f'tag_bg_{tag_id}', panel_sdf, 4.99, y_pos, 0.15)
            pattern_sdf = self.get_box_sdf(0.008, 0.14, 0.14, 0.1, 0.1, 0.1)
            self.spawn_sdf(f'tag_pattern_{tag_id}', pattern_sdf, 4.98, y_pos, 0.15)
            tag_id += 1
        
        # Obstacle-mounted tags (vertical, on sides - flat against surface)
        self.get_logger().info('Spawning AprilTags on obstacles...')
        for (row, col) in self.obstacles:
            x = col * self.cell_size + self.cell_size / 2
            y = row * self.cell_size + self.cell_size / 2
            
            # South side (y - 0.46) - faces NORTH, panel in XZ plane
            panel_sdf = self.get_box_sdf(0.15, 0.005, 0.15, 1.0, 1.0, 1.0)
            self.spawn_sdf(f'tag_obs_bg_{tag_id}', panel_sdf, x, y - 0.46, 0.25)
            pattern_sdf = self.get_box_sdf(0.10, 0.008, 0.10, 0.1, 0.1, 0.1)
            self.spawn_sdf(f'tag_obs_pattern_{tag_id}', pattern_sdf, x, y - 0.45, 0.25)
            tag_id += 1
            
            # North side (y + 0.46) - faces SOUTH, panel in XZ plane
            panel_sdf = self.get_box_sdf(0.15, 0.005, 0.15, 1.0, 1.0, 1.0)
            self.spawn_sdf(f'tag_obs_bg_{tag_id}', panel_sdf, x, y + 0.46, 0.25)
            pattern_sdf = self.get_box_sdf(0.10, 0.008, 0.10, 0.1, 0.1, 0.1)
            self.spawn_sdf(f'tag_obs_pattern_{tag_id}', pattern_sdf, x, y + 0.45, 0.25)
            tag_id += 1
            
            # West side (x - 0.46) - faces EAST, panel in YZ plane
            panel_sdf = self.get_box_sdf(0.005, 0.15, 0.15, 1.0, 1.0, 1.0)
            self.spawn_sdf(f'tag_obs_bg_{tag_id}', panel_sdf, x - 0.46, y, 0.25)
            pattern_sdf = self.get_box_sdf(0.008, 0.10, 0.10, 0.1, 0.1, 0.1)
            self.spawn_sdf(f'tag_obs_pattern_{tag_id}', pattern_sdf, x - 0.45, y, 0.25)
            tag_id += 1
            
            # East side (x + 0.46) - faces WEST, panel in YZ plane
            panel_sdf = self.get_box_sdf(0.005, 0.15, 0.15, 1.0, 1.0, 1.0)
            self.spawn_sdf(f'tag_obs_bg_{tag_id}', panel_sdf, x + 0.46, y, 0.25)
            pattern_sdf = self.get_box_sdf(0.008, 0.10, 0.10, 0.1, 0.1, 0.1)
            self.spawn_sdf(f'tag_obs_pattern_{tag_id}', pattern_sdf, x + 0.45, y, 0.25)
            tag_id += 1
        
        # 6. START MARKER (green circle)
        self.get_logger().info('Spawning start/goal markers...')
        # start_cell = (row=0, col=0) -> world (0.5, 0.5)
        start_x = self.start_cell[1] * self.cell_size + self.cell_size / 2  # col -> x
        start_y = self.start_cell[0] * self.cell_size + self.cell_size / 2  # row -> y
        start_sdf = self.get_cylinder_sdf(0.25, 0.02, 0.0, 0.8, 0.0)
        self.spawn_sdf('start_marker', start_sdf, start_x, start_y, 0.01)
        
        # 7. GOAL MARKER (blue circle)
        # goal_cell = (row=4, col=2) -> world (2.5, 4.5)
        goal_x = self.goal_cell[1] * self.cell_size + self.cell_size / 2  # col -> x
        goal_y = self.goal_cell[0] * self.cell_size + self.cell_size / 2  # row -> y
        goal_sdf = self.get_cylinder_sdf(0.25, 0.02, 0.0, 0.0, 0.8)
        self.spawn_sdf('goal_marker', goal_sdf, goal_x, goal_y, 0.01)
        
        self.get_logger().info('='*50)
        self.get_logger().info('WORLD SPAWNING COMPLETE')
        self.get_logger().info(f'Grid: {self.grid_size}x{self.grid_size}')
        self.get_logger().info(f'Obstacles: {self.obstacles}')
        self.get_logger().info(f'AprilTags: {tag_id}')
        self.get_logger().info('='*50)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = WorldSpawner()
        # Spawn complete, shutdown
        node.destroy_node()
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

