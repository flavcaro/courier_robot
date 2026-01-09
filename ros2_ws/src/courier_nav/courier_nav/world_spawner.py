#!/usr/bin/env python3
"""
World Spawner using gz service with parallel spawning (Gazebo Harmonic)
- Griglia 5x5 con pavimento a scacchiera
- Muri di confine
- Ostacoli che riempiono le celle
- AprilTag sui bordi delle celle

✨ IMPROVEMENT: Spawn all objects in parallel using background processes
"""

import rclpy
from rclpy.node import Node
import subprocess
import time
import math
import os
from ament_index_python.packages import get_package_share_directory
from concurrent.futures import ThreadPoolExecutor, as_completed


class WorldSpawner(Node):
    def __init__(self):
        super().__init__('world_spawner')
        
        self.grid_size = 5
        self.cell_size = 1.0
        self.obstacles = [(1, 1), (1, 2), (3, 1), (3, 3)]
        self.start_cell = (0, 0)
        self.goal_cell = (4, 2)
        
        self.spawn_counter = 0
        self.spawn_queue = []  # Queue of spawn tasks
        
        # ✨ Cache apriltag directory path to avoid repeated ROS lookups
        try:
            pkg_share = get_package_share_directory('courier_nav')
            self.apriltag_dir = os.path.join(pkg_share, 'apriltag_images')
        except:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            self.apriltag_dir = os.path.join(current_dir, 'apriltag_images')
        
        self.get_logger().info('World Spawner starting...')
        time.sleep(2.0)
        
        self.spawn_world()
    
    def spawn_sdf(self, name: str, sdf: str, x: float, y: float, z: float = 0.0, yaw: float = 0.0):
        """Queue an entity for spawning via gz service."""
        # Convert yaw to quaternion
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        
        # Escape SDF for command line
        sdf_escaped = sdf.replace('\n', ' ').replace('"', '\\"').replace("'", "\\'")
        
        req = f'sdf: "{sdf_escaped}", name: "{name}", pose: {{position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{z: {qz}, w: {qw}}}}}'
        
        cmd = [
            'gz', 'service', '-s', '/world/empty/create',
            '--reqtype', 'gz.msgs.EntityFactory',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '3000',
            '--req', req
        ]
        
        # Queue the spawn command (will execute in parallel)
        return (name, cmd)
    
    def execute_spawn_parallel(self, spawn_tasks, max_workers=8):
        """Execute all spawn tasks in parallel using ThreadPoolExecutor."""
        self.get_logger().info(f'Spawning {len(spawn_tasks)} objects in parallel (max_workers={max_workers})...')
        
        success_count = 0
        fail_count = 0
        
        with ThreadPoolExecutor(max_workers=max_workers) as executor:
            # Submit all tasks
            future_to_task = {executor.submit(self._run_spawn_cmd, cmd, name): name 
                            for name, cmd in spawn_tasks}
            
            # Process completed tasks
            for i, future in enumerate(as_completed(future_to_task), 1):
                name = future_to_task[future]
                try:
                    result = future.result()
                    if result:
                        success_count += 1
                        # Throttle logging - only show every 10th success
                        if success_count % 10 == 0:
                            self.get_logger().info(f'✓ Spawned {success_count} objects...')
                    else:
                        fail_count += 1
                except Exception as e:
                    fail_count += 1
                    self.get_logger().warn(f'Failed to spawn {name}: {e}')
        
        self.get_logger().info(f'Spawning complete: {success_count} success, {fail_count} failed')
        return success_count, fail_count
    
    def _run_spawn_cmd(self, cmd, name):
        """Execute a single spawn command (called by executor)."""
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            return result.returncode == 0
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
    
    def get_apriltag_sdf(self, tag_id: int, size: float, thickness: float, orientation: str = 'XZ') -> str:
        """Generate SDF for an AprilTag marker."""
        
        if orientation == 'XZ':
            panel_dims = f'{size} {thickness} {size}'
        else:  # YZ
            panel_dims = f'{thickness} {size} {size}'
        
        # Use cached apriltag directory (set in __init__)
        tag_image_path = f'file://{self.apriltag_dir}/tag_{tag_id}.png'
        
        return f'''<?xml version="1.0"?>
<sdf version="1.8">
  <model name="apriltag_{tag_id}">
    <static>true</static>
    <link name="link">
      <visual name="tag_visual">
        <geometry><box><size>{panel_dims}</size></box></geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <pbr>
            <metal>
              <albedo_map>{tag_image_path}</albedo_map>
            </metal>
          </pbr>
        </material>
      </visual>
      <visual name="id_marker">
        <pose>0 0 {size*0.55} 0 0 0</pose>
        <geometry><box><size>{size*0.15} {thickness*2.0} {size*0.08}</size></box></geometry>
        <material>
          <ambient>1 0.5 0 1</ambient>
          <diffuse>1 0.5 0 1</diffuse>
          <emissive>0.8 0.4 0 1</emissive>
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
        """Spawn all world elements in parallel."""
        self.get_logger().info('='*50)
        self.get_logger().info('SPAWNING WORLD ELEMENTS (PARALLEL)')
        self.get_logger().info('='*50)
        
        grid_length = self.grid_size * self.cell_size
        
        # 1. FLOOR TILES
        self.get_logger().info('Queueing floor tiles...')
        for i in range(self.grid_size):
            for j in range(self.grid_size):
                is_white = (i + j) % 2 == 0
                color = (0.9, 0.9, 0.9) if is_white else (0.6, 0.6, 0.6)
                x = i * self.cell_size + self.cell_size / 2
                y = j * self.cell_size + self.cell_size / 2
                
                sdf = self.get_box_sdf(0.98, 0.98, 0.01, *color)
                task = self.spawn_sdf(f'floor_{i}_{j}', sdf, x, y, -0.005)
                self.spawn_queue.append(task)
        
        # 2. GRID LINES
        self.get_logger().info('Queueing grid lines...')
        for i in range(self.grid_size + 1):
            sdf_h = self.get_box_sdf(grid_length, 0.02, 0.01, 0, 0, 0)
            task_h = self.spawn_sdf(f'hline_{i}', sdf_h, grid_length/2, i * self.cell_size, 0.01)
            self.spawn_queue.append(task_h)
            
            sdf_v = self.get_box_sdf(0.02, grid_length, 0.01, 0, 0, 0)
            task_v = self.spawn_sdf(f'vline_{i}', sdf_v, i * self.cell_size, grid_length/2, 0.01)
            self.spawn_queue.append(task_v)
        
        # 3. BOUNDARY WALLS
        self.get_logger().info('Queueing boundary walls...')
        wall_height = 0.4
        wall_thickness = 0.1
        sdf = self.get_box_sdf(grid_length + 0.2, wall_thickness, wall_height, 0.2, 0.2, 0.8, True)
        
        self.spawn_queue.append(self.spawn_sdf('wall_south', sdf, grid_length/2, -wall_thickness/2, wall_height/2))
        self.spawn_queue.append(self.spawn_sdf('wall_north', sdf, grid_length/2, grid_length + wall_thickness/2, wall_height/2))
        
        sdf = self.get_box_sdf(wall_thickness, grid_length + 0.2, wall_height, 0.2, 0.2, 0.8, True)
        self.spawn_queue.append(self.spawn_sdf('wall_west', sdf, -wall_thickness/2, grid_length/2, wall_height/2))
        self.spawn_queue.append(self.spawn_sdf('wall_east', sdf, grid_length + wall_thickness/2, grid_length/2, wall_height/2))
        
        # 4. OBSTACLES
        self.get_logger().info('Queueing obstacles...')
        obstacle_sdf = self.get_box_sdf(0.9, 0.9, 0.5, 0.8, 0.1, 0.1, True)
        
        for (row, col) in self.obstacles:
            x = col * self.cell_size + self.cell_size / 2
            y = row * self.cell_size + self.cell_size / 2
            
            self.spawn_queue.append(self.spawn_sdf(f'obstacle_{row}_{col}', obstacle_sdf, x, y, 0.25))
            
            # X markers
            x_sdf = self.get_box_sdf(0.6, 0.08, 0.05, 1.0, 0.5, 0.0)
            self.spawn_queue.append(self.spawn_sdf(f'x1_{row}_{col}', x_sdf, x, y, 0.55, 0.785))
            self.spawn_queue.append(self.spawn_sdf(f'x2_{row}_{col}', x_sdf, x, y, 0.55, -0.785))
        
        # 5. APRILTAG MARKERS
        self.get_logger().info('Queueing AprilTag markers...')
        tag_id = 0
        tag_size = 0.20
        
        # South wall
        for x_pos in [0.5, 2.5, 4.5]:
            tag_sdf = self.get_apriltag_sdf(tag_id, tag_size, 0.01, 'XZ')
            self.spawn_queue.append(self.spawn_sdf(f'apriltag_{tag_id}', tag_sdf, x_pos, 0.02, 0.15))
            tag_id += 1
        
        # North wall
        for x_pos in [0.5, 2.5, 4.5]:
            tag_sdf = self.get_apriltag_sdf(tag_id, tag_size, 0.01, 'XZ')
            self.spawn_queue.append(self.spawn_sdf(f'apriltag_{tag_id}', tag_sdf, x_pos, 4.98, 0.15))
            tag_id += 1
        
        # West wall
        for y_pos in [0.5, 2.5, 4.5]:
            tag_sdf = self.get_apriltag_sdf(tag_id, tag_size, 0.01, 'YZ')
            self.spawn_queue.append(self.spawn_sdf(f'apriltag_{tag_id}', tag_sdf, 0.02, y_pos, 0.15))
            tag_id += 1
        
        # East wall
        for y_pos in [0.5, 2.5, 4.5]:
            tag_sdf = self.get_apriltag_sdf(tag_id, tag_size, 0.01, 'YZ')
            self.spawn_queue.append(self.spawn_sdf(f'apriltag_{tag_id}', tag_sdf, 4.98, y_pos, 0.15))
            tag_id += 1
        
        # Obstacle-mounted tags
        tag_size_obs = 0.15
        for (row, col) in self.obstacles:
            x = col * self.cell_size + self.cell_size / 2
            y = row * self.cell_size + self.cell_size / 2
            
            # South side
            tag_sdf = self.get_apriltag_sdf(tag_id, tag_size_obs, 0.01, 'XZ')
            self.spawn_queue.append(self.spawn_sdf(f'apriltag_{tag_id}', tag_sdf, x, y - 0.45, 0.25))
            tag_id += 1
            
            # North side
            tag_sdf = self.get_apriltag_sdf(tag_id, tag_size_obs, 0.01, 'XZ')
            self.spawn_queue.append(self.spawn_sdf(f'apriltag_{tag_id}', tag_sdf, x, y + 0.45, 0.25))
            tag_id += 1
            
            # West side
            tag_sdf = self.get_apriltag_sdf(tag_id, tag_size_obs, 0.01, 'YZ')
            self.spawn_queue.append(self.spawn_sdf(f'apriltag_{tag_id}', tag_sdf, x - 0.45, y, 0.25))
            tag_id += 1
            
            # East side
            tag_sdf = self.get_apriltag_sdf(tag_id, tag_size_obs, 0.01, 'YZ')
            self.spawn_queue.append(self.spawn_sdf(f'apriltag_{tag_id}', tag_sdf, x + 0.45, y, 0.25))
            tag_id += 1
        
        # 6. START MARKER
        self.get_logger().info('Queueing markers...')
        start_x = self.start_cell[1] * self.cell_size + self.cell_size / 2
        start_y = self.start_cell[0] * self.cell_size + self.cell_size / 2
        start_sdf = self.get_cylinder_sdf(0.25, 0.02, 0.0, 0.8, 0.0)
        self.spawn_queue.append(self.spawn_sdf('start_marker', start_sdf, start_x, start_y, 0.01))
        
        # 7. GOAL MARKER
        goal_x = self.goal_cell[1] * self.cell_size + self.cell_size / 2
        goal_y = self.goal_cell[0] * self.cell_size + self.cell_size / 2
        goal_sdf = self.get_cylinder_sdf(0.25, 0.02, 0.0, 0.0, 0.8)
        self.spawn_queue.append(self.spawn_sdf('goal_marker', goal_sdf, goal_x, goal_y, 0.01))
        
        # === EXECUTE ALL SPAWNS IN PARALLEL ===
        self.get_logger().info(f'Total objects to spawn: {len(self.spawn_queue)}')
        success, fail = self.execute_spawn_parallel(self.spawn_queue, max_workers=12)
        
        self.get_logger().info('='*50)
        self.get_logger().info('WORLD SPAWNING COMPLETE')
        self.get_logger().info(f'Grid: {self.grid_size}x{self.grid_size}')
        self.get_logger().info(f'Obstacles: {self.obstacles}')
        self.get_logger().info(f'AprilTags: {tag_id}')
        self.get_logger().info(f'Result: {success} spawned, {fail} failed')
        self.get_logger().info('='*50)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = WorldSpawner()
        node.destroy_node()
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

