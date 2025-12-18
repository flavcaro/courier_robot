#!/usr/bin/env python3
"""
Script to generate a PGM map file from the grid map.
Run this once to create the map for Nav2.
"""

# Grid map from courier_controller.py (0=free, 1=obstacle)
# 5x5 grid with 1m cell size
grid_map = [
    [0, 0, 0, 0, 0],
    [0, 1, 1, 0, 0],
    [0, 0, 0, 0, 0],
    [0, 1, 0, 1, 0],
    [0, 0, 0, 0, 0]
]

# Map parameters
cell_size = 1.0  # meters
resolution = 0.05  # meters per pixel
pixels_per_cell = int(cell_size / resolution)  # 20 pixels per cell

# Calculate map dimensions
grid_rows = len(grid_map)
grid_cols = len(grid_map[0])
map_height = grid_rows * pixels_per_cell
map_width = grid_cols * pixels_per_cell

# Create the map image (white = free, black = obstacle)
# PGM values: 254 = free, 0 = occupied
map_image = [[254 for _ in range(map_width)] for _ in range(map_height)]

# Fill in obstacles
for row in range(grid_rows):
    for col in range(grid_cols):
        if grid_map[row][col] == 1:  # Obstacle
            # Calculate pixel coordinates
            y_start = row * pixels_per_cell
            y_end = (row + 1) * pixels_per_cell
            x_start = col * pixels_per_cell
            x_end = (col + 1) * pixels_per_cell
            for y in range(y_start, y_end):
                for x in range(x_start, x_end):
                    map_image[y][x] = 0

# Add walls around the perimeter
wall_thickness = 2  # pixels
for y in range(map_height):
    for x in range(map_width):
        if y < wall_thickness or y >= map_height - wall_thickness:
            map_image[y][x] = 0
        if x < wall_thickness or x >= map_width - wall_thickness:
            map_image[y][x] = 0

# Save as PGM file
pgm_path = "courier_map.pgm"
with open(pgm_path, 'wb') as f:
    # PGM header
    f.write(f"P5\n{map_width} {map_height}\n255\n".encode())
    # Image data
    for row in map_image:
        f.write(bytes(row))

print(f"Map saved to {pgm_path}")
print(f"Map dimensions: {map_width}x{map_height} pixels")
print(f"Resolution: {resolution} m/pixel")
print(f"Grid size: {grid_cols}x{grid_rows} cells")
