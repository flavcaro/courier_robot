#!/usr/bin/env python3
"""
Test BFS Path Planning
Verifica che il percorso BFS sia calcolato correttamente
"""

from collections import deque
import math

def compute_bfs_path(grid_map, start, goal):
    """Compute BFS path from start to goal"""
    rows = len(grid_map)
    cols = len(grid_map[0])
    
    queue = deque([start])
    visited = {start}
    parent = {start: None}
    
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    
    while queue:
        current = queue.popleft()
        
        if current == goal:
            path = []
            while current is not None:
                path.append(current)
                current = parent[current]
            return list(reversed(path))
        
        r, c = current
        for dr, dc in directions:
            nr, nc = r + dr, c + dc
            
            if 0 <= nr < rows and 0 <= nc < cols:
                if grid_map[nr][nc] == 0 and (nr, nc) not in visited:
                    visited.add((nr, nc))
                    parent[(nr, nc)] = current
                    queue.append((nr, nc))
    
    return []

# Test con la griglia reale
grid_map = [
    [0, 0, 0, 0, 0],  # Row 0
    [0, 1, 1, 0, 0],  # Row 1
    [0, 0, 0, 0, 0],  # Row 2
    [0, 1, 0, 1, 0],  # Row 3
    [0, 0, 0, 0, 0]   # Row 4
]

start = (0, 0)  # Cella verde
goal = (4, 2)   # Cella blu

path = compute_bfs_path(grid_map, start, goal)

print("=" * 60)
print(f"📍 BFS Path Test: Cell{start} → Cell{goal}")
print("=" * 60)
print(f"\n✅ Path trovato: {len(path)} waypoints\n")

total_distance = 0.0

for i, cell in enumerate(path):
    row, col = cell
    x = (col + 0.5)  # X = colonna
    y = (row + 0.5)  # Y = riga
    
    # Calcola distanza dal punto precedente
    if i > 0:
        prev_row, prev_col = path[i-1]
        prev_x = (prev_col + 0.5)
        prev_y = (prev_row + 0.5)
        dist = math.sqrt((x - prev_x)**2 + (y - prev_y)**2)
        total_distance += dist
        print(f"  {i+1:2d}. Cell({row},{col}) → World({x:.2f},{y:.2f}) [+{dist:.2f}m]")
    else:
        print(f"  {i+1:2d}. Cell({row},{col}) → World({x:.2f},{y:.2f}) [START 🟢]")

print(f"\n📏 Distanza totale: {total_distance:.2f}m ({len(path)-1} celle)")

# Visualizza griglia
print("\n🗺️  Grid Visualization:")
print("    ", end="")
for c in range(len(grid_map[0])):
    print(f"Col{c} ", end="")
print()

for r in range(len(grid_map)):
    row_str = f"Row{r}: "
    for c in range(len(grid_map[0])):
        if (r, c) == start:
            row_str += '🟢   '
        elif (r, c) == goal:
            row_str += '🔵   '
        elif (r, c) in path:
            idx = path.index((r, c))
            row_str += f'{idx:2d}   '
        elif grid_map[r][c] == 1:
            row_str += '🟥   '
        else:
            row_str += '⬜   '
    print(row_str)

print("\n" + "=" * 60)
print("🎯 Configurazione:")
print(f"   Robot spawna a (0.5, 0.5) = centro di Cell(0,0) 🟢")
print(f"   Destinazione: (2.5, 4.5) = centro di Cell(4,2) 🔵")
print(f"   Tolleranza: 0.15m (15cm)")
print("=" * 60)

# Verifica path
expected_path = [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2), (3, 2), (4, 2)]
if path == expected_path:
    print("✅ Path corretto!")
else:
    print("❌ Path NON corrisponde all'atteso!")
    print(f"   Atteso: {expected_path}")
    print(f"   Ottenuto: {path}")
