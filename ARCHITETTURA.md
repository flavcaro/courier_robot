# Architettura del Sistema Courier Robot

## 📋 Panoramica

Il sistema utilizza **Nav2** (Navigation2) per la navigazione autonoma, integrato con un **Behavior Tree** (py_trees) per la logica di missione ad alto livello. Questa architettura combina la robustezza di Nav2 per path planning e controllo con la flessibilità di py_trees per la gestione delle missioni.

---

## 🏗️ Struttura dei Package

### 1️⃣ **courier_description**
Contiene la descrizione del robot (URDF/SDF).

**File principali:**
- `urdf/courier.urdf.xacro` - Descrizione fisica del robot
- `launch/sim.launch.py` - Launch file per Gazebo + Bridge ROS2

---

### 2️⃣ **courier_nav** 🗺️

**Responsabilità:** Navigazione con Nav2 e logica di missione con Behavior Tree.

**Nodi principali:**
- `nav2_mission_controller` - Controller della missione con behavior tree
- `spawner` - Spawner della griglia di celle in Gazebo

**Cosa fa:**
- ✅ Utilizza **Nav2** per path planning globale (NavFn) e locale (DWB)
- ✅ Gestisce la missione con un **Behavior Tree** (py_trees)
- ✅ Invia goal di navigazione a Nav2 tramite action client
- ✅ Gestisce raccolta e consegna oggetti
- ✅ Visualizza la griglia in RViz

**Configurazione Nav2:**
- `config/nav2_params.yaml` - Parametri completi di Nav2
- `maps/courier_map.yaml` - Mappa dell'ambiente
- `maps/courier_map.pgm` - Occupancy grid (100x100 pixel)

---

## 🤖 Stack Nav2

Nav2 fornisce navigazione autonoma completa:

| Componente | Plugin | Funzione |
|------------|--------|----------|
| **Global Planner** | NavFn | Calcola percorso ottimale sulla mappa |
| **Local Controller** | DWB | Genera comandi velocità smooth |
| **Costmap Global** | Static + Obstacle + Inflation | Mappa con ostacoli |
| **Costmap Local** | Voxel + Inflation | Ostacoli dinamici (rolling window) |
| **Recovery** | Spin, Backup, Wait | Comportamenti di recupero |
| **Localization** | AMCL | Localizzazione con particle filter |
| **BT Navigator** | BehaviorTree.CPP | Behavior tree interno Nav2 |

**Vantaggi rispetto a BFS custom:**
- ✅ Path smoothing e curve ottimizzate
- ✅ Obstacle avoidance dinamico
- ✅ Recovery behaviors automatici
- ✅ Velocity smoothing per movimenti fluidi
- ✅ Replanning automatico se bloccato

---

## 🌳 Behavior Tree (py_trees)

Il behavior tree gestisce la **logica di missione** ad alto livello, delegando la navigazione a Nav2:

```
Root (Selector)
├── Mission Complete? ────────────────────► SUCCESS se missione finita
└── Main Mission (Sequence)
    ├── Check Battery ────────────────────► FAILURE se batteria < 10%
    ├── Go To Pickup (Selector)
    │   ├── Already Collected? ───────────► Skip se già raccolto
    │   └── Navigate To Pickup (Repeat)
    │       ├── Get Pickup Waypoint ──────► Pop dalla coda
    │       └── Nav2 To Pickup ───────────► Action client Nav2
    ├── Collect Object ───────────────────► Simula raccolta (2 sec)
    ├── Plan Return (Selector)
    │   ├── Return Planned? ──────────────► Skip se già pianificato
    │   └── Plan Return Path ─────────────► Imposta destinazione home
    ├── Navigate Home (Sequence)
    │   ├── Get Home Waypoint ────────────► Pop dalla coda
    │   └── Nav2 To Home ─────────────────► Action client Nav2
    └── Deliver Object ───────────────────► Simula consegna (2 sec)
```

**Behavior key:**

| Behavior | Tipo | Descrizione |
|----------|------|-------------|
| `NavigateToCell` | Action | Invia goal a Nav2, monitora completamento |
| `GetNextWaypoint` | Action | Estrae prossimo waypoint dalla coda |
| `CheckBattery` | Condition | Verifica livello batteria |
| `CollectObject` | Action | Simula raccolta oggetto |
| `DeliverObject` | Action | Simula consegna oggetto |
| `PlanReturnPath` | Action | Pianifica percorso di ritorno |

---

## 🔄 Flusso di Comunicazione

```
┌─────────────────────────────────────────────────────────────────┐
│                     BEHAVIOR TREE (py_trees)                    │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │  Mission Controller                                        │  │
│  │  • Gestisce stati missione                                │  │
│  │  • Monitora battery, object status                        │  │
│  │  • Decide quando navigare/raccogliere/consegnare          │  │
│  └───────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
                              │
                              │ NavigateToPose Action
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                        NAV2 STACK                               │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐  │
│  │   Planner   │  │ Controller  │  │   Behavior Server       │  │
│  │   (NavFn)   │  │   (DWB)     │  │ (Spin/Backup/Wait)      │  │
│  └──────┬──────┘  └──────┬──────┘  └─────────────────────────┘  │
│         │                │                                      │
│         ▼                ▼                                      │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │              Costmap2D (Global + Local)                  │   │
│  └─────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
                              │
                              │ /cmd_vel
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                     GAZEBO SIMULATION                           │
│  • Robot (differential drive)                                   │
│  • LiDAR sensor (/scan)                                        │
│  • Odometry (/odom)                                            │
│  • Grid world with obstacles                                    │
└─────────────────────────────────────────────────────────────────┘
```

---

## 📡 Topic e Action

### Action Servers (Nav2)
| Action | Tipo | Descrizione |
|--------|------|-------------|
| `/navigate_to_pose` | NavigateToPose | Naviga a una posa specifica |
| `/navigate_through_poses` | NavigateThroughPoses | Naviga attraverso waypoint |
| `/follow_waypoints` | FollowWaypoints | Segue lista di waypoint |

### Topic Principali
| Topic | Tipo | Direzione | Descrizione |
|-------|------|-----------|-------------|
| `/cmd_vel` | Twist | Nav2 → Robot | Comandi velocità |
| `/odom` | Odometry | Robot → Nav2 | Odometria |
| `/scan` | LaserScan | Robot → Nav2 | Dati LiDAR |
| `/map` | OccupancyGrid | Map Server → All | Mappa statica |
| `/global_costmap/costmap` | OccupancyGrid | Nav2 → RViz | Costmap globale |
| `/local_costmap/costmap` | OccupancyGrid | Nav2 → RViz | Costmap locale |
| `/grid_markers` | MarkerArray | Mission → RViz | Visualizzazione griglia |

---

## 🚀 Come Avviare il Sistema

### Metodo 1: Script automatico (consigliato)
```bash
# Nel container Docker
./start_nav2.sh
```

Questo script:
1. Installa Nav2 se non presente
2. Builda il workspace
3. Avvia Gazebo + Bridge
4. Spawna griglia e robot
5. Avvia Nav2 stack
6. Avvia mission controller

### Metodo 2: Manuale (per debug)

#### 1. Build del workspace
```bash
cd ros2_ws
colcon build --packages-select courier_nav courier_description
source install/setup.bash
```

#### 2. Avvia simulazione
```bash
ros2 launch courier_description sim.launch.py
```

#### 3. Spawna il robot (in altro terminale)
```bash
ros2 run ros_gz_sim create -world empty -file robot.sdf -name courier_robot -x 0.5 -y 0.5 -z 0.15
```

#### 4. Avvia Nav2 (in altro terminale)
```bash
ros2 launch courier_nav nav2_bringup.launch.py use_sim_time:=true
```

#### 5. Avvia mission controller (in altro terminale)
```bash
ros2 run courier_nav nav2_mission_controller
```

---

## 📊 Monitoraggio

### Visualizzare in RViz2
```bash
rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz
```

### Status navigazione Nav2
```bash
ros2 topic echo /navigate_to_pose/_action/status
```

### Comandi velocità
```bash
ros2 topic echo /cmd_vel
```

### Costmap globale
```bash
ros2 topic echo /global_costmap/costmap
```

### Behavior Tree status (nei log)
Il mission controller stampa:
- 🌳 Struttura del behavior tree all'avvio
- 📍 Navigazione verso waypoint
- ✅ Waypoint raggiunto
- 📦 Oggetto raccolto/consegnato

---

## 🔧 Parametri Configurabili

### Nav2 Parameters ([config/nav2_params.yaml](ros2_ws/src/courier_nav/config/nav2_params.yaml))

**Controller (DWB):**
```yaml
max_vel_x: 0.26        # Velocità lineare max
max_vel_theta: 1.0     # Velocità angolare max
xy_goal_tolerance: 0.15 # Tolleranza posizione goal
```

**Planner (NavFn):**
```yaml
tolerance: 0.5         # Tolleranza planning
use_astar: false       # Usa Dijkstra (più robusto)
```

**Costmap:**
```yaml
robot_radius: 0.15     # Raggio robot per inflazione
inflation_radius: 0.35 # Raggio zona di sicurezza
resolution: 0.05       # Risoluzione mappa (m/pixel)
```

### Mission Controller ([nav2_mission_controller.py](ros2_ws/src/courier_nav/courier_nav/nav2_mission_controller.py))
```python
cell_size = 1.0        # Dimensione cella (metri)
start_cell = (0, 0)    # Cella di partenza
goal_cell = (4, 2)     # Cella pickup
```

---

## 📐 Mappa dell'Ambiente

**Griglia 5x5** (celle da 1m):

```
     Col 0   Col 1   Col 2   Col 3   Col 4
    ┌───────┬───────┬───────┬───────┬───────┐
Row 0│ START │       │       │       │       │
    ├───────┼───────┼───────┼───────┼───────┤
Row 1│       │  ███  │  ███  │       │       │
    ├───────┼───────┼───────┼───────┼───────┤
Row 2│       │       │       │       │       │
    ├───────┼───────┼───────┼───────┼───────┤
Row 3│       │  ███  │       │  ███  │       │
    ├───────┼───────┼───────┼───────┼───────┤
Row 4│       │       │ GOAL  │       │       │
    └───────┴───────┴───────┴───────┴───────┘

███ = Ostacolo
START = Cella (0,0) - Partenza e consegna
GOAL = Cella (4,2) - Punto di raccolta
```

---

## ✅ Vantaggi dell'Architettura Nav2

| Aspetto | Prima (BFS custom) | Ora (Nav2) |
|---------|-------------------|------------|
| **Path Planning** | BFS su griglia discreta | NavFn con smooth paths |
| **Controllo** | PID custom | DWB con velocity smoothing |
| **Ostacoli** | Solo statici da mappa | Dinamici con costmap |
| **Recovery** | Nessuno | Spin, Backup, Wait automatici |
| **Movimento** | Rotate-Move-Rotate | Curve fluide continue |
| **Replanning** | Manuale | Automatico se bloccato |
| **Scalabilità** | Limitata | Pronto per ambienti reali |

---

## 🐛 Troubleshooting

### Nav2 non si avvia
1. Verificare installazione: `ros2 pkg list | grep nav2`
2. Se mancante: `sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup`

### Il robot non si muove
1. Verificare TF tree: `ros2 run tf2_tools view_frames`
2. Deve esistere: `map → odom → base_link`
3. Controllare costmap: `ros2 topic echo /global_costmap/costmap`

### Navigation goal rejected
1. Verificare che il goal sia in area libera della costmap
2. Controllare log di Nav2: cercare "rejected" o "failed"

### Behavior tree non avanza
1. Controllare log del mission controller
2. Verificare blackboard values
3. Stampare tree status con `py_trees.display.unicode_tree()`

---

## 📚 Riferimenti

- [Nav2 Documentation](https://docs.nav2.org/)
- [py_trees Documentation](https://py-trees.readthedocs.io/)
- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [BehaviorTree.CPP](https://www.behaviortree.dev/) (usato internamente da Nav2)
