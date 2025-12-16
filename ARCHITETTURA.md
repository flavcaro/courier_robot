# Architettura del Sistema Courier Robot

## 📋 Panoramica

Il sistema è stato riorganizzato secondo l'architettura modulare specificata nel Project Proposal, con separazione delle responsabilità tra **Path Planning** e **Controllo**.

---

## 🏗️ Struttura dei Package

### 1️⃣ **courier_description**
Contiene la descrizione del robot (URDF/SDF).

**File principali:**
- `urdf/courier.urdf.xacro` - Descrizione fisica del robot
- `launch/sim.launch.py` - Launch file per Gazebo

---

### 2️⃣ **courier_control** ⚙️

**Responsabilità:** Controllo a basso livello del movimento del robot.

**Nodo principale:** `pid_controller`

**Cosa fa:**
- ✅ Implementa un controllore **PID** per movimento lineare e angolare
- ✅ Riceve target pose da `/target_pose` (pubblicato dal path planner)
- ✅ Pubblica comandi di velocità su `/cmd_vel`
- ✅ Notifica quando un target è raggiunto su `/target_reached`

**Topic:**
- **Subscribe:** `/target_pose` (PoseStamped) - Target da raggiungere
- **Subscribe:** `/odom` (Odometry) - Odometria del robot
- **Publish:** `/cmd_vel` (Twist) - Comandi di velocità
- **Publish:** `/target_reached` (PoseStamped) - Notifica target raggiunto

**Controllo PID:**
```
PID Lineare:
- Kp = 0.5
- Ki = 0.0
- Kd = 0.1

PID Angolare:
- Kp = 1.0
- Ki = 0.0
- Kd = 0.2
```

**Strategia di controllo:**
1. **Fase 1:** Rotazione in-place fino ad allineamento con il target
2. **Fase 2:** Movimento lineare con correzione angolare ridotta

---

### 3️⃣ **courier_nav** 🗺️

**Responsabilità:** Path planning e decisioni ad alto livello.

**Nodo principale:** `path_planner`

**Cosa fa:**
- ✅ Calcola il percorso con algoritmo **BFS** (Breadth-First Search)
- ✅ Gestisce le decisioni con un **Behavior Tree**
- ✅ Pubblica target pose su `/target_pose` per il controllore PID
- ✅ Gestisce la batteria simulata e ricarica
- ✅ Visualizza la griglia e il percorso in RViz

**Topic:**
- **Subscribe:** `/odom` (Odometry) - Per conoscere la posizione del robot
- **Subscribe:** `/target_reached` (PoseStamped) - Per sapere quando avanzare al prossimo waypoint
- **Publish:** `/target_pose` (PoseStamped) - Prossimo waypoint da raggiungere
- **Publish:** `/grid_markers` (MarkerArray) - Visualizzazione in RViz

**Behavior Tree:**
```
Root (Sequence)
├── Battery Manager (Selector)
│   ├── BatteryOK (condizione >= 20%)
│   └── GoCharge (azione)
└── Mission Selector (Selector)
    ├── Navigation (se ha target attivo)
    │   ├── HasTarget
    │   └── Navigate to Target
    │       ├── IsAtTarget → ClearTarget
    │       └── Move Sequence
    │           ├── IsAligned? → RotateToTarget (monitoraggio)
    │           └── MoveToTarget (monitoraggio)
    ├── Get Next Waypoint (se coda non vuota)
    │   ├── HasPathQueue
    │   └── GetNextTarget (pubblica nuovo target)
    └── StopRobot (missione completata)
```

---

## 🔄 Flusso di Comunicazione

```
┌─────────────────┐         /target_pose          ┌─────────────────┐
│  Path Planner   │ ─────────────────────────────> │ PID Controller  │
│  (courier_nav)  │                                │(courier_control)│
│                 │ <───────────────────────────── │                 │
└─────────────────┘      /target_reached          └─────────────────┘
         │                                                   │
         │ /odom (subscribe)                     /cmd_vel (publish)
         │                                                   │
         └───────────────────────────────────────────────────┘
                                │
                                ▼
                          [ Robot in Gazebo ]
```

---

## 🚀 Come Avviare il Sistema

### 1. Build del workspace
```bash
cd ros2_ws
colcon build --packages-select courier_control courier_nav courier_description
source install/setup.bash
```

### 2. Avvia Gazebo e il robot
```bash
ros2 launch courier_description sim.launch.py
```

### 3. Avvia il sistema di navigazione (in un nuovo terminale)
```bash
cd ros2_ws
source install/setup.bash
ros2 launch courier_nav courier_mission.launch.py
```

Questo avvierà **entrambi** i nodi:
- `pid_controller` (courier_control)
- `path_planner` (courier_nav)

---

## 🧪 Test Individuali

### Solo PID Controller
```bash
ros2 run courier_control pid_controller
```

### Solo Path Planner
```bash
ros2 run courier_nav path_planner
```

### Pubblicare un target manualmente (per testare il PID)
```bash
ros2 topic pub /target_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'odom'},
  pose: {position: {x: 2.0, y: 2.0, z: 0.0}}
}"
```

---

## 📊 Monitoraggio

### Visualizzare i topic attivi
```bash
ros2 topic list
```

### Vedere i messaggi su /target_pose
```bash
ros2 topic echo /target_pose
```

### Vedere i comandi di velocità
```bash
ros2 topic echo /cmd_vel
```

### Vedere la batteria (nei log del path_planner)
I log mostrano:
- 📍 Nuovo target pubblicato
- ✅ Target raggiunto
- 🔋 Livello batteria

---

## 🔧 Parametri Configurabili

### PID Controller ([pid_controller.py](ros2_ws/src/courier_control/courier_control/pid_controller.py))
- `kp_linear`, `ki_linear`, `kd_linear` - Guadagni PID lineari
- `kp_angular`, `ki_angular`, `kd_angular` - Guadagni PID angolari
- `dist_tolerance` - Tolleranza distanza (default: 0.15m)
- `angle_tolerance` - Tolleranza angolare (default: 0.15 rad)

### Path Planner ([courier_controller.py](ros2_ws/src/courier_nav/courier_nav/courier_controller.py))
- `cell_size` - Dimensione cella griglia (default: 1.0m)
- `grid_map` - Mappa con ostacoli (0=libero, 1=ostacolo)
- `start_cell`, `goal_cell` - Celle di partenza e arrivo
- `battery_threshold` - Soglia batteria per ricarica (default: 20%)

---

## 📝 Differenze rispetto all'architettura precedente

| Aspetto | Prima | Dopo |
|---------|-------|------|
| **Controllo movimento** | Tutto in `courier_controller` | Separato in `pid_controller` |
| **Path planning** | Integrato con controllo | In `path_planner` (courier_nav) |
| **Comunicazione** | Diretta con `/cmd_vel` | Tramite `/target_pose` |
| **Architettura** | Monolitica | Modulare (come da PDF) |
| **Responsabilità** | Mescolate | Chiare e separate |

---

## ✅ Conformità al Project Proposal

✔️ **courier_control** implementa controllore PID  
✔️ **courier_nav** gestisce path planning e behavior tree  
✔️ **courier_description** contiene modello robot  
✔️ Separazione chiara tra path planning e controllo  
✔️ Comunicazione tramite topic ROS2 standard  

---

## 🐛 Troubleshooting

### Il robot non si muove
1. Verificare che entrambi i nodi siano attivi: `ros2 node list`
2. Controllare i topic: `ros2 topic list`
3. Verificare che `/target_pose` riceva messaggi: `ros2 topic echo /target_pose`

### Il PID è troppo aggressivo
Ridurre i guadagni `kp_linear` e `kp_angular` in [pid_controller.py](ros2_ws/src/courier_control/courier_control/pid_controller.py)

### Il robot non raggiunge il target
Aumentare le tolleranze `dist_tolerance` e `angle_tolerance`

---

## 📚 Riferimenti

- [Project Proposal.pdf](../Project Proposal.pdf)
- [Behavior Tree Documentation](ros2_ws/src/courier_nav/BEHAVIOR_TREE.md)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
