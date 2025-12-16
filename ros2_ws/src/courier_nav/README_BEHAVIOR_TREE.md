# Courier Navigation con Behavior Tree

Il controller del robot courier è stato aggiornato per utilizzare un **behavior tree** per gestire tutte le decisioni, semplificando la struttura del codice e rendendolo più modulare.

## Cosa è Cambiato

### Prima
Il controller usava una logica procedurale con if-else annidati nel metodo `control_loop()`:
- Difficile da estendere
- Logica sparsa in vari metodi
- Difficile da testare

### Dopo
Il controller usa un **behavior tree** che organizza la logica in nodi riutilizzabili:
- Struttura modulare e gerarchica
- Nodi facilmente testabili
- Facile aggiungere nuovi comportamenti
- Decisioni reattive ad ogni tick

## Installazione

### 1. Installa py_trees

```bash
pip install py_trees>=2.2.0
```

### 2. Rebuilda il workspace

```bash
cd /home/giovanni/Scrivania/Università/Magistrale/ISRLab/robot/ros2_ws
colcon build --packages-select courier_nav
source install/setup.bash
```

## Esecuzione

### Avvia la simulazione

```bash
cd /home/giovanni/Scrivania/Università/Magistrale/ISRLab/robot/ros2_ws
./start_simulation.sh
```

### Avvia il controller (in un altro terminale)

```bash
cd /home/giovanni/Scrivania/Università/Magistrale/ISRLab/robot/ros2_ws
source install/setup.bash
ros2 run courier_nav controller
```

## Funzionalità del Behavior Tree

### 1. Gestione Batteria
- Il robot monitora la batteria ad ogni tick
- Se scende sotto il 20%, interrompe la missione e va a ricaricare
- La ricarica simula un incremento del 30% per tick

### 2. Navigazione Intelligente
Il tree gestisce automaticamente:
- **Prelievo waypoint**: Prende il prossimo waypoint dalla coda del path
- **Allineamento**: Ruota verso il target prima di muoversi
- **Movimento**: Si muove verso il target con velocità proporzionale alla distanza
- **Raggiungimento**: Quando raggiunge il target, lo cancella e passa al successivo

### 3. Stop Automatico
Quando tutti i waypoint sono stati visitati, il robot si ferma automaticamente.

## Struttura del Behavior Tree

Vedi [BEHAVIOR_TREE.md](BEHAVIOR_TREE.md) per:
- Diagramma completo della struttura
- Descrizione di ogni nodo
- Esempi di esecuzione
- Guide per personalizzazione ed estensione

## Debug

### Visualizza lo stato della batteria
Nel log vedrai periodicamente:
```
[courier_controller] 🔋 Battery: 85.0%
```

### Visualizza la struttura del tree
Per vedere lo stato completo del behavior tree ad ogni tick, decomenta questa riga in `courier_controller.py`:

```python
# In control_loop()
self.get_logger().info(f'\n{py_trees.display.unicode_tree(self.behavior_tree, show_status=True)}')
```

Output esempio:
```
Root [✓]
├── Battery Manager [✓]
│   ├── BatteryOK [✓]
│   └── GoCharge [-]
└── Mission Selector [✓]
    ├── Navigation [✓]
    │   ├── HasTarget [✓]
    │   └── Navigate to Target [*]
    │       ├── Reach Target [-]
    │       └── Move to Target [*]
    │           ├── Rotation Check [✓]
    │           │   ├── IsAligned [✓]
    │           │   └── RotateToTarget [-]
    │           └── MoveToTarget [*]
    ├── Get Next Waypoint [-]
    └── StopRobot [-]
```

Legenda:
- `✓` = SUCCESS
- `✗` = FAILURE
- `*` = RUNNING
- `-` = Non eseguito

## Personalizzazione

### Modificare le soglie

In [`courier_behavior_tree.py`](courier_nav/courier_behavior_tree.py):

```python
# Soglia batteria minima
battery_ok = BatteryOK(name="BatteryOK", battery_threshold=30.0)  # 30% invece di 20%

# Velocità di ricarica
go_charge = GoCharge(name="GoCharge", charge_rate=50.0)  # 50% invece di 30%

# Tolleranza allineamento
is_aligned = IsAligned(name="IsAligned", angle_tolerance=0.1)  # Più preciso

# Tolleranza distanza
is_at_target = IsAtTarget(name="IsAtTarget", dist_tolerance=0.2)  # Meno preciso
```

### Aggiungere nuovi comportamenti

Esempio: Aggiungere rilevamento ostacoli

1. Crea il nodo condizione:
```python
class ObstacleDetected(py_trees.behaviour.Behaviour):
    def update(self):
        # Leggi dal sensore laser
        obstacle_distance = self.blackboard.get("obstacle_distance")
        if obstacle_distance < 0.5:  # 50cm
            return Status.SUCCESS
        return Status.FAILURE
```

2. Crea il nodo azione:
```python
class AvoidObstacle(py_trees.behaviour.Behaviour):
    def update(self):
        # Logica per evitare ostacolo
        cmd_vel_pub = self.blackboard.get("cmd_vel_publisher")
        msg = Twist()
        msg.angular.z = 1.0  # Ruota
        cmd_vel_pub(msg)
        return Status.RUNNING
```

3. Aggiungi al tree:
```python
# In create_courier_behavior_tree()
obstacle_sequence = Sequence("Obstacle Avoidance", memory=False)
obstacle_detected = ObstacleDetected()
avoid = AvoidObstacle()
obstacle_sequence.add_children([obstacle_detected, avoid])

# Inserisci prima del movimento
mission_selector.add_children([
    obstacle_sequence,  # <-- Nuovo
    navigation_sequence,
    get_waypoint_sequence,
    stop_robot
])
```

## File Principali

- [`courier_controller.py`](courier_nav/courier_controller.py) - Controller ROS2 principale
- [`courier_behavior_tree.py`](courier_nav/courier_behavior_tree.py) - Definizione behavior tree e nodi
- [`BEHAVIOR_TREE.md`](BEHAVIOR_TREE.md) - Documentazione dettagliata del behavior tree

## Testing

### Test di un nodo
```python
import py_trees
from courier_nav.courier_behavior_tree import BatteryOK

def test_battery_ok():
    blackboard = py_trees.blackboard.Client()
    blackboard.register_key("battery_level", access=py_trees.common.Access.WRITE)
    blackboard.set("battery_level", 50.0)
    
    node = BatteryOK(battery_threshold=20.0)
    node.setup()
    
    status = node.update()
    assert status == py_trees.common.Status.SUCCESS
```

### Test del tree completo
```python
from courier_nav.courier_behavior_tree import create_courier_behavior_tree

def test_full_tree():
    tree = create_courier_behavior_tree(cell_size=1.0)
    blackboard = py_trees.blackboard.Client()
    
    # Setup
    blackboard.set("battery_level", 100.0)
    blackboard.set("current_target", None)
    blackboard.set("path_queue", [(1, 0), (2, 0)])
    
    tree.setup_with_descendants()
    tree.tick_once()
    
    # Verifica che abbia prelevato un target
    current_target = blackboard.get("current_target")
    assert current_target is not None
```

## Risoluzione Problemi

### Il robot non si muove
1. Controlla che la batteria non sia sotto il 20%
2. Verifica che il path_queue contenga waypoint
3. Guarda i log per capire quale nodo sta fallendo

### La batteria non si ricarica
Verifica che il nodo `GoCharge` sia configurato correttamente nel blackboard e che il battery_threshold sia impostato.

### Il robot non ruota correttamente
Modifica la soglia `angle_tolerance` in `IsAligned` per essere più/meno permissivo.

## Vantaggi del Behavior Tree

✅ **Modularità**: Ogni comportamento è un nodo isolato  
✅ **Reattività**: Decisioni prese ad ogni tick (10Hz)  
✅ **Testabilità**: Ogni nodo è testabile indipendentemente  
✅ **Estendibilità**: Facile aggiungere nuovi comportamenti  
✅ **Manutenibilità**: Codice più pulito e organizzato  
✅ **Debug**: Visualizzazione dello stato ad ogni tick  

## Risorse

- [py_trees Documentation](https://py-trees.readthedocs.io/)
- [Behavior Trees in Robotics](https://arxiv.org/abs/1709.00084)
- [BEHAVIOR_TREE.md](BEHAVIOR_TREE.md) - Documentazione completa

## Supporto

Per domande o problemi, consulta:
1. La documentazione in [BEHAVIOR_TREE.md](BEHAVIOR_TREE.md)
2. I commenti nel codice
3. Gli esempi in `/Script/Esercizio_trees/py_trees_demo/bt2/`
