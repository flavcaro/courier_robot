# Courier Robot - Standalone Version

Versione standalone del sistema di navigazione autonoma per robot Makeblock Ultimate, adattata dal progetto ROS2 originale.

## 📋 Panoramica

Questo sistema implementa navigazione autonoma basata su:
- **Griglia virtuale** sovrapposta all'ambiente fisico
- **Behavior Tree** (py_trees) per controllo missione
- **BFS pathfinding** per pianificazione percorso
- **Odometria stimata** per localizzazione
- **Sensore ultrasuoni** per rilevamento ostacoli

## 🏗️ Architettura

```
┌─────────────────────────────────────┐
│  Raspberry Pi 2                     │
│  ┌───────────────────────────────┐  │
│  │ robot_controller.py           │  │
│  │ - Behavior Tree               │  │
│  │ - BFS Pathfinding             │  │
│  │ - Odometria stimata           │  │
│  └───────────────────────────────┘  │
│              ↓                       │
│  ┌───────────────────────────────┐  │
│  │ rover_API.py                  │  │
│  │ - Comandi movimento           │  │
│  │ - Controllo braccio/pinza     │  │
│  │ - Lettura ultrasuoni          │  │
│  └───────────────────────────────┘  │
│              ↓ Serial               │
└─────────────────────────────────────┘
              ↓
┌─────────────────────────────────────┐
│  Arduino (Makeblock Ultimate)       │
│  - Controllo motori                 │
│  - Attuatori braccio                │
│  - Sensore ultrasuoni               │
└─────────────────────────────────────┘
```

## 📁 Struttura File

```
rpi_standalone/
├── robot_controller.py      # Controller principale
├── rover_API.py              # API comunicazione Arduino
├── behaviors/                # Behavior Tree nodes
│   ├── __init__.py
│   ├── navigation.py         # Rotazione, movimento, waypoint
│   ├── mission.py            # Raccolta/consegna oggetto
│   ├── obstacle.py           # Gestione ostacoli
│   └── battery.py            # Gestione batteria
├── requirements.txt          # Dipendenze Python
└── README.md                 # Questa guida
```

## 🚀 Setup

### 1. Installazione su Raspberry Pi

```bash
# Clona il progetto
cd /home/pi
git clone <repository-url>
cd courier_robot1/rpi_standalone

# Installa dipendenze
pip3 install -r requirements.txt

# Verifica connessione Arduino
ls /dev/ttyUSB*  # Dovrebbe mostrare /dev/ttyUSB0
```

### 2. Configurazione Griglia Fisica

#### Opzione A: Griglia con Nastro Adesivo (CONSIGLIATA)

Materiali necessari:
- Nastro adesivo colorato
- Metro a nastro
- Superficie piana (3m × 3m minimo)

**Procedura:**

1. **Definisci dimensioni griglia**
   ```
   Griglia 5×5 con celle da 50cm:
   Dimensione totale: 2.5m × 2.5m
   ```

2. **Segna la griglia sul pavimento**
   ```
   Esempio griglia 5×5:
   
   2.5m ┌────┬────┬────┬────┬────┐
        │    │    │GOAL│    │    │  Row 4
   2.0m ├────┼────┼────┼────┼────┤
        │    │ XX │    │ XX │    │  Row 3
   1.5m ├────┼────┼────┼────┼────┤
        │    │    │    │    │    │  Row 2
   1.0m ├────┼────┼────┼────┼────┤
        │    │    │    │    │    │  Row 1
   0.5m ├────┼────┼────┼────┼────┤
        │STRT│    │    │    │    │  Row 0
   0.0m └────┴────┴────┴────┴────┘
        0.0  0.5  1.0  1.5  2.0  2.5m
        
   STRT = Cella (0,0) - Posizione iniziale
   GOAL = Cella (4,2) - Obiettivo pickup
   XX   = Ostacoli fisici (scatole, coni, ecc.)
   ```

3. **Posiziona ostacoli**
   - Metti oggetti fisici nelle celle con ostacoli
   - Assicurati siano più alti di 10cm (per ultrasuoni)

4. **Calibra posizione iniziale**
   - Posiziona robot al centro della cella (0,0)
   - Orientato verso Est (direzione +X, yaw=0°)

#### Opzione B: Griglia Virtuale (senza marker)

Se non volete segnare il pavimento:
- Misurate l'ambiente
- Definite origine griglia (angolo sud-ovest)
- Posizionate robot manualmente all'inizio

**⚠️ Nota:** Senza marker visivi, l'odometria stimata avrà più drift.

### 3. Configurazione Software

Modifica `robot_controller.py`:

```python
class RobotController:
    def __init__(self, serial_port='/dev/ttyUSB0'):
        # === Configurazione Griglia ===
        self.cell_size = 0.5  # Dimensione cella in metri
        self.grid_size = 5    # Griglia NxN
        
        # Ostacoli predefiniti (row, col)
        self.obstacles = {(1, 1), (3, 3)}
        
        # === Parametri Missione ===
        self.start_cell = (0, 0)  # Cella iniziale
        self.goal_cell = (4, 2)   # Cella obiettivo
        
        # === Parametri Controllo ===
        self.rotation_speed = 0.3    # Velocità rotazione (0-1)
        self.linear_speed = 0.2      # Velocità lineare (0-1)
        self.angle_tolerance = 0.15  # Tolleranza angolo (rad)
        self.position_tolerance = 0.15  # Tolleranza posizione (m)
        
        # === Sensori ===
        self.obstacle_threshold = 0.40  # Soglia ostacolo (m)
```

## 🎮 Utilizzo

### Avvio Missione

```bash
# Avvia controller
python3 robot_controller.py
```

**Output atteso:**
```
============================================================
🤖 COURIER ROBOT - STANDALONE CONTROLLER
============================================================
✅ Connesso ad Arduino su /dev/ttyUSB0
📍 Griglia: 5×5 celle da 0.5m
🎯 Missione: (0, 0) → (4, 2) → (0, 0)
🔋 Batteria: 100%
============================================================
[  0.00s] ℹ️  🚀 Avvio missione...
[  0.15s] ℹ️  ✅ Behavior Tree creato
[  0.20s] ℹ️  ▶️  Inizio esecuzione missione
============================================================
📍 PIANIFICAZIONE PERCORSO INIZIALE
============================================================
[  0.25s] ℹ️  Partenza: (0, 0) → Obiettivo: (4, 2)
[  0.30s] ℹ️  ✓ Percorso trovato con 6 waypoints
[  0.30s] ℹ️    1. Cella(1, 0) → (0.25, 0.75)
[  0.30s] ℹ️    2. Cella(2, 0) → (0.25, 1.25)
...
```

### Interruzione Manuale

Premi `Ctrl+C` per fermare il robot in sicurezza:
```
^C
[  45.23s] ⚠️  ⏸️  Missione interrotta dall'utente
[  45.25s] ℹ️  🏁 Controller terminato
🔌 Connessione seriale chiusa
```

## 🧪 Test e Calibrazione

### Test 1: Movimento Base

Crea `test_movement.py`:

```python
from rover_API import RoverApi
import time

rover = RoverApi('/dev/ttyUSB0')

print("Test movimento...")

# Avanti 1 secondo
rover.moveTo('Forward', 0.3)
time.sleep(1)
rover.stop()

# Rotazione sinistra
rover.moveTo('Left', 0.3)
time.sleep(1)
rover.stop()

rover.close()
print("Test completato!")
```

### Test 2: Calibrazione Celle

Misura quanto si muove il robot:

```python
# Muovi per 2 secondi a velocità 0.3
rover.moveTo('Forward', 0.3)
time.sleep(2)
rover.stop()

# Misura distanza percorsa fisicamente
# Calcola: velocità_reale = distanza / tempo
```

Aggiorna `navigation.py` con velocità calibrata:
```python
linear_velocity = speed * VELOCITA_CALIBRATA  # m/s
```

### Test 3: Griglia Piccola

Inizia con griglia 3×3 per test:

```python
self.cell_size = 0.6  # 60cm
self.grid_size = 3
self.obstacles = {(1, 1)}
self.start_cell = (0, 0)
self.goal_cell = (2, 2)
```

## 🔧 Risoluzione Problemi

### Problema: Robot non si muove

**Cause possibili:**
1. Arduino non connesso → Controlla `/dev/ttyUSB0`
2. Batteria scarica → Ricarica batteria robot
3. Porta seriale sbagliata → Prova `/dev/ttyACM0`

**Soluzione:**
```bash
# Trova porta corretta
ls -l /dev/tty* | grep USB

# Testa connessione
python3 -c "from rover_API import RoverApi; r = RoverApi('/dev/ttyUSB0'); r.stop(); r.close()"
```

### Problema: Odometria imprecisa

**Cause:**
- Velocità non calibrata
- Superficie scivolosa
- Encoder non disponibili

**Soluzioni:**
1. Calibra velocità (vedi Test 2)
2. Aumenta tolleranze:
   ```python
   self.position_tolerance = 0.20  # 20cm invece di 15cm
   ```
3. Usa griglia più grande (celle da 80cm)

### Problema: Ostacoli non rilevati

**Cause:**
- Ultrasuono fuori range
- Ostacolo troppo basso/piccolo

**Soluzioni:**
1. Verifica range sensore:
   ```python
   distance = rover.getUltrasonicSensor()
   print(f"Distanza: {distance}cm")
   ```
2. Usa ostacoli alti >10cm
3. Riduci soglia: `self.obstacle_threshold = 0.30`

## 📊 Parametri Configurabili

| Parametro | Default | Range | Descrizione |
|-----------|---------|-------|-------------|
| `cell_size` | 0.5m | 0.3-1.0m | Dimensione celle griglia |
| `grid_size` | 5 | 3-10 | Dimensione griglia NxN |
| `rotation_speed` | 0.3 | 0.1-0.5 | Velocità rotazione |
| `linear_speed` | 0.2 | 0.1-0.4 | Velocità lineare |
| `angle_tolerance` | 0.15 rad | 0.1-0.3 | ~8.6° tolleranza angolo |
| `position_tolerance` | 0.15m | 0.1-0.25 | 15cm tolleranza posizione |
| `obstacle_threshold` | 0.40m | 0.2-0.6 | 40cm soglia ostacolo |

## 🎯 Missioni Personalizzate

### Esempio: Percorso a L

```python
self.grid_size = 4
self.obstacles = {(1, 1), (2, 1)}
self.start_cell = (0, 0)
self.goal_cell = (3, 3)

# Percorso BFS troverà automaticamente:
# (0,0) → (1,0) → (2,0) → (3,0) → (3,1) → (3,2) → (3,3)
```

### Esempio: Consegna Multipla

Modifica `robot_controller.py` per aggiungere waypoint intermedi.

## 📝 Note Importanti

1. **Odometria Stimata**: Senza encoder, la posizione ha drift. Ricalibrare periodicamente.
2. **Superficie**: Funziona meglio su pavimento liscio e uniforme.
3. **Batteria**: Il sistema simula batteria. Per batteria reale, integra sensore.
4. **Sicurezza**: Testa sempre in area sicura, lontano da scale/ostacoli pericolosi.

## 🔗 Riferimenti

- Progetto originale ROS2: `../ros2_ws/`
- Documentazione py_trees: https://py-trees.readthedocs.io/
- API Makeblock: Vedi `rover_API.py`

## 📧 Supporto

Per problemi o domande, consulta la documentazione del progetto principale.
