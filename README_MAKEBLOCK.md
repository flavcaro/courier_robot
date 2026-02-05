# Makeblock Ultimate - Sistema Navigazione Autonoma

Branch dedicato al codice per il robot fisico Makeblock Ultimate con Raspberry Pi 2.

## 📁 Struttura Repository

```
courier_robot1/
├── RoverAPI.ino                  # Sketch Arduino per Makeblock
├── rpi_bt_integration/           # Codice Python per Raspberry Pi
│   ├── sensors.py                # Gestione sensori e stato robot
│   ├── navigation_actions.py     # Azioni navigazione + BFS
│   ├── navigation_behaviours.py  # Nodi Behavior Tree
│   ├── main_mission.py           # Controller principale
│   ├── deploy_navigation.sh      # Script deployment
│   └── INSTALLAZIONE.md          # Guida installazione
└── README_MAKEBLOCK.md           # Questa guida
```

## 🤖 Hardware Richiesto

- **Robot**: Makeblock Ultimate 2.0
- **Controller**: Arduino Mega 2560
- **Computer**: Raspberry Pi 2 Model B
- **Sensori**: Ultrasuono frontale
- **Alimentazione**: 6× batterie AA (1.5V)
- **Connessione**: Cavo USB Arduino ↔ Raspberry Pi

## 🚀 Setup Rapido

### 1. Arduino

```bash
# Carica RoverAPI.ino su Arduino Mega
# Usa Arduino IDE o arduino-cli
```

### 2. Raspberry Pi

```bash
# Sulla Raspberry Pi
cd ~/robot_code/bt

# Copia i file Python da rpi_bt_integration/
# Usa deploy_navigation.sh dal PC
```

### 3. Test

```bash
# Sulla Raspberry Pi
cd ~/robot_code
source ~/venv/bin/activate
python3 main_mission.py
```

## 📖 Documentazione

- **Installazione completa**: [`rpi_bt_integration/INSTALLAZIONE.md`](rpi_bt_integration/INSTALLAZIONE.md)
- **Guida deployment**: Usa `deploy_navigation.sh`
- **Configurazione griglia**: Modifica `sensors.py`

## 🔧 Configurazione Griglia

Parametri in `rpi_bt_integration/sensors.py`:

```python
self.cell_size = 0.6          # 60cm × 60cm
self.grid_size = 3            # Griglia 3×3
self.obstacles = {(1, 1)}     # Ostacoli
self.start_cell = (0, 0)      # Partenza
self.goal_cell = (1, 2)       # Arrivo
```

## 📊 File Esclusi da Questo Branch

Questo branch contiene **solo** i file necessari per il Makeblock fisico:
- ✅ `RoverAPI.ino` - Arduino
- ✅ `rpi_bt_integration/` - Raspberry Pi
- ❌ `ros2_ws/` - Non necessario (ROS2 solo per simulazione)
- ❌ `rpi_standalone/` - Versione alternativa non usata

## 🌿 Branch Disponibili

- `main` - Codice originale
- `final` - Versione completa con ROS2 + Gazebo
- **`makeblock`** - Solo codice robot fisico (questo branch)

## 🔗 Branch Principale

Per tornare al branch con ROS2 e simulazione:

```bash
git checkout final
```
