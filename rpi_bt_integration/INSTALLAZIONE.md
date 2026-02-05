# Guida Integrazione Sistema Navigazione

## 📁 Struttura File

Hai creato questi nuovi file da copiare sulla Raspberry Pi:

```
rpi_bt_integration/
├── sensors.py                    # Sensori e stato robot (NUOVO)
├── navigation_actions.py         # Azioni navigazione (NUOVO)
├── navigation_behaviours.py      # Behaviors navigazione (NUOVO)
└── main_mission.py               # Controller principale (NUOVO)
```

Questi si integrano con i file esistenti in `~/robot_code/bt/`:

```
~/robot_code/bt/
├── actions.py          # Azioni base (ESISTENTE)
├── behaviours.py       # Behaviors base (ESISTENTE)
├── sensors.py          # ← SOSTITUISCI con nuovo
└── rover_API.py        # API rover (ESISTENTE)
```

## 🚀 Procedura di Installazione

### Sulla Raspberry Pi (via SSH):

```bash
# 1. Vai nella directory bt
cd ~/robot_code/bt

# 2. Backup file esistenti
cp sensors.py sensors.py.backup

# 3. Scarica i nuovi file dal PC
# (esegui questo dal PC in un altro terminale)
```

### Sul PC:

```bash
# Copia i nuovi file sulla Raspberry Pi
cd /home/giovanni/Scrivania/Università/Magistrale/ISRLab/courier_robot1

scp rpi_bt_integration/sensors.py pi@pi.local:~/robot_code/bt/
scp rpi_bt_integration/navigation_actions.py pi@pi.local:~/robot_code/bt/
scp rpi_bt_integration/navigation_behaviours.py pi@pi.local:~/robot_code/bt/
scp rpi_bt_integration/main_mission.py pi@pi.local:~/robot_code/
```

## 🧪 Test

### Test 1: Navigazione Semplice

```bash
# Sulla Raspberry Pi
cd ~/robot_code
python3 main_mission.py

# Scegli opzione 1 (test navigazione)
# Il robot andrà a (1,2) e tornerà a (0,0)
```

### Test 2: Missione Completa

```bash
python3 main_mission.py

# Scegli opzione 2 (missione completa)
# Il robot farà pickup + delivery completo
```

## 📝 Configurazione

Modifica parametri in `sensors.py`:

```python
class RobotState:
    def __init__(self):
        # Griglia
        self.cell_size = 0.6          # Dimensione celle
        self.grid_size = 3            # Griglia 3×3
        self.obstacles = {(1, 1)}     # Ostacoli
        
        # Missione
        self.start_cell = (0, 0)      # Partenza
        self.goal_cell = (1, 2)       # Arrivo
        
        # Velocità
        self.rotation_speed = 0.3     # Rotazione
        self.linear_speed = 0.2       # Lineare
```

## 🔧 Risoluzione Problemi

### Import Error

Se ottieni errori di import, verifica che tutti i file siano in `~/robot_code/bt/`:

```bash
ls -la ~/robot_code/bt/
# Dovresti vedere:
# - actions.py
# - behaviours.py
# - sensors.py (nuovo)
# - navigation_actions.py (nuovo)
# - navigation_behaviours.py (nuovo)
```

### ModuleNotFoundError

Assicurati di avere il virtual environment attivo:

```bash
source ~/venv/bin/activate  # o il path del tuo venv
```

## ✅ Checklist

- [ ] File copiati su Raspberry Pi
- [ ] Virtual environment attivo
- [ ] Griglia fisica preparata (1.8m × 1.8m)
- [ ] Robot posizionato in (0,0)
- [ ] Arduino connesso (`/dev/ttyUSB0`)
- [ ] Test navigazione eseguito con successo
- [ ] Missione completa pronta
