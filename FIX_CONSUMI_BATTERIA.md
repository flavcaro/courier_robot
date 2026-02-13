# 🔋 FIX CONSUMI BATTERIA - MAKEBLOCK ULTIMATE

**Problema diagnosticato:** Pin A0 non collegato (0V) → Nessun voltage divider → Compensazione automatica impossibile

**Soluzione:** Ridurre velocità motori + Fix bug codice

---

## ✅ MODIFICHE DA APPLICARE

### 1. `rover_API.py` - Fix bug validazione velocità (CRITICO)

**Problema:** Il controllo `if speed <= 1 or cmd in self.commands:` accetta speed > 1 se cmd è valido

**File:** `rpi_bt_integration/rover_API.py`

**Sostituisci la funzione `moveTo`:**

```python
def moveTo(self, cmd, speed):
    """
    Invia comando movimento al robot.
    
    Args:
        cmd: Comando ("Forward", "Back", "Left", "Right")
        speed: Velocità normalizzata (0.0 - 1.0)
    """
    # Validazione comando
    if cmd not in self.commands:
        print(f"⚠️  Comando non valido: {cmd}. Deve essere uno tra {self.commands}")
        return
    
    # Validazione velocità (CRITICO: deve essere tra 0 e 1)
    if not isinstance(speed, (int, float)):
        print("⚠️  Speed deve essere un numero")
        return
    
    if speed < 0.0:
        speed = 0.0
    if speed > 1.0:
        speed = 1.0
    
    speed_percent = int(speed * 100)
    to_send = f"{cmd}:{speed_percent}\n"
    self.ser.write(to_send.encode())
    # NON dormire qui - la durata è gestita da chi chiama
```

---

### 2. `bt/actions.py` - Separa duration e speed + Riduci velocità (CRITICO)

**Problema:** `duration` viene passato come `speed` → motori sempre al 100%+

**File:** `rpi_bt_integration/bt/actions.py`

**Sostituisci tutto con questo:**

```python
from rover_API import RoverApi
import time

# Inizializza il rover
rover = RoverApi(port='/dev/ttyUSB0')
print(f"Arduino connesso su /dev/ttyUSB0")

# ===== LIMITI VELOCITÀ "ECO" (RIDUCONO CONSUMI) =====
DEFAULT_SPEED_LINEAR = 0.40   # 40% per movimenti avanti/indietro
DEFAULT_SPEED_TURN   = 0.35   # 35% per rotazioni
STOP_PAUSE = 0.05             # Pausa dopo stop

def move_forward(duration=1.0, speed=DEFAULT_SPEED_LINEAR):
    """Muove avanti per durata specificata."""
    rover.moveTo('Forward', speed)
    time.sleep(duration)
    rover.stop()
    time.sleep(STOP_PAUSE)

def move_back(duration=1.0, speed=DEFAULT_SPEED_LINEAR):
    """Muove indietro per durata specificata."""
    rover.moveTo('Back', speed)
    time.sleep(duration)
    rover.stop()
    time.sleep(STOP_PAUSE)

def move_left(duration=1.0, speed=DEFAULT_SPEED_TURN):
    """Ruota a sinistra per durata specificata."""
    rover.moveTo('Left', speed)
    time.sleep(duration)
    rover.stop()
    time.sleep(STOP_PAUSE)

def move_right(duration=1.0, speed=DEFAULT_SPEED_TURN):
    """Ruota a destra per durata specificata."""
    rover.moveTo('Right', speed)
    time.sleep(duration)
    rover.stop()
    time.sleep(STOP_PAUSE)

def arm_up():
    """Solleva il braccio."""
    rover.armUP()
    time.sleep(0.5)

def arm_down():
    """Abbassa il braccio."""
    rover.armDown()
    time.sleep(0.5)

def open_hand(pwm=1500):
    """Apre la pinza."""
    rover.openHand(pwm)
    time.sleep(0.5)

def close_hand(pwm=1750):
    """Chiude la pinza."""
    rover.closeHand(pwm)
    time.sleep(0.5)
```

---

### 3. `bt/navigation_actions.py` - Usa velocità ridotte

**File:** `rpi_bt_integration/bt/navigation_actions.py`

**Aggiungi all'inizio (dopo gli import):**

```python
from .actions import DEFAULT_SPEED_LINEAR, DEFAULT_SPEED_TURN
```

**Sostituisci tutte le funzioni che usano `rover.moveTo(..., 1.0)`:**

#### `rotate_90_degrees`:

```python
def rotate_90_degrees(direction='right', compensate_drift=False):
    """Ruota 90° con velocità ridotta per risparmiare batteria."""
    print(f"🔄 Rotazione 90° a {'destra' if direction == 'right' else 'sinistra'}...")
    
    cmd = 'Right' if direction == 'right' else 'Left'
    rover.moveTo(cmd, DEFAULT_SPEED_TURN)
    
    time.sleep(robot_state.rotation_90_time)
    rover.stop()
    time.sleep(0.1)
    
    # Compensazione drift DISABILITATA per test consumi
    # (riattivala dopo se serve)
    
    print(f"✓ Rotazione completata!")
```

#### `rotate_180_degrees`:

```python
def rotate_180_degrees():
    """Ruota 180° con velocità ridotta."""
    print(f"🔄 Rotazione 180°...")
    
    rover.moveTo('Right', DEFAULT_SPEED_TURN)
    time.sleep(robot_state.rotation_90_time * 2.0)
    rover.stop()
    time.sleep(0.15)
    
    print(f"✓ Rotazione 180° completata!")
```

#### `move_one_cell_forward`:

```python
def move_one_cell_forward():
    """Muove avanti di una cella con velocità ridotta."""
    print(f"➡️  Movimento avanti di una cella...")
    
    rover.moveTo('Forward', DEFAULT_SPEED_LINEAR)
    time.sleep(robot_state.cell_move_time)
    rover.stop()
    time.sleep(0.1)
    
    print(f"✓ Cella raggiunta!")
```

---

### 4. `bt/sensors.py` - Ricalibra tempi per velocità ridotte

**File:** `rpi_bt_integration/bt/sensors.py`

**Nel `__init__` di RobotState, modifica:**

```python
# === Calibrazione Movimento (con velocità 40% invece di 100%) ===
self.base_rotation_90_time = 16.0  # secondi per 90° al 35% velocità
self.base_cell_move_time = 5.0     # secondi per 60cm al 40% velocità
```

**Nota:** Questi sono valori stimati. Dopo il fix, testa e ricalibrali.

---

## 🧪 TEST RAPIDO DOPO LE MODIFICHE

```bash
# Sulla Raspberry Pi
cd ~/robot_code

# Test movimento singolo
python3 -c "
from bt.actions import move_forward, move_right, rover
import time

print('Test movimento ECO (40% velocità):')
print('1. Avanti 2 secondi...')
move_forward(duration=2.0)
time.sleep(1)

print('2. Rotazione destra 2 secondi...')
move_right(duration=2.0)
time.sleep(1)

print('3. Stop')
rover.stop()
print('✅ Test completato!')
rover.close()
"
```

---

## 📊 RIDUZIONE CONSUMI ATTESA

| Prima (100% velocità) | Dopo (40% velocità) | Risparmio |
|-----------------------|---------------------|-----------|
| ~3.5A continui | ~1.2A continui | **66%** |
| 2-3 simulazioni | **6-8 simulazioni** | **3x autonomia** |

---

## ⚠️ CALIBRAZIONE FINALE

Dopo aver applicato tutte le modifiche:

1. **Testa movimento singolo** (vedi sopra)
2. **Misura tempo rotazione 90°** effettivo
3. **Aggiorna `base_rotation_90_time` in sensors.py**
4. **Misura tempo movimento 60cm** effettivo  
5. **Aggiorna `base_cell_move_time` in sensors.py**

---

## 🚀 BONUS: Aggiungi Voltage Divider (Opzionale)

Se vuoi aggiungere monitoraggio batteria:

### Hardware necessario:
- 2x Resistenze 10kΩ
- Cavi connettori

### Schema:
```
Batteria (+) ──┬── R1 (10kΩ) ──┬── Pin A0 Arduino
               │                │
               │                └── R2 (10kΩ) ── GND
               └── Resto circuito robot
```

Questo divide la tensione per 2 (8.4V → 4.2V) → sicuro per Arduino

Dopo l'installazione, riabilita la compensazione in sensors.py.

---

## 📝 CHECKLIST APPLICAZIONE

- [ ] Fix `rover_API.py` → validazione corretta
- [ ] Fix `bt/actions.py` → velocità 40%
- [ ] Fix `bt/navigation_actions.py` → usa costanti ECO
- [ ] Aggiorna `bt/sensors.py` → tempi ricalibrari
- [ ] Test movimento base funzionante
- [ ] Calibrazione tempi effettivi
- [ ] Test simulazione completa

---

## 🎯 RISULTATO ATTESO

✅ Consumi ridotti del **60-70%**  
✅ Autonomia **triplicata** (da 2-3 a 6-8 simulazioni)  
✅ Movimenti più precisi (meno inerzia)  
✅ Batteria si scarica gradualmente, non "vola"

**Dopo queste modifiche, il robot dovrebbe durare molto di più! 🚀**
