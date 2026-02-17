# 🤖 Robot Courier - Struttura Progetto

Sistema di navigazione lineare con aggiramento ostacoli **SENZA griglia**.

## 📁 Struttura File

### ✅ **File Principali da Usare**

```
simple_mission.py              ← 🚀 ESEGUI QUESTO per missione completa
test_obstacle_avoid_simple.py  ← Test aggiramento ostacoli (debug)
```

### 📦 **Moduli Sistema Semplice** (`bt/`)

```
bt/
├── simple_state.py            ← Stato robot (distanza + offset)
├── simple_actions.py          ← Movimento lineare (rotate, move_forward_meters)
└── simple_behaviours.py       ← Behavior Tree aggiramento ostacoli
```

### 🔧 **Moduli Base** (usati da sistema semplice)

```
bt/
├── actions.py                 ← Azioni base (move_forward, arm_up, ecc)
├── behaviours.py              ← BT base (GrabObject, DropObject)
└── sensors.py                 ← Stato sensori (da semplificare)
```

### 🗑️ **File Vecchi** (NON usare)

```
main_mission.py                ← Sistema vecchio con griglia
main.py                        ← BT base senza navigazione
```

---

## 🚀 Come Usare

### 1️⃣ **Missione Completa (Consigliato)**

```bash
cd ~/robot_code
python3 simple_mission.py
```

**Cosa fa:**
- Calibrazione rotazioni e compensazione motori
- Vai dritto verso Nord per X metri
- Aggira ostacoli quando li trova (EST o OVEST)
- Torna sulla linea centrale dopo aggiramento
- Grab oggetto al target

### 2️⃣ **Test Aggiramento Ostacoli**

```bash
python3 test_obstacle_avoid_simple.py
```

**Cosa fa:**
- Test puro aggiramento ostacoli
- Loop infinito: Nord → Ostacolo → Aggira → Nord
- Utile per calibrare tempi rotazione

---

## 📊 Sistema Semplice vs Griglia

| Caratteristica | Sistema Semplice ✅ | Sistema Griglia ❌ |
|---|---|---|
| **Localizzazione** | Distanza percorsa (m) + offset laterale | Celle 4x4 con coordinate (row, col) |
| **Movimento** | Lineare verso target | Pathfinding BFS con celle |
| **Aggiramento** | Scansione Est/Ovest | Memoria ostacoli in griglia |
| **Complessità** | ~500 righe | ~2000 righe |
| **Usa caso** | Missione lineare A→B | Navigazione complessa multi-punto |

**Il tuo caso d'uso:** Linea retta + aggiramento → **Sistema Semplice** ✅

---

## 🔧 Calibrazione

**Parametri da calibrare** (in `simple_mission.py`):

1. **Rotazione 90°** (`rotation_90_time`): Default 5.0s
   - Se gira poco → aumenta a 5.5s
   - Se gira troppo → riduci a 4.5s

2. **Compensazione motori** (`left_factor`, `right_factor`):
   - Default: Left 1.0, Right 0.95
   - Se devia sinistra → riduci `right_factor` (0.90-0.93)
   - Se devia destra → aumenta `right_factor` (0.98-1.0)

3. **Velocità movimento** (opzionale):
   - `meters_per_second_forward` = 0.20 (default 60% velocità)
   - Misura tempo per percorrere 1 metro e calibra

---

## 🧪 Debug

Se qualcosa non funziona:

1. **Test movimenti base:**
   ```bash
   python3 rover_API.py
   ```

2. **Test aggiramento:**
   ```bash
   python3 test_obstacle_avoid_simple.py
   ```

3. **Test comportamento singolo nodo BT:**
   ```python
   from bt.simple_actions import check_obstacle_ahead, move_forward_meters
   is_obstacle, dist = check_obstacle_ahead()
   print(f"Ostacolo: {is_obstacle}, Distanza: {dist}cm")
   ```

---

## 📝 Prossimi Passi (Opzionale)

- [ ] Semplificare `bt/sensors.py` (rimuovere parti griglia)
- [ ] Aggiungere missione "torna indietro" dopo grab
- [ ] Salvare calibrazioni in file config
- [ ] Aggiungere visualizzazione ASCII della traiettoria

---

**Creato:** 15 Feb 2026  
**Sistema:** Lineare senza griglia
