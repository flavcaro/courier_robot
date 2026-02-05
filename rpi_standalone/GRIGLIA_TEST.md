# Configurazione Griglia Test 3×3

## 📐 Dimensioni

- **Griglia**: 3×3 celle
- **Dimensione cella**: 60cm × 60cm
- **Dimensione totale**: 1.8m × 1.8m

## 🗺️ Layout Griglia

```
     Col 0    Col 1    Col 2
   ┌────────┬────────┬────────┐
   │        │        │  GOAL  │  Row 2 (1.8m)
   │        │        │  (1,2) │
   ├────────┼────────┼────────┤
   │        │   XX   │        │  Row 1 (1.2m)
   │        │  (1,1) │        │
   ├────────┼────────┼────────┤
   │ START  │        │        │  Row 0 (0.6m)
   │ (0,0)  │        │        │
   └────────┴────────┴────────┘
   0.0m    0.6m     1.2m     1.8m
```

## 📍 Punti Chiave

| Elemento | Cella (row, col) | Coordinate (x, y) | Descrizione |
|----------|------------------|-------------------|-------------|
| **START** | (0, 0) | (0.3m, 0.3m) | Posizione iniziale robot |
| **GOAL** | (1, 2) | (1.5m, 0.9m) | Destinazione pickup |
| **OSTACOLO** | (1, 1) | (0.9m, 0.9m) | Scatola/cono fisico |

## 🛤️ Percorso Calcolato (BFS)

Il robot seguirà questo percorso:

```
START (0,0) 
   ↓
(0,1) - Cella intermedia
   ↓
(0,2) - Cella intermedia
   ↓
GOAL (1,2) - Destinazione
```

**Distanza totale**: 3 celle = ~1.8m

## 📏 Setup Fisico

### Materiali Necessari

- Nastro adesivo colorato
- Metro a nastro
- 1 ostacolo (scatola/cono alto >10cm)
- Superficie piana 2m × 2m

### Procedura

1. **Segna l'origine** (angolo sud-ovest):
   ```
   Metti un segno a terra per (0, 0)
   ```

2. **Traccia la griglia**:
   ```
   Linee verticali: a 0.6m, 1.2m, 1.8m dall'origine
   Linee orizzontali: a 0.6m, 1.2m, 1.8m dall'origine
   ```

3. **Posiziona ostacolo**:
   ```
   Centro cella (1,1): a 0.9m in X, 0.9m in Y
   ```

4. **Segna START e GOAL**:
   ```
   START: Centro cella (0,0) - nastro verde
   GOAL: Centro cella (1,2) - nastro rosso
   ```

## 🤖 Posizionamento Robot

1. Posiziona robot al **centro** della cella (0,0)
2. Coordinate esatte: **x=0.3m, y=0.3m**
3. Orientamento: **verso Est** (direzione +X, yaw=0°)
4. Verifica che il robot sia centrato usando il nastro

## 🧪 Test Previsti

### Fase 1: Navigazione verso GOAL
- Robot parte da (0,0)
- Ruota verso Nord (90°)
- Si muove a (0,1)
- Si muove a (0,2)
- Ruota verso Est (0°)
- Si muove a (1,2) - GOAL

### Fase 2: Raccolta Oggetto
- Animazione braccio/pinza (6 secondi)

### Fase 3: Ritorno a START
- Robot ripianifica percorso
- Torna a (0,0)

### Fase 4: Consegna
- Animazione consegna (5 secondi)

## ⚙️ Parametri Configurati

```python
# In robot_controller.py
self.cell_size = 0.6          # 60cm
self.grid_size = 3            # 3×3
self.obstacles = {(1, 1)}     # Ostacolo in (1,1)
self.start_cell = (0, 0)      # Partenza
self.goal_cell = (1, 2)       # Arrivo

# Velocità (conservative per test)
self.rotation_speed = 0.3     # 30%
self.linear_speed = 0.2       # 20%

# Tolleranze
self.angle_tolerance = 0.15   # ~8.6°
self.position_tolerance = 0.15 # 15cm
```

## 📊 Tempo Stimato Missione

| Fase | Tempo Stimato |
|------|---------------|
| Pianificazione | 1s |
| Navigazione → GOAL (3 celle) | ~15-20s |
| Raccolta oggetto | 6s |
| Navigazione → START (3 celle) | ~15-20s |
| Consegna oggetto | 5s |
| **TOTALE** | **~45-55 secondi** |

## 🔍 Cosa Osservare

Durante il test, verifica:

- ✅ Robot ruota correttamente verso direzione cardinale
- ✅ Si muove dritto senza deviare troppo
- ✅ Si ferma al centro di ogni cella
- ✅ Rileva ostacolo con ultrasuono (se si avvicina)
- ✅ Braccio/pinza funzionano correttamente
- ✅ Torna alla posizione iniziale

## ⚠️ Problemi Comuni

### Robot non raggiunge centro cella
- **Causa**: Velocità non calibrata
- **Soluzione**: Esegui `test_robot.py` → Test 5 (calibrazione)

### Robot deriva durante movimento
- **Causa**: Superficie irregolare o ruote slittano
- **Soluzione**: Aumenta `position_tolerance` a 0.20m

### Ostacolo non rilevato
- **Causa**: Ultrasuono fuori range o ostacolo troppo basso
- **Soluzione**: Usa ostacolo >15cm altezza, riduci `obstacle_threshold` a 0.30m

## 🚀 Avvio Test

```bash
# Sulla Raspberry Pi
cd /home/pi/courier_robot

# Posiziona robot in START (0,0) orientato verso Est
# Premi Invio quando pronto

python3 robot_controller.py
```

## 📝 Note

- Questa è una configurazione **TEST** semplificata
- Dopo validazione, puoi passare a griglia 5×5 più complessa
- Mantieni sempre spazio libero 50cm attorno alla griglia
