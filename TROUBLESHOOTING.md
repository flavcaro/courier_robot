## 🔧 Troubleshooting

### **Problema: Robot Non Si Muove Avanti/Indietro**

**Sintomo**: Il robot ruota ma non va avanti né indietro.

**Causa**: Il sensore ME Shutter (bumper) legge sempre `0` (ostacolo rilevato) e blocca i movimenti Forward/Back nel firmware.

**Diagnosi**:
```bash
# Sulla Raspberry Pi
cd ~/robot_code
python3 -c "
from rover_API import RoverApi
rover = RoverApi('/dev/ttyUSB0')
import time
time.sleep(1)
print('Stato Shutter:', rover.getShutter())
rover.close()
"
```

Se ritorna sempre `0`, il sensore blocca il movimento.

**Soluzione**:
1. Carica `RoverAPI_NO_SHUTTER.ino` su Arduino (firmware senza controllo shutter)
2. Oppure: Verifica fisicamente il sensore su PORT_8
3. Oppure: Scollega il sensore shutter se non necessario

---

### **Problema: Batterie Scariche**

**Sintomo**: Robot si muove lentamente o non si muove.

**Soluzione**: Sostituisci tutte le 6 batterie AA con batterie nuove o completamente cariche.

---

### **Problema: Timeout Movimento**

**Sintomo**: `⚠️ Timeout movimento` durante navigazione.

**Causa**: Velocità troppo bassa o odometria non si aggiorna.

**Soluzione**: Aumenta velocità in `sensors.py`:
```python
self.rotation_speed = 0.6
self.linear_speed = 0.5
```
