# 🔋 Gestione Batteria SENZA Pin A0 Collegato

## 📌 SITUAZIONE HARDWARE

**Problema rilevato:** Il pin A0 dell'Arduino NON è collegato alla batteria (voltage divider assente)
- Test diagnostici mostrano: **0V costante**
- Hardware Makeblock Ultimate: circuito monitoraggio batteria **non incluso di default**

---

## ✅ SOLUZIONE IMPLEMENTATA

### 1. **RoverAPI.ino** (Firmware Arduino)
```cpp
else if (cmd == "getBattery"){
    // NOTA: Pin A0 non collegato alla batteria
    // Restituiamo sempre tensione nominale LiPo 2S invece di 0V
    float voltage = 7.4;  // Tensione nominale (7.4V)
    Serial.println(voltage, 2);
}
```
**Comportamento:** Restituisce sempre **7.4V** (tensione nominale LiPo 2S)

### 2. **rover_API.py** (Python API)
```python
def getBatteryVoltage(self):
    self.ser.write("getBattery\n".encode())
    time.sleep(0.1)
    data = self.read()
    try:
        return float(data)  # ← Riceve 7.4V dall'Arduino
    except ValueError:
        return 7.4  # Fallback se errore comunicazione
```
**Comportamento:** Riceve 7.4V e lo restituisce (nessun errore)

### 3. **bt/sensors.py** (Compensazione Batteria)
```python
def update_battery_voltage(self, rover_api):
    """Lettura tensione disabilitata - voltage divider non presente."""
    # Pin A0 non collegato → compensazione sempre 1.0
    # NON stampiamo warning (verrebbe chiamato ad ogni movimento)
    self.compensation_factor = 1.0
    self.rotation_90_time = self.base_rotation_90_time
    self.cell_move_time = self.base_cell_move_time
```
**Comportamento:** Compensazione **sempre disabilitata** (factor = 1.0), nessun warning spam

### 4. **bt/navigation_actions.py** (Chiamate Batteria)
```python
# NOTA: Compensazione batteria disabilitata (voltage divider non installato)
# robot_state.update_battery_voltage(rover)  # ← COMMENTATO
```
**Comportamento:** Non chiama più `update_battery_voltage()` ad ogni movimento

---

## 🎯 COME FUNZIONA ADESSO

### ✅ Vantaggi:
1. **Nessun errore** - `getBatteryVoltage()` restituisce sempre 7.4V
2. **Nessun warning spam** - Non stampa più messaggi ad ogni movimento
3. **Compensazione disabilitata** - Tempi movimento fissi (non cambiano con tensione)
4. **Codice pulito** - Tutto funziona come prima ma senza hardware sensore

### ⚠️ Limitazioni:
1. **Non puoi monitorare batteria in tempo reale** - valore sempre 7.4V
2. **Nessun avviso batteria scarica automatico** - devi controllare manualmente
3. **Movimenti non compensati** - se batteria scende, robot rallenta (normale)

---

## 📊 COME CAPIRE QUANDO LA BATTERIA È SCARICA

Senza sensore A0, devi usare **indicatori indiretti**:

### 1. **Tempo di Utilizzo**
```
Batteria LiPo 2S tipica: 2200mAh
Consumo medio con ECO mode (40%): ~1.2A

Autonomia teorica = 2200mAh / 1200mA ≈ 1.8h (110 minuti)
```
**Regola pratica:** Ricarica dopo **90 minuti** di uso continuo

### 2. **Comportamento del Robot**
| Sintomo | Causa | Azione |
|---------|-------|--------|
| Robot diventa più lento | Tensione batteria scesa | Ricarica subito |
| Movimenti meno precisi | Voltage sag sotto carico | Ricarica |
| Raspberry si riavvia | Undervoltage CPU | Ricarica URGENTE |
| LED Arduino tremola | Tensione instabile | Ricarica URGENTE |

### 3. **Conta Missioni**
```python
# Aggiungi al tuo codice un contatore
missioni_completate = 0

def run_mission():
    global missioni_completate
    # ... tua missione ...
    missioni_completate += 1
    
    if missioni_completate >= 6:
        print("🔋 RICARICA BATTERIA (6+ missioni completate)")
```

### 4. **Test Manuale Tensione**
Con un **multimetro digitale**:
```
Batteria LiPo 2S (7.4V nominale):
- 8.4V  → Carica completa (100%)
- 7.4V  → Carica buona (50-70%)
- 7.0V  → Media (30-50%)
- 6.4V  → Scarica (ricarica subito!)
- <6.0V → DANNEGGIATA (non usare!)
```

---

## 🔧 OPZIONE FUTURA: Installare Voltage Divider

Se vuoi **monitoraggio batteria reale**, aggiungi questo circuito:

### Hardware Necessario:
- 2x Resistenze 10kΩ (1/4W, 5% tolleranza)
- Cavi jumper maschio-femmina
- (Opzionale) Condensatore 100nF per filtro rumore

### Schema Circuito:
```
Batteria (+) ───┬──── [R1: 10kΩ] ────┬──── Pin A0 Arduino
                │                      │
                │                      └──── [R2: 10kΩ] ──── GND
                │
                └──── Resto del robot (alimentazione normale)
```

### Perché serve il divisore?
- Batteria LiPo 2S: **max 8.4V** (carica completa)
- Arduino Mega pin analogico: **max 5V** (oltre = danno)
- Divisore 2:1 → 8.4V diventa **4.2V** (sicuro per Arduino)

### Dopo installazione hardware:

1. **Carica il firmware originale:**
```cpp
else if (cmd == "getBattery"){
    // Leggi tensione batteria
    int adcValue = analogRead(BATTERY_PIN);
    float voltage = (adcValue / 1023.0) * ARDUINO_VREF * VOLTAGE_DIVIDER_FACTOR;
    Serial.println(voltage, 2);
}
```

2. **Riabilita compensazione in sensors.py:**
```python
def update_battery_voltage(self, rover_api):
    """Aggiorna fattore compensazione in base a tensione batteria."""
    try:
        voltage = rover_api.getBatteryVoltage()
        self.current_voltage = voltage
        
        # Calcola compensazione solo se tensione valida
        if voltage > self.min_valid_voltage:
            self.compensation_factor = self.reference_voltage / voltage
            # Limita compensazione max
            self.compensation_factor = min(self.compensation_factor, self.max_compensation)
        
        # Applica compensazione ai tempi
        self.rotation_90_time = self.base_rotation_90_time * self.compensation_factor
        self.cell_move_time = self.base_cell_move_time * self.compensation_factor
        
        # Warning batteria bassa
        if voltage < 6.8:
            print(f"⚠️ Batteria in esaurimento: {voltage:.2f}V")
            
    except Exception as e:
        print(f"❌ Errore lettura batteria: {e}")
```

3. **Riattiva chiamata in navigation_actions.py:**
```python
# Riabilita questa riga (togli commento):
robot_state.update_battery_voltage(rover)
```

4. **Testa con test_a0_diagnostics.py** per verificare lettura corretta

---

## 📝 RIEPILOGO

### Configurazione Attuale (SENZA voltage divider):
✅ Firmware restituisce 7.4V fisso  
✅ Compensazione batteria disabilitata  
✅ Nessun warning spam nei log  
✅ Monitoraggio manuale tramite tempo utilizzo  
✅ **Autonomia stimata: 6-8 missioni (90 min) con ECO mode**  

### Quando Ricaricare (INDICATORI):
1. ⏱️ Dopo **90 minuti** di utilizzo continuo
2. 🔢 Dopo **6-8 missioni** complete
3. 🐌 Robot diventa notevolmente più lento
4. 🔴 LED Raspberry lampeggia (undervoltage)
5. 📏 Multimetro mostra **< 7.0V** ai capi batteria

### Se Vuoi Monitoraggio Hardware:
→ Segui sezione "Installare Voltage Divider"  
→ Costo: ~2€ (2 resistenze)  
→ Tempo: 10 minuti saldatura/connessione  
→ Risultato: Monitoraggio real-time + compensazione automatica  

---

## 🚀 File da Caricare su Arduino

**Dopo le modifiche**, ricarica il firmware:

```powershell
# Da Windows (Arduino IDE)
1. Apri RoverAPI.ino
2. Seleziona Board: "Arduino Mega 2560"
3. Seleziona Porta: COM<X> (quella del robot)
4. Click Upload (→)
```

**OPPURE** usa lo script (se hai arduino-cli):
```powershell
arduino-cli compile --fqbn arduino:avr:mega RoverAPI.ino
arduino-cli upload -p COM<X> --fqbn arduino:avr:mega RoverAPI.ino
```

---

**✅ Ora il sistema funziona perfettamente anche senza voltage divider!**
