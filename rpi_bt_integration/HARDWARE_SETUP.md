# Setup Hardware - Divisore di Tensione per Lettura Batteria

## Panoramica

Per leggere la tensione della batteria LiPo 2S (6.0-8.4V) con l'Arduino MegaPi, è necessario un **divisore di tensione** perché i pin analogici dell'Arduino accettano massimo 5V.

## Schema Collegamento

```
Batteria (+) ──┬─── [Resto del circuito robot]
               │
               └─── R1 (10kΩ) ──┬─── Pin A0 (Arduino)
                                 │
                                 └─── R2 (10kΩ) ──── GND
```

## Componenti Necessari

- **2x Resistenze 10kΩ** (1/4W o superiore)
- Cavi di collegamento
- (Opzionale) Condensatore 100nF per filtrare rumore

## Spiegazione

Il divisore di tensione riduce la tensione della batteria a metà:
- Batteria carica (8.4V) → Pin A0 legge ~4.2V ✅
- Batteria scarica (6.0V) → Pin A0 legge ~3.0V ✅

La formula è: `V_out = V_in × (R2 / (R1 + R2))`

Con R1 = R2 = 10kΩ: `V_out = V_in × 0.5`

## Procedura di Installazione

### 1. Preparazione Componenti

- Prendi 2 resistenze da 10kΩ
- Identifica il polo positivo della batteria (filo rosso)
- Identifica il pin A0 sull'Arduino MegaPi

### 2. Assemblaggio Divisore

**Opzione A - Breadboard (per test):**
1. Collega R1 tra batteria+ e una riga centrale
2. Collega R2 tra la riga centrale e GND
3. Collega il punto centrale (tra R1 e R2) al pin A0

**Opzione B - Saldatura (permanente):**
1. Salda R1 e R2 in serie
2. Isola le saldature con guaina termorestringente
3. Collega:
   - Un capo a batteria+
   - Il punto centrale ad A0
   - L'altro capo a GND

### 3. Verifica Collegamento

Prima di alimentare:
1. Usa un multimetro per verificare la resistenza totale (dovrebbe essere ~20kΩ)
2. Verifica che non ci siano cortocircuiti
3. Controlla che il punto centrale sia isolato

### 4. Test Funzionamento

1. Alimenta il robot
2. Carica il codice Arduino modificato (`RoverAPI.ino`)
3. Esegui lo script di test:
   ```bash
   python3 test_battery.py
   ```
4. Verifica che la tensione letta sia ragionevole (6.0-8.4V)

## Calibrazione VOLTAGE_DIVIDER_FACTOR

Se usi resistenze con valori diversi o se la lettura non è accurata:

1. Misura la tensione reale della batteria con un multimetro: `V_real`
2. Leggi la tensione con `test_battery.py`: `V_read`
3. Calcola il fattore: `VOLTAGE_DIVIDER_FACTOR = V_real / V_read`
4. Aggiorna il valore in `RoverAPI.ino` (riga 18)
5. Ricarica il codice sull'Arduino

## Troubleshooting

### Lettura sempre 0V
- Verifica collegamento pin A0
- Controlla che le resistenze non siano bruciate
- Verifica che la batteria sia collegata

### Lettura troppo alta (>5V)
- **PERICOLO!** Scollega immediatamente
- Verifica che R2 sia collegato a GND
- Controlla che entrambe le resistenze siano da 10kΩ

### Lettura instabile
- Aggiungi condensatore 100nF in parallelo a R2
- Verifica collegamenti saldi
- Allontana cavi da motori (interferenze)

## Note di Sicurezza

⚠️ **IMPORTANTE:**
- Non collegare mai direttamente la batteria (8.4V) al pin A0 (max 5V)!
- Se non sei sicuro, chiedi aiuto
- Testa sempre con multimetro prima di collegare all'Arduino

## Schema Alternativo (Senza Divisore)

Se la tua batteria è 1S LiPo (3.7V nominale, max 4.2V), puoi collegare direttamente:
- Batteria+ → A0
- Imposta `VOLTAGE_DIVIDER_FACTOR = 1.0` nel codice Arduino

**Ma per batterie 2S (7.4V) il divisore è OBBLIGATORIO!**
