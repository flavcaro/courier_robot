#!/usr/bin/env python3
"""
Test Diagnostico Batteria - Versione Pulita
===========================================
Legge la tensione batteria usando RoverApi.getBatteryVoltage()
e fa una piccola analisi (media/min/max/varianza + suggerimenti).
"""

import sys
import os
import time
import platform

# Aggiungi path per importare rover_API
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from rover_API import RoverApi


def safe_float(x):
    try:
        return float(x)
    except Exception:
        return None


def test_battery_diagnostics(port="/dev/ttyUSB0", samples=20, interval_s=0.5):
    print("=" * 70)
    print("🔬 TEST DIAGNOSTICO BATTERIA - ARDUINO MEGA")
    print("=" * 70)
    print()

    # Connessione
    print(f"📡 Connessione a {port} ...")
    rover = RoverApi(port=port)
    print("✅ Connesso!")
    print()

    print("=" * 70)
    print(f"TEST: LETTURA TENSIONE BATTERIA ({samples} campioni)")
    print("=" * 70)
    print(f"📊 Leggo la tensione per ~{samples * interval_s:.1f} secondi...\n")

    voltages = []
    for i in range(samples):
        v = rover.getBatteryVoltage()
        v = safe_float(v)
        voltages.append(v)

        if v is None:
            print(f"  [{i+1:2d}] lettura = ERRORE")
        else:
            print(f"  [{i+1:2d}] tensione = {v:.2f}V")

        time.sleep(interval_s)

    # Filtra letture valide
    valid = [v for v in voltages if v is not None]

    print("\n" + "=" * 70)
    print("📊 ANALISI RISULTATI")
    print("=" * 70)
    print()

    if not valid:
        print("❌ Nessuna lettura valida.")
        print("Possibili cause:")
        print("- Baudrate errato (Arduino a 9600? 115200?)")
        print("- Firmware non risponde a 'getBattery'")
        print("- Porta seriale errata (/dev/ttyUSB0 vs /dev/ttyACM0)")
        return

    avg_v = sum(valid) / len(valid)
    min_v = min(valid)
    max_v = max(valid)
    variance = max_v - min_v

    print(f"Campioni validi: {len(valid)}/{len(voltages)}")
    print(f"Media:    {avg_v:.2f}V")
    print(f"Minimo:   {min_v:.2f}V")
    print(f"Massimo:  {max_v:.2f}V")
    print(f"Varianza: {variance:.2f}V")
    print()

    print("🔍 INTERPRETAZIONE:")
    print()

    # Range tipico LiPo 2S: 6.0V (scarica) - 8.4V (carica piena)
    if avg_v < 0.5:
        print("❌ Valore quasi 0V.")
        print("→ Probabile batteria assente / pin non collegato / comando non corretto.")
        print("→ Prima di usare la compensazione, risolvi la lettura.")
    elif 0.5 <= avg_v < 6.0:
        print("⚠️ Valore sotto 6V (non tipico per una LiPo 2S sotto carico leggero).")
        print("→ Possibile partitore/scala errata oppure batteria molto scarica.")
        print("→ Consiglio: misura con multimetro ai capi batteria per confronto.")
    elif 6.0 <= avg_v <= 8.4:
        print("✅ Range coerente con batteria LiPo 2S.")
        if avg_v >= 7.4:
            print("🔋 Stato: CARICA (≥7.4V)")
        elif avg_v >= 7.0:
            print("🔋 Stato: BUONA (7.0–7.4V)")
        elif avg_v >= 6.4:
            print("🔋 Stato: MEDIA (6.4–7.0V)")
        else:
            print("🔴 Stato: BASSA (<6.4V) → ricarica consigliata")
    else:
        print("🟠 Valore sopra 8.4V.")
        print("→ O batteria non è 2S (es. 3S) oppure scala/partitore errato.")
        print("→ Verifica hardware.")

    print()
    if variance > 0.5:
        print("⚡ Varianza alta (>0.5V) → tensione instabile.")
        print("→ Se stai muovendo i motori mentre leggi, è normale vedere 'sag'.")
        print("→ Se sei fermo e varia tanto: controlla contatti/batteria.")
    else:
        print("✅ Varianza bassa → lettura stabile.")

    print("\n" + "=" * 70)
    print("⚙️  COMPENSAZIONE TENSIONE (SUGGERITA)")
    print("=" * 70)
    print()

    reference_v = 7.4
    if avg_v >= 6.0:
        comp = reference_v / avg_v
        # limite per non “esagerare” e aumentare consumi
        comp = min(max(comp, 1.0), 1.25)

        base_rot90 = 5.9
        base_cell = 4.5

        print(f"Fattore compensazione: {comp:.3f}x (ref {reference_v}V)")
        print(f"- Rotazione 90°: {base_rot90:.2f}s → {base_rot90 * comp:.2f}s")
        print(f"- Movimento 0.6m: {base_cell:.2f}s → {base_cell * comp:.2f}s")

        if comp > 1.15:
            print("\n⚠️ Compensazione alta (>15%) → batteria scarica o sag forte.")
    else:
        print("Compensazione non consigliata: tensione troppo bassa o lettura sospetta.")

    print("\n" + "=" * 70)
    print("🏁 TEST COMPLETATO")
    print("=" * 70)
    print("\n📋 PROSSIMI PASSI:")

    if avg_v >= 6.0:
        print("✅ La lettura batteria sembra sensata.")
        print("→ Se però la batteria 'finisce subito', il problema è più probabilmente consumo/picchi motori.")
        print("→ Applica il profilo ECO (speed < 0.5) e correggi moveTo (l’OR).")
    else:
        print("❌ La lettura batteria non è affidabile.")
        print("→ Verifica comando nel firmware (getBattery vs battery).")
        print("→ Verifica cablaggio/partitore e confronta con multimetro.")

    # chiusura seriale (senza rover.close)
    try:
        rover.ser.close()
    except Exception:
        pass


if __name__ == "__main__":
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = "COM3" if platform.system() == "Windows" else "/dev/ttyUSB0"

    print(f"🔌 Porta seriale: {port}")
    print("   (puoi cambiarla con: python3 test_battery_diagnostics.py <porta>)\n")

    try:
        test_battery_diagnostics(port=port)
    except KeyboardInterrupt:
        print("\n\n⏸️  Test interrotto dall'utente")
