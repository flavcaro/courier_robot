#!/usr/bin/env python3
"""
Script di test per verificare la lettura della tensione della batteria
e il sistema di compensazione automatica.
"""

from rover_API import RoverApi
import time

def test_battery_reading():
    """Test lettura tensione batteria."""
    print("=" * 50)
    print("TEST LETTURA TENSIONE BATTERIA")
    print("=" * 50)
    print()
    
    # Connetti al robot
    print("📡 Connessione a /dev/ttyUSB0...")
    rover = RoverApi(port='/dev/ttyUSB0')
    print("✅ Connesso!")
    print()
    
    # Leggi tensione 5 volte per verificare stabilità
    print("🔋 Lettura tensione batteria (5 campioni):")
    voltages = []
    for i in range(5):
        voltage = rover.getBatteryVoltage()
        voltages.append(voltage)
        print(f"   Campione {i+1}: {voltage:.2f}V")
        time.sleep(0.5)
    
    # Calcola media
    avg_voltage = sum(voltages) / len(voltages)
    print()
    print(f"📊 Tensione media: {avg_voltage:.2f}V")
    print()
    
    # Valutazione stato batteria
    if avg_voltage >= 7.4:
        print("✅ Batteria CARICA (≥7.4V)")
        compensation = 1.0
    elif avg_voltage >= 7.0:
        print("🟡 Batteria BUONA (7.0-7.4V)")
        compensation = 7.4 / avg_voltage
    elif avg_voltage >= 6.4:
        print("🟠 Batteria MEDIA (6.4-7.0V)")
        compensation = 7.4 / avg_voltage
    else:
        print("🔴 Batteria BASSA (<6.4V) - RICARICARE!")
        compensation = 7.4 / avg_voltage
    
    print()
    print(f"⚙️  Fattore di compensazione: {compensation:.3f}")
    print(f"   Tempo rotazione 90° base: 5.9s")
    print(f"   Tempo rotazione 90° compensato: {5.9 * compensation:.2f}s")
    print()
    
    # Stima autonomia
    if avg_voltage >= 7.0:
        print("🕐 Autonomia stimata: ALTA (>30 minuti)")
    elif avg_voltage >= 6.6:
        print("🕐 Autonomia stimata: MEDIA (15-30 minuti)")
    else:
        print("🕐 Autonomia stimata: BASSA (<15 minuti)")
    
    print()
    print("=" * 50)
    print("TEST COMPLETATO")
    print("=" * 50)

if __name__ == "__main__":
    try:
        test_battery_reading()
    except KeyboardInterrupt:
        print("\n\n⚠️  Test interrotto dall'utente")
    except Exception as e:
        print(f"\n❌ Errore durante il test: {e}")
        import traceback
        traceback.print_exc()
