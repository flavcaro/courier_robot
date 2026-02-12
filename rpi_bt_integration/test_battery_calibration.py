#!/usr/bin/env python3
"""
Test script per verificare la calibrazione automatica basata sulla tensione della batteria.
"""

from rover_API import RoverApi
import time

def test_battery_monitoring():
    """Testa il sistema di monitoraggio e compensazione batteria."""
    
    print("=" * 60)
    print("TEST SISTEMA CALIBRAZIONE BATTERIA")
    print("=" * 60)
    
    # Inizializza rover con compensazione attiva
    print("\n1️⃣  Inizializzazione con compensazione attiva...")
    rover = RoverApi(
        port='/dev/ttyUSB0',
        enable_voltage_compensation=True,
        reference_voltage=7.4,  # Tensione di riferimento (batteria carica)
        min_voltage=6.4          # Tensione minima sicura
    )
    
    # Verifica stato batteria
    print("\n2️⃣  Stato batteria:")
    status = rover.getBatteryStatus()
    print(f"   Tensione: {status['voltage']:.2f}V")
    print(f"   Percentuale: {status['percentage']:.1f}%")
    print(f"   Stato: {status['status']}")
    print(f"   Fattore compensazione: {status['compensation_factor']:.3f}x")
    print(f"   Compensazione: {'✅ ATTIVA' if status['compensation_enabled'] else '❌ DISATTIVA'}")
    
    # Test movimento con compensazione
    print("\n3️⃣  Test movimento con compensazione...")
    print("   Muovo avanti per 2 secondi a velocità 0.7")
    rover.moveTo('Forward', 0.7)
    time.sleep(2)
    rover.stop()
    print("   ✅ Movimento completato")
    
    # Test senza compensazione
    print("\n4️⃣  Test movimento SENZA compensazione...")
    rover.setVoltageCompensation(False)
    print("   Muovo avanti per 2 secondi a velocità 0.7 (no compensazione)")
    rover.moveTo('Forward', 0.7)
    time.sleep(2)
    rover.stop()
    print("   ✅ Movimento completato")
    
    # Riabilita compensazione
    rover.setVoltageCompensation(True)
    
    # Test continuo con monitoraggio
    print("\n5️⃣  Monitoraggio continuo durante movimento...")
    print("   Premi Ctrl+C per terminare")
    
    try:
        while True:
            status = rover.getBatteryStatus()
            print(f"\r   📊 V: {status['voltage']:.2f}V | "
                  f"{status['percentage']:.0f}% | "
                  f"Comp: {status['compensation_factor']:.2f}x | "
                  f"Status: {status['status']}", end='', flush=True)
            
            # Movimento test
            rover.moveTo('Forward', 0.5)
            time.sleep(1)
            rover.moveTo('Back', 0.5)
            time.sleep(1)
            rover.stop()
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("\n\n   ⏹️  Test interrotto dall'utente")
    
    # Chiusura
    print("\n6️⃣  Chiusura connessione...")
    rover.close()
    print("   ✅ Test completato!")
    print("=" * 60)


def test_without_compensation():
    """Testa il robot senza compensazione (comportamento originale)."""
    
    print("\n" + "=" * 60)
    print("TEST SENZA COMPENSAZIONE (modalità originale)")
    print("=" * 60)
    
    rover = RoverApi(
        port='/dev/ttyUSB0',
        enable_voltage_compensation=False  # Disabilita compensazione
    )
    
    status = rover.getBatteryStatus()
    print(f"\n📊 Tensione: {status['voltage']:.2f}V "
          f"({status['percentage']:.1f}%) - Compensazione DISATTIVA")
    
    print("\nMovimento avanti per 2 secondi...")
    rover.moveTo('Forward', 0.7)
    time.sleep(2)
    rover.stop()
    
    rover.close()
    print("✅ Test completato")
    print("=" * 60)


if __name__ == "__main__":
    import sys
    
    print("\n🤖 BATTERY CALIBRATION TEST SUITE")
    print("\nScegli un test:")
    print("  1 - Test completo con monitoraggio")
    print("  2 - Test senza compensazione")
    print("  q - Esci")
    
    choice = input("\nScelta: ").strip()
    
    if choice == "1":
        test_battery_monitoring()
    elif choice == "2":
        test_without_compensation()
    else:
        print("❌ Scelta non valida o uscita")
        sys.exit(0)
