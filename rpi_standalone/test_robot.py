#!/usr/bin/env python3
"""
Script di test per verificare funzionalità base del robot.
Esegui questo prima di avviare la missione completa.
"""

from rover_API import RoverApi
import time
import sys


def test_connection(port='/dev/ttyUSB0'):
    """Test 1: Connessione seriale."""
    print("\n" + "="*60)
    print("TEST 1: Connessione Seriale")
    print("="*60)
    
    try:
        rover = RoverApi(port=port)
        print("✅ Connessione riuscita!")
        return rover
    except Exception as e:
        print(f"❌ Errore connessione: {e}")
        print(f"\n💡 Suggerimenti:")
        print(f"   - Verifica che Arduino sia connesso")
        print(f"   - Prova: ls /dev/ttyUSB* o ls /dev/ttyACM*")
        print(f"   - Controlla permessi: sudo usermod -a -G dialout $USER")
        sys.exit(1)


def test_ultrasonic(rover):
    """Test 2: Sensore ultrasuoni."""
    print("\n" + "="*60)
    print("TEST 2: Sensore Ultrasuoni")
    print("="*60)
    
    print("Lettura 5 campioni...")
    for i in range(5):
        distance = rover.getUltrasonicSensor()
        print(f"  Campione {i+1}: {distance:.1f} cm")
        time.sleep(0.5)
    
    print("✅ Sensore ultrasuoni funzionante!")


def test_movement(rover):
    """Test 3: Movimenti base."""
    print("\n" + "="*60)
    print("TEST 3: Movimenti Base")
    print("="*60)
    print("⚠️  ATTENZIONE: Il robot si muoverà!")
    print("   Assicurati che ci sia spazio libero (1m)")
    
    input("   Premi INVIO per continuare o Ctrl+C per annullare...")
    
    # Test avanti
    print("\n1. Avanti (0.5s a velocità 0.3)...")
    rover.moveTo('Forward', 0.3)
    time.sleep(0.5)
    rover.stop()
    time.sleep(1)
    
    # Test indietro
    print("2. Indietro (0.5s a velocità 0.3)...")
    rover.moveTo('Back', 0.3)
    time.sleep(0.5)
    rover.stop()
    time.sleep(1)
    
    # Test rotazione sinistra
    print("3. Rotazione Sinistra (0.5s a velocità 0.3)...")
    rover.moveTo('Left', 0.3)
    time.sleep(0.5)
    rover.stop()
    time.sleep(1)
    
    # Test rotazione destra
    print("4. Rotazione Destra (0.5s a velocità 0.3)...")
    rover.moveTo('Right', 0.3)
    time.sleep(0.5)
    rover.stop()
    
    print("✅ Movimenti base funzionanti!")


def test_arm(rover):
    """Test 4: Braccio e pinza."""
    print("\n" + "="*60)
    print("TEST 4: Braccio e Pinza")
    print("="*60)
    print("⚠️  ATTENZIONE: Il braccio si muoverà!")
    
    input("   Premi INVIO per continuare o Ctrl+C per annullare...")
    
    print("\n1. Sollevamento braccio...")
    rover.armUP()
    time.sleep(2)
    
    print("2. Apertura pinza...")
    rover.openHand(1000)
    time.sleep(1.5)
    
    print("3. Chiusura pinza...")
    rover.closeHand(1500)
    time.sleep(2)
    
    print("4. Abbassamento braccio...")
    rover.armDown()
    time.sleep(2)
    
    print("✅ Braccio e pinza funzionanti!")


def test_calibration(rover):
    """Test 5: Calibrazione velocità."""
    print("\n" + "="*60)
    print("TEST 5: Calibrazione Velocità")
    print("="*60)
    print("Questo test aiuta a calibrare la velocità del robot.")
    print("\n📏 Procedura:")
    print("   1. Posiziona il robot su una superficie piana")
    print("   2. Segna la posizione iniziale")
    print("   3. Il robot si muoverà avanti per 3 secondi")
    print("   4. Misura la distanza percorsa")
    
    input("\n   Premi INVIO quando pronto o Ctrl+C per saltare...")
    
    print("\n⏱️  Movimento in corso (3 secondi)...")
    rover.moveTo('Forward', 0.3)
    time.sleep(3)
    rover.stop()
    
    print("\n📏 Misura la distanza percorsa dal robot.")
    distance_str = input("   Inserisci distanza in cm (es. 45): ")
    
    try:
        distance_cm = float(distance_str)
        distance_m = distance_cm / 100.0
        velocity = distance_m / 3.0  # m/s
        
        print(f"\n📊 Risultati calibrazione:")
        print(f"   Distanza percorsa: {distance_cm} cm ({distance_m:.2f} m)")
        print(f"   Velocità stimata: {velocity:.3f} m/s a speed=0.3")
        print(f"   Velocità normalizzata: {velocity/0.3:.3f} m/s a speed=1.0")
        
        print(f"\n💡 Aggiorna navigation.py:")
        print(f"   linear_velocity = speed * {velocity/0.3:.3f}  # m/s")
        
    except ValueError:
        print("⚠️  Input non valido, calibrazione saltata")


def main():
    """Esegue tutti i test."""
    print("="*60)
    print("🧪 SUITE DI TEST ROBOT MAKEBLOCK")
    print("="*60)
    print("\nQuesto script testerà:")
    print("  1. Connessione seriale")
    print("  2. Sensore ultrasuoni")
    print("  3. Movimenti base")
    print("  4. Braccio e pinza")
    print("  5. Calibrazione velocità")
    
    # Chiedi porta seriale
    port = input("\nPorta seriale [/dev/ttyUSB0]: ").strip()
    if not port:
        port = '/dev/ttyUSB0'
    
    try:
        # Test 1: Connessione
        rover = test_connection(port)
        
        # Test 2: Ultrasuoni
        test_ultrasonic(rover)
        
        # Test 3: Movimenti
        test_movement(rover)
        
        # Test 4: Braccio
        test_arm(rover)
        
        # Test 5: Calibrazione
        test_calibration(rover)
        
        # Chiudi connessione
        rover.close()
        
        print("\n" + "="*60)
        print("✅ TUTTI I TEST COMPLETATI CON SUCCESSO!")
        print("="*60)
        print("\n🚀 Ora puoi avviare la missione completa:")
        print("   python3 robot_controller.py")
        
    except KeyboardInterrupt:
        print("\n\n⏸️  Test interrotti dall'utente")
        try:
            rover.close()
        except:
            pass
    except Exception as e:
        print(f"\n❌ Errore durante i test: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
