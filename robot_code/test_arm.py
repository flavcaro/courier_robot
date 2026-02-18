#!/usr/bin/env python3
"""
Test script per animazione braccio robotico
Testa la sequenza completa: apri pinza → abbassa → chiudi → solleva
"""
import time
from bt.actions import rover, arm_up, arm_down, open_hand, close_hand


def test_arm_sequence():
    """
    Esegue sequenza completa di grab object per test.
    """
    print("\n" + "="*60)
    print("🤖 TEST ANIMAZIONE BRACCIO ROBOTICO")
    print("="*60)
    print("Sequenza:")
    print("  1. Apri pinza")
    print("  2. Abbassa braccio")
    print("  3. Chiudi pinza")
    print("  4. Solleva braccio")
    print("="*60 + "\n")
    
    input("⏸️  Premi INVIO per iniziare animazione...")
    
    # Step 1: Apri pinza
    print("\n1️⃣ Apertura pinza...")
    open_hand(1000)  # Apertura massima
    print("   ✅ Pinza aperta")
    time.sleep(0.5)
    
    # Step 2: Abbassa braccio
    print("\n2️⃣ Abbassamento braccio...")
    arm_down()  # Include sleep(1.5)
    print("   ✅ Braccio abbassato")
    time.sleep(0.5)
    
    # Step 3: Chiudi pinza
    print("\n3️⃣ Chiusura pinza su oggetto...")
    close_hand(1750)  # Presa forte
    print("   ✅ Pinza chiusa")
    time.sleep(0.5)
    
    # Step 4: Solleva braccio
    print("\n4️⃣ Sollevamento braccio a posizione iniziale...")
    arm_up()  # Include sleep(1.5) - simmetrico con arm_down
    print("   ✅ Braccio sollevato completamente")
    
    print("\n" + "="*60)
    print("✅ ANIMAZIONE COMPLETATA!")
    print("="*60 + "\n")


def test_individual_actions():
    """
    Menu per testare singole azioni del braccio.
    """
    print("\n" + "="*60)
    print("🔧 TEST AZIONI INDIVIDUALI BRACCIO")
    print("="*60)
    
    while True:
        print("\nScegli azione:")
        print("  1 - Apri pinza (1000 PWM)")
        print("  2 - Chiudi pinza (1750 PWM)")
        print("  3 - Abbassa braccio")
        print("  4 - Solleva braccio")
        print("  5 - Test sequenza completa")
        print("  0 - Esci")
        
        choice = input("\nSelezione: ").strip()
        
        if choice == "1":
            print("\n🔓 Apertura pinza...")
            open_hand(1000)
            print("   ✅ Completato")
        
        elif choice == "2":
            print("\n🔒 Chiusura pinza...")
            close_hand(1750)
            print("   ✅ Completato")
        
        elif choice == "3":
            print("\n⬇️  Abbassamento braccio...")
            arm_down()
            print("   ✅ Completato")
        
        elif choice == "4":
            print("\n⬆️  Sollevamento braccio...")
            arm_up()
            print("   ✅ Completato")
        
        elif choice == "5":
            test_arm_sequence()
        
        elif choice == "0":
            print("\n👋 Uscita...")
            break
        
        else:
            print("\n❌ Scelta non valida")


def main():
    """Main entry point."""
    print("\n" + "="*60)
    print("🔌 CONNESSIONE AL ROVER")
    print("="*60)
    print(f"Porta: {rover.port}")
    print(f"Batteria: {rover.getBatteryVoltage():.2f}V")
    print("="*60)
    
    # Chiedi modalità
    print("\nModalità test:")
    print("  1 - Test sequenza completa (automatico)")
    print("  2 - Test azioni individuali (menu interattivo)")
    print("  3 - Test ripetuto (loop)")
    
    mode = input("\nSelezione [default=1]: ").strip() or "1"
    
    try:
        if mode == "1":
            test_arm_sequence()
        
        elif mode == "2":
            test_individual_actions()
        
        elif mode == "3":
            print("\n🔁 MODALITÀ LOOP")
            print("Premere Ctrl+C per fermare\n")
            
            count = 0
            while True:
                count += 1
                print(f"\n{'='*60}")
                print(f"🔄 CICLO {count}")
                print(f"{'='*60}")
                
                test_arm_sequence()
                
                time.sleep(2)
                print(f"\n⏱️  Pausa 2 secondi prima del prossimo ciclo...")
                time.sleep(2)
        
        else:
            print("\n❌ Modalità non valida")
    
    except KeyboardInterrupt:
        print("\n\n⏸️  Test interrotto dall'utente")
    
    finally:
        rover.stop()
        print("\n🏁 Test terminato\n")


if __name__ == "__main__":
    main()
