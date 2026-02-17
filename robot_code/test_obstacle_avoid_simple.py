#!/usr/bin/env python3
"""
Test Semplice: Va dritto, rileva ostacolo, aggira
"""
import time
from bt.actions import rover, DEFAULT_SPEED_LINEAR, DEFAULT_SPEED_TURN

def check_obstacle(threshold=40):
    """Controlla se c'è ostacolo davanti"""
    distance = rover.getUltrasonicSensor()
    print(f"   📡 Distanza: {distance:.1f}cm")
    return distance < threshold

def go_straight(duration=2.0, left_factor=1.0, right_factor=0.08):
    """Va dritto per tot secondi, controlla ostacoli"""
    print(f"➡️  Vado dritto per {duration}s (L:{left_factor:.2f} R:{right_factor:.2f})...")
    
    # Movimento con compensazione calibrata
    speed_left = DEFAULT_SPEED_LINEAR * left_factor
    speed_right = DEFAULT_SPEED_LINEAR * right_factor
    
    rover.moveTo('Forward', speed_left, speed_right)
    
    start = time.time()
    while time.time() - start < duration:
        time.sleep(0.5)
        
        if check_obstacle():
            rover.stop()
            print("🚧 OSTACOLO RILEVATO!")
            return True  # Ostacolo trovato
    
    rover.stop()
    print("✓ Tratto completato, nessun ostacolo")
    return False

def rotate_90_right(rotation_time=4.5):
    """Rotazione 90° a DESTRA usando rotazioni normali (non differenziali)"""
    print(f"   🔄 Rotazione 90° destra ({rotation_time}s)...")
    rover.moveTo('Right', DEFAULT_SPEED_TURN)
    time.sleep(rotation_time)
    rover.stop()
    time.sleep(0.3)

def rotate_90_left(rotation_time=4.5):
    """Rotazione 90° a SINISTRA usando rotazioni normali (non differenziali)"""
    print(f"   🔄 Rotazione 90° sinistra ({rotation_time}s)...")
    rover.moveTo('Left', DEFAULT_SPEED_TURN)
    time.sleep(rotation_time)
    rover.stop()
    time.sleep(0.3)

def check_direction(direction_name, threshold=35):
    """Controlla e stampa distanza in una direzione"""
    distance = rover.getUltrasonicSensor()
    is_free = distance >= threshold
    status = "✅ LIBERO" if is_free else "❌ BLOCCATO"
    print(f"      {direction_name}: {distance:.1f}cm {status}")
    return is_free

def avoid_obstacle(rotation_time=4.5, lateral_move_time=3.5, left_factor=1.0, right_factor=0.95):
    """
    Logica INTELLIGENTE: prova tutte le direzioni
    1. Ostacolo NORD → controlla EST e OVEST
    2. Scegli direzione libera
    3. Se entrambe bloccate → torna NORD e indietreggia
    """
    print("\n🔄 AGGIRO OSTACOLO - Scansione direzioni:")
    
    speed_left = DEFAULT_SPEED_LINEAR * left_factor
    speed_right = DEFAULT_SPEED_LINEAR * right_factor
    
    # 1. Controlla EST (gira destra)
    print("   1️⃣ Controllo direzione EST (destra)...")
    rotate_90_right(rotation_time)
    est_libero = check_direction("EST", threshold=35)
    
    if est_libero:
        # EST libero → vai EST
        print("   ✅ EST libero! Vado in quella direzione.")
        return move_lateral_and_return(rotation_time, lateral_move_time, speed_left, speed_right, "EST", left_factor, right_factor)
    
    # 2. EST bloccato → controlla OVEST (gira sinistra 2 volte da EST)
    print("   2️⃣ EST bloccato. Controllo direzione OVEST (sinistra)...")
    rotate_90_left(rotation_time)  # EST → NORD
    rotate_90_left(rotation_time)  # NORD → OVEST
    
    ovest_libero = check_direction("OVEST", threshold=35)
    
    if ovest_libero:
        # OVEST libero → vai OVEST
        print("   ✅ OVEST libero! Vado in quella direzione.")
        return move_lateral_and_return(rotation_time, lateral_move_time, speed_left, speed_right, "OVEST", left_factor, right_factor)
    
    # 3. EST e OVEST bloccati → torna NORD e indietreggia
    print("   3️⃣ EST e OVEST bloccati. Torno NORD e indietreggio...")
    rotate_90_right(rotation_time)  # OVEST → NORD
    
    # Indietreggia per 2 secondi
    print("      ⬅️  Indietreggio 2s...")
    rover.moveTo('Back', speed_left, speed_right)
    time.sleep(2.0)
    rover.stop()
    time.sleep(0.3)
    
    # Controlla di nuovo NORD
    print("      Controllo NORD dopo indietreggiamento...")
    if check_direction("NORD", threshold=40):
        print("   ✅ NORD ora libero! Continuo.")
        return True
    else:
        print("   ❌ Completamente bloccato! Mi fermo.")
        return False

def move_lateral_and_return(rotation_time, lateral_move_time, speed_left, speed_right, direction_name, left_factor=1.0, right_factor=0.95):
    """
    Movimento laterale con controlli continui, poi ritorno a NORD
    """
    # Avanza lateralmente CON CONTROLLI
    print(f"   🚶 Avanzo {direction_name} per {lateral_move_time}s...")
    rover.moveTo('Forward', speed_left, speed_right)
    
    start = time.time()
    while time.time() - start < lateral_move_time:
        time.sleep(0.5)
        distance = rover.getUltrasonicSensor()
        print(f"      📡 {distance:.1f}cm")
        
        if distance < 30:
            rover.stop()
            print(f"      ⚠️  Ostacolo durante movimento {direction_name}!")
            # Se trova ostacolo, indietreggia un po'
            print("      ⬅️  Indietreggio...")
            rover.moveTo('Back', speed_left, speed_right)
            time.sleep(1.0)
            rover.stop()
            time.sleep(0.3)
            break  # Esce dal movimento laterale
    else:
        rover.stop()
        time.sleep(0.3)
    
    # Torna verso NORD
    if direction_name == "EST":
        print("   🔄 Giro SINISTRA (Est → Nord)")
        rotate_90_left(rotation_time)
    elif direction_name == "OVEST":
        print("   🔄 Giro DESTRA (Ovest → Nord)")
        rotate_90_right(rotation_time)
    
    # Controlla se NORD è libero
    print("   🔍 Controllo NORD...")
    if check_direction("NORD", threshold=40):
        print("   ✅ NORD libero! Riprendo.")
        return True
    else:
        print("   ⚠️  NORD ancora bloccato, continuo lateralmente...")
        # Se NORD ancora bloccato, continua un po' lateralmente
        # (tornerà a controllare al prossimo ciclo)
        return True

def calibrate_forward():
    """Calibrazione movimento dritto - compensa deriva"""
    print("\n🔧 CALIBRAZIONE MOVIMENTO DRITTO")
    print("="*60)
    print("⚙️  Metti un segno sul pavimento davanti al robot")
    print("⚙️  Il robot andrà dritto per 3 secondi")
    print("⚙️  Osserva se va perfettamente dritto o storto")
    print("💡 Cingolo DESTRO più avanti → potrebbe deviare")
    
    risposta = input("\nVuoi calibrare movimento dritto? [s/N]: ").strip().lower()
    if risposta != 's':
        # Default: cingolo destro rallentato per compensare deriva sinistra
        return 1.0, 0.95  # left_factor, right_factor (ridotto da 0.98)
    
    print("\n📍 Premi INVIO per test movimento dritto...")
    input()
    
    # Test con compensazione default
    speed_left = DEFAULT_SPEED_LINEAR * 1.0
    speed_right = DEFAULT_SPEED_LINEAR * 0.95
    
    print(f"🚀 Movimento dritto 3s (Left:{speed_left*100:.0f}% Right:{speed_right*100:.0f}%)...")
    rover.moveTo('Forward', speed_left, speed_right)
    time.sleep(3.0)
    rover.stop()
    time.sleep(0.3)
    
    print("\n❓ Come si è comportato?")
    print("  1 = Devia verso SINISTRA ↖️ (destro troppo veloce)")
    print("  2 = Va perfettamente DRITTO ✅")
    print("  3 = Devia verso DESTRA ↗️ (sinistro troppo veloce)")
    
    risultato = input("\nRisultato [1/2/3]: ").strip()
    
    if risultato == '1':
        print("\n💡 Devia sinistra → rallento ancora DI PIÙ il destro")
        print("   Prova: Left 1.0, Right 0.90-0.93")
        right = input("   Right factor [default 0.92]: ").strip()
        right_factor = float(right) if right else 0.92
        return 1.0, right_factor
    elif risultato == '2':
        print("\n✅ Perfetto! Uso Left 1.0, Right 0.95")
        return 1.0, 0.95
    elif risultato == '3':
        print("\n💡 Devia destra → destro OK, accelero il sinistro o rallento meno il destro")
        print("   Prova: Left 1.0, Right 1.0 (nessuna compensazione)")
        choice = input("   Vuoi inserire valori custom? [s/N]: ").strip().lower()
        if choice == 's':
            left = input("   Left factor [default 1.0]: ").strip()
            right = input("   Right factor [default 1.0]: ").strip()
            left_factor = float(left) if left else 1.0
            right_factor = float(right) if right else 1.0
            return left_factor, right_factor
        else:
            return 1.0, 1.0  # Nessuna compensazione
    else:
        print("\n⚠️  Risposta non valida, uso default: Left 1.0, Right 0.95")
        return 1.0, 0.95

def calibrate_rotation():
    """Calibrazione rapida tempo rotazione 90°"""
    print("\n🔧 CALIBRAZIONE ROTAZIONE 90°")
    print("="*60)
    print("⚙️  Metti un segno sul pavimento allineato col robot")
    print("⚙️  Il robot girerà a destra")
    print("⚙️  Osserva se gira esattamente 90° o più/meno")
    
    risposta = input("\nVuoi calibrare rotazioni? [s/N]: ").strip().lower()
    if risposta != 's':
        return 4.5  # Tempo default RIDOTTO (era 5.5s, troppo)
    
    print("\n📍 Premi INVIO per test rotazione...")
    input()
    
    test_time = 4.5  # Ridotto da 5.5s
    print(f"🚀 Rotazione {test_time}s...")
    rover.moveTo('Right', DEFAULT_SPEED_TURN)
    time.sleep(test_time)
    rover.stop()
    time.sleep(0.3)
    
    print("\n❓ Quanto ha girato?")
    print("  1 = Meno di 90° (60-85°) → serve più tempo")
    print("  2 = Esatto 90° ✅")
    print("  3 = Più di 90° (95-120°) → serve meno tempo")
    
    risultato = input("\nRisultato [1/2/3]: ").strip()
    
    if risultato == '1':
        print("\n💡 Serve più tempo. Prova: 5.2-5.5s")
        new_time = input("   Nuovo tempo [default 5.2]: ").strip()
        return float(new_time) if new_time else 5.2
    elif risultato == '2':
        print("\n✅ Perfetto! Uso 4.5s")
        return 4.5
    elif risultato == '3':
        print("\n💡 Gira troppo. Prova: 3.8-4.2s")
        new_time = input("   Nuovo tempo [default 4.2]: ").strip()
        return float(new_time) if new_time else 4.2
    else:
        print("\n⚠️  Risposta non valida, uso default 4.5s")
        return 4.5

def main():
    print("="*60)
    print("🤖 TEST AGGIRAMENTO OSTACOLI SEMPLICE")
    print("="*60)
    print(f"🔋 Batteria: {rover.getBatteryVoltage():.2f}V")
    print(f"⚙️  Velocità: {DEFAULT_SPEED_LINEAR*100:.0f}% lineare, {DEFAULT_SPEED_TURN*100:.0f}% rotazione")
    print(f"🎯 Obiettivo: Nord → Ostacolo → Est → Nord → ripeti")
    print("="*60)
    
    # Calibrazione rotazione
    rotation_time = calibrate_rotation()
    print(f"\n✅ Tempo rotazione 90°: {rotation_time}s")
    
    # Calibrazione movimento dritto
    left_factor, right_factor = calibrate_forward()
    print(f"\n✅ Compensazione movimento: Left {left_factor:.2f}, Right {right_factor:.2f}")
    
    # Check iniziale lidar
    print("\n🔍 Test lidar iniziale:")
    distance = rover.getUltrasonicSensor()
    print(f"   Distanza: {distance:.1f}cm")
    
    if distance < 50:
        print("⚠️  ATTENZIONE: C'è già un ostacolo vicino!")
        risposta = input("   Continuo comunque? [s/N]: ").strip().lower()
        if risposta != 's':
            return
    
    input("\n⏸️  Premi INVIO per iniziare missione (Ctrl+C per fermare)...")
    
    try:
        iteration = 1
        while True:
            print(f"\n{'='*60}")
            print(f"🔄 ITERAZIONE {iteration} - Direzione: NORD ⬆️")
            print(f"{'='*60}")
            
            # Va dritto monitorando (con compensazione calibrata)
            obstacle_found = go_straight(duration=3.0, left_factor=left_factor, right_factor=right_factor)
            
            if obstacle_found:
                # Aggira ostacolo con tempo e compensazione calibrati
                success = avoid_obstacle(
                    rotation_time=rotation_time,
                    lateral_move_time=3.5,  # Tempo movimento laterale (Est/Ovest)
                    left_factor=left_factor,
                    right_factor=right_factor
                )
                
                if not success:
                    print("\n❌ Impossibile aggirare! Mi fermo.")
                    break
                
                # Riprova ad andare dritto (torna verso Nord)
                print("\n➡️  Riprendo percorso verso Nord...")
            else:
                # Tutto libero, continua
                pass
            
            iteration += 1
            time.sleep(0.5)
    
    except KeyboardInterrupt:
        print("\n⏸️  Test interrotto")
        rover.stop()
    
    finally:
        rover.stop()
        rover.ser.close()
        print("🏁 Test terminato")


if __name__ == "__main__":
    main()
