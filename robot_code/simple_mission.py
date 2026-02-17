#!/usr/bin/env python3
"""
Simple Mission - Missione Lineare con Aggiramento Ostacoli
Sistema SENZA griglia - solo movimento rettilineo + aggiramento
 
Aggiornato:
- La calibrazione rotazioni usa get_rotation_factors() così è coerente con le rotazioni in missione
  (boost extra sul cingolo che va in retro).
"""
import py_trees
import time
from bt.simple_behaviours import create_main_mission_tree
from bt.simple_state import simple_state
from bt.actions import rover, DEFAULT_SPEED_TURN
 
 
def _turn_speeds_for(direction_str):
    """
    Calcola (speed_left, speed_right) per rotate_differential_compensated(),
    usando i fattori del simple_state, inclusi i boost sul cingolo in retro.
 
    direction_str: 'right' o 'left'
    """
    if direction_str == 'right':
        lf, rf = simple_state.get_rotation_factors('R')  # destra: destro in retro (boost)
    elif direction_str == 'left':
        lf, rf = simple_state.get_rotation_factors('L')  # sinistra: sinistro in retro (boost)
    else:
        raise ValueError("direction_str deve essere 'right' o 'left'")
 
    speed_left = DEFAULT_SPEED_TURN * lf
    speed_right = DEFAULT_SPEED_TURN * rf
    return speed_left, speed_right
 
 
def calibrate_system():
    """
    Calibrazione parametri da test_obstacle_avoid_simple.py
    """
    print("\n" + "="*60)
    print("🔧 CALIBRAZIONE SISTEMA")
    print("="*60)
 
    # Calibrazione rotazione 90° - TEST 360° OBBLIGATORIO
    print("\n1️⃣ CALIBRAZIONE ROTAZIONE 90°")
    print(f"   ⚠️  IMPORTANTE: La carica delle batterie influenza la velocità di rotazione!")
    print(f"   Tempo attuale salvato: {simple_state.rotation_90_time}s")
    print(f"   Si raccomanda di calibrare AD OGNI MISSIONE.")
    risposta = input("\n   Vuoi calibrare con test 360°? [S/n]: ").strip().lower()
 
    if risposta != 'n':
        is_perfect = False
        while not is_perfect:
            print(f"\n   📍 TEST 360°: Il robot farà 4 rotazioni (tempo: {simple_state.rotation_90_time}s per 90°)")
            print(f"   Deve tornare ESATTAMENTE alla direzione iniziale")
            input("   Premi INVIO per iniziare test...")
 
            # ✅ USO FATTORI ROTAZIONE COERENTI CON LA MISSIONE (boost retro incluso)
            speed_left, speed_right = _turn_speeds_for('right')
 
            for i in range(4):
                print(f"      Rotazione {i+1}/4...")
                rover.rotate_differential_compensated('right', speed_left, speed_right)
                time.sleep(simple_state.rotation_90_time)
                rover.stop()
                time.sleep(1.0)
 
            print("\n   ❓ È tornato alla direzione iniziale?")
            print("      1 = Ruotato troppo (guarda a SINISTRA)")
            print("      2 = Perfetto ✅")
            print("      3 = Ruotato poco (guarda a DESTRA)")
 
            risultato_360 = input("   Risultato [1/2/3]: ").strip()
 
            if risultato_360 == '2':
                is_perfect = True
                print(f"   ✅ Calibrazione perfetta! Tempo: {simple_state.rotation_90_time}s")
            elif risultato_360 == '1':
                aggiustamento = input("   Riduzione tempo [-0.05/-0.1/-0.2, default -0.05]: ").strip()
                if aggiustamento:
                    simple_state.rotation_90_time += float(aggiustamento)
                else:
                    simple_state.rotation_90_time -= 0.05
                print(f"   ⚙️  Nuovo tempo: {simple_state.rotation_90_time:.2f}s")
                print("   🔄 Ripeto test...")
            elif risultato_360 == '3':
                aggiustamento = input("   Aumento tempo [+0.05/+0.1/+0.2, default +0.05]: ").strip()
                if aggiustamento:
                    simple_state.rotation_90_time += float(aggiustamento)
                else:
                    simple_state.rotation_90_time += 0.05
                print(f"   ⚙️  Nuovo tempo: {simple_state.rotation_90_time:.2f}s")
                print("   🔄 Ripeto test...")
            else:
                print("   ⚠️  Risposta non valida, ripeto test...")
 
    print(f"   ✅ Tempo rotazione 90°: {simple_state.rotation_90_time:.2f}s")
    
    # Calibrazione separata sinistra/destra
    print("\n   🔧 Calibrazione avanzata: tempi separati sinistra/destra")
    risposta_adv = input("   Vuoi calibrare tempi separati per compensare asimmetrie? [s/N]: ").strip().lower()
    
    if risposta_adv == 's':
        print("\n   📍 Test rotazione SINISTRA (90°)")
        input("   Premi INVIO per test sinistra...")
        
        speed_left, speed_right = _turn_speeds_for('left')
        rover.rotate_differential_compensated('left', speed_left, speed_right)
        time.sleep(simple_state.rotation_90_time)
        rover.stop()
        time.sleep(1.0)
        
        print("\n   ❓ Come si è comportato?")
        print("      1 = Ruotato troppo")
        print("      2 = Perfetto ✅")
        print("      3 = Ruotato poco")
        
        risultato_sx = input("   Risultato [1/2/3]: ").strip()
        
        if risultato_sx == '1':
            aggiustamento = input("   Riduzione tempo sinistra [-0.05/-0.1, default -0.05]: ").strip()
            simple_state.rotation_90_time_left = simple_state.rotation_90_time + (float(aggiustamento) if aggiustamento else -0.05)
        elif risultato_sx == '3':
            aggiustamento = input("   Aumento tempo sinistra [+0.05/+0.1, default +0.05]: ").strip()
            simple_state.rotation_90_time_left = simple_state.rotation_90_time + (float(aggiustamento) if aggiustamento else +0.05)
        else:
            simple_state.rotation_90_time_left = simple_state.rotation_90_time
            
        print("\n   📍 Test rotazione DESTRA (90°)")
        input("   Premi INVIO per test destra...")
        
        speed_left, speed_right = _turn_speeds_for('right')
        rover.rotate_differential_compensated('right', speed_left, speed_right)
        time.sleep(simple_state.rotation_90_time)
        rover.stop()
        time.sleep(1.0)
        
        print("\n   ❓ Come si è comportato?")
        print("      1 = Ruotato troppo")
        print("      2 = Perfetto ✅")
        print("      3 = Ruotato poco")
        
        risultato_dx = input("   Risultato [1/2/3]: ").strip()
        
        if risultato_dx == '1':
            aggiustamento = input("   Riduzione tempo destra [-0.05/-0.1, default -0.05]: ").strip()
            simple_state.rotation_90_time_right = simple_state.rotation_90_time + (float(aggiustamento) if aggiustamento else -0.05)
        elif risultato_dx == '3':
            aggiustamento = input("   Aumento tempo destra [+0.05/+0.1, default +0.05]: ").strip()
            simple_state.rotation_90_time_right = simple_state.rotation_90_time + (float(aggiustamento) if aggiustamento else +0.05)
        else:
            simple_state.rotation_90_time_right = simple_state.rotation_90_time
            
        print(f"   ✅ Tempi calibrati: SX={simple_state.rotation_90_time_left:.2f}s, DX={simple_state.rotation_90_time_right:.2f}s")
 
    # Calibrazione movimento dritto
    print("\n2️⃣ CALIBRAZIONE COMPENSAZIONE MOTORI")
    print("   Default: Left 1.0, Right 0.95")
    risposta = input("   Vuoi calibrare? [s/N]: ").strip().lower()
 
    if risposta == 's':
        print("\n   📍 Il robot andrà dritto per 3 secondi")
        input("   Premi INVIO per test...")
 
        speed_left = 0.6 * 1.0
        speed_right = 0.6 * 0.95
 
        rover.moveTo('Forward', speed_left, speed_right)
        time.sleep(3.0)
        rover.stop()
        time.sleep(0.3)
 
        print("\n   ❓ Come si è comportato?")
        print("      1 = Devia SINISTRA (destro troppo veloce)")
        print("      2 = Va DRITTO ✅")
        print("      3 = Devia DESTRA (sinistro troppo veloce)")
 
        risultato = input("   Risultato [1/2/3]: ").strip()
 
        if risultato == '1':
            right = input("   Right factor [default 0.92]: ").strip()
            simple_state.right_factor = float(right) if right else 0.92
        elif risultato == '3':
            right = input("   Right factor [default 0.98]: ").strip()
            simple_state.right_factor = float(right) if right else 0.98
 
    print(f"   ✅ Compensazione movimento: Left {simple_state.left_factor:.2f}, Right {simple_state.right_factor:.2f}")
 
    # Calibrazione rotazione differenziale compensata
    print("\n3️⃣ CALIBRAZIONE COMPENSAZIONE ROTAZIONE")
    print(f"   Default: Left {simple_state.rotation_left_factor:.2f}, Right {simple_state.rotation_right_factor:.2f}")
    print("   ⚠️  IMPORTANTE: Solleva il robot da terra per vedere entrambi i cingoli!")
    risposta = input("   Vuoi calibrare? [s/N]: ").strip().lower()
 
    if risposta == 's':
        print("\n   📍 Test rotazione: solleva il robot e osserva i cingoli")
        print("   Il robot girerà a destra per 2 secondi")
        input("   Premi INVIO quando il robot è SOLLEVATO...")
 
        # ✅ USO FATTORI ROTAZIONE COERENTI CON LA MISSIONE (boost retro incluso)
        speed_left, speed_right = _turn_speeds_for('right')
 
        print(f"   🔄 Test con Left={speed_left:.2f}, Right={speed_right:.2f}")
        rover.rotate_differential_compensated('right', speed_left, speed_right)
        time.sleep(2.0)
        rover.stop()
        time.sleep(0.3)
 
        print("\n   ❓ Quale cingolo è più LENTO o FERMO?")
        print("      1 = SINISTRO più lento/fermo")
        print("      2 = Entrambi uguali ✅")
        print("      3 = DESTRO più lento/fermo")
        print("      4 = ENTRAMBI si muovono ma troppo piano (aumenta entrambi)")
 
        risultato = input("   Risultato [1/2/3/4]: ").strip()
 
        if risultato == '1':
            print("   💡 Il sinistro è più debole, aumento solo left_factor")
            new_left = input("   Rotation left factor [suggerito 1.20-1.40]: ").strip()
            simple_state.rotation_left_factor = float(new_left) if new_left else 1.30
        elif risultato == '3':
            print("   💡 Il destro è più debole, aumento solo right_factor")
            new_right = input("   Rotation right factor [suggerito 1.20-1.40]: ").strip()
            simple_state.rotation_right_factor = float(new_right) if new_right else 1.30
        elif risultato == '4':
            print("   💡 Aumento entrambi i fattori per più potenza")
            new_both = input("   Fattore per entrambi [suggerito 1.20-1.40]: ").strip()
            factor = float(new_both) if new_both else 1.30
            simple_state.rotation_left_factor = factor
            simple_state.rotation_right_factor = factor
 
        # Test finale a terra
        print("\n   📍 Test finale: POSA il robot a terra")
        input("   Premi INVIO quando il robot è A TERRA...")
 
        # ✅ Ricalcolo velocità dopo eventuale modifica ai fattori
        speed_left, speed_right = _turn_speeds_for('right')
 
        print(f"   🔄 Test rotazione a terra con Left={speed_left:.2f}, Right={speed_right:.2f}")
        rover.rotate_differential_compensated('right', speed_left, speed_right)
        time.sleep(2.0)
        rover.stop()
        time.sleep(0.3)
 
        print("\n   ❓ Il robot ha RUOTATO sul posto?")
        print("      1 = NO, si è solo spostato in avanti (serve più bilanciamento)")
        print("      2 = SÌ, ruota sul posto ✅")
 
        risultato_terra = input("   Risultato [1/2]: ").strip()
 
        if risultato_terra == '1':
            print("   ⚠️  Regola ulteriormente i fattori e riprova")
 
    print(f"   ✅ Compensazione rotazione: Left {simple_state.rotation_left_factor:.2f}, Right {simple_state.rotation_right_factor:.2f}")
 
    print("\n" + "="*60)
    print("✅ Calibrazione completata!")
    print("="*60)
 
 
def main():
    """Main entry point per missione lineare."""
 
    print("="*60)
    print("🤖 SIMPLE LINEAR MISSION")
    print("="*60)
    print(f"🔋 Batteria: {rover.getBatteryVoltage():.2f}V")
    print(f"⚙️  Velocità: 60% lineare, 75% rotazione")
    print("="*60)
 
    # Chiedi distanza target
    print("\n📏 Imposta parametri missione:")
    target_input = input("   Distanza target in metri [default 2.4]: ").strip()
    target_distance = float(target_input) if target_input else 2.4
 
    simple_state.target_distance = target_distance
    print(f"   🎯 Target: {target_distance}m verso Nord")
 
    # Calibrazione
    calibrate_system()
 
    # Test lidar iniziale
    print("\n🔍 Test lidar iniziale:")
    distance = rover.getUltrasonicSensor()
    print(f"   Distanza: {distance:.1f}cm")
 
    if distance < 50:
        print("⚠️  ATTENZIONE: C'è già un ostacolo vicino!")
        risposta = input("   Continuo comunque? [s/N]: ").strip().lower()
        if risposta != 's':
            rover.ser.close()
            return
 
    # Crea Behavior Tree
    print("\n🌳 Creazione Behavior Tree...")
    tree = create_main_mission_tree(target_distance=target_distance)
 
    # Visualizza struttura albero
    print("\n📋 Struttura Behavior Tree:")
    print(py_trees.display.unicode_tree(tree, show_status=True))
    print()
 
    input("⏸️  Premi INVIO per iniziare missione (Ctrl+C per fermare)...\n")
 
    tick_count = 0
    last_status_print = 0
 
    try:
        while True:
            tick_count += 1
 
            if tick_count - last_status_print >= 50:
                simple_state.print_status()
                last_status_print = tick_count
 
            tree.tick_once()
 
            if tree.status == py_trees.common.Status.SUCCESS:
                print("\n" + "="*60)
                print("✅ MISSIONE COMPLETATA CON SUCCESSO!")
                print("="*60)
                simple_state.print_status()
                rover.stop()
                break
 
            elif tree.status == py_trees.common.Status.FAILURE:
                print("\n" + "="*60)
                print("❌ MISSIONE FALLITA!")
                print("="*60)
                simple_state.print_status()
                rover.stop()
                break
 
            time.sleep(0.1)
 
    except KeyboardInterrupt:
        print("\n⏸️  Missione interrotta dall'utente")
        rover.stop()
        simple_state.print_status()
 
    finally:
        rover.ser.close()
        print("🏁 Programma terminato")
 
 
if __name__ == "__main__":
    main()
 
 