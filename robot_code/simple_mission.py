#!/usr/bin/env python3
"""
Maze Mission - Navigazione Labirinto 
Priorità fissa N > W > E > S, prima direzione libera = scelta

Features:
- Sensore ultrasonic controlla continuamente (threshold 20cm default)
- Robot si ferma quando rileva ostacolo (NON colpisce muri)
- Quando bloccato, controlla direzioni in ordine: N → W → E → S
- Prima direzione libera trovata = scelta finale (efficiente, risparmia batteria)
- Nord prioritizzato perché è l'uscita del maze
- IMU per movimenti precisi
"""
import time
import py_trees
from bt.simple_behaviours import create_main_mission_tree
from bt.simple_state import simple_state
from bt.actions import rover, DEFAULT_SPEED_TURN


def calibrate_system():
    """Calibrazione SEMPLIFICATA - Solo IMU automatico"""
    print("\n" + "="*60)
    print("🔧 CALIBRAZIONE SISTEMA")
    print("="*60)
    
    # Calibrazione IMU automatica
    if simple_state.imu.is_available():
        print("\n🧭 IMU DISPONIBILE - Calibrazione automatica...")
        print("   ⚠️  Robot FERMO su superficie PIANA per 5 secondi")
        input("   Premere INVIO per iniziare...")
        
        # Calibrazione statica (aumentato a 300 campioni per precisione)
        simple_state.imu.calibrate(samples=300)
        
        # Reset heading a Nord (0°)
        print("\n   📍 Posiziona robot orientato a NORD")
        input("   Premere INVIO quando pronto...")
        
        simple_state.imu.reset_heading(0.0)
        time.sleep(0.5)
        
        # Leggi heading stabile
        for _ in range(10):
            simple_state.imu.update_heading()
            time.sleep(0.05)
        
        reference_heading = simple_state.imu.get_heading()
        simple_state.imu_reference_heading = reference_heading
        
        # Abilita IMU per tutto
        simple_state.use_imu_rotation = True
        simple_state.use_imu_odometry = True
        
        print(f"   ✅ IMU calibrato - Heading Nord: {reference_heading:.2f}°")
        print("   ✅ IMU abilitato per rotazioni e odometria")
    else:
        print("\n⚠️  IMU non disponibile - modalità fallback (tempo)")
        simple_state.use_imu_rotation = False
        simple_state.use_imu_odometry = False
    
    print("\n" + "="*60)
    print("✅ Calibrazione completata!")
    print("="*60)


def main():
    """Main entry point per missione labirinto."""

    print("="*60)
    print("🧭 MAZE NAVIGATION MISSION")
    print("="*60)
    print(f"🔋 Batteria: {rover.getBatteryVoltage():.2f}V")
    print(f"⚙️  Velocità: 60% lineare, 75% rotazione")
    print(f"🧭 Strategia: Avanza → Sensore rileva muro → Controllo priorità")
    print(f"   Priorità: N → W → E → S (prima libera = scelta ✓)")
    print("="*60)

    # Chiedi distanza target
    print("\n📏 Imposta parametri missione:")
    target_input = input("   Distanza target dal punto iniziale in metri [default 3.0]: ").strip()
    target_distance = float(target_input) if target_input else 3.0

    simple_state.target_distance = target_distance
    print(f"   🎯 Target: {target_distance}m dal punto iniziale")
    print(f"   📡 Sensore: ferma robot quando rileva ostacolo (<{simple_state.obstacle_threshold:.0f}cm)")
    print(f"   🚶 Step: 0.5m per iterazione (con controllo continuo)")
    print(f"   ⚡ Efficienza: controlla solo fino a prima direzione libera")

    # Calibrazione SEMPLIFICATA
    calibrate_system()

    # Test lidar iniziale
    print("\n🔍 Test lidar iniziale:")
    distance = rover.getUltrasonicSensor()
    print(f"   Distanza: {distance:.1f}cm")

    if distance < 50:
        print("   ⚠️  Ostacolo troppo vicino! Muovi il robot più indietro.")
        return

    # Crea Behavior Tree
    print("\n🌳 Creazione Behavior Tree...")
    tree = create_main_mission_tree(target_distance=target_distance)

    # Visualizza struttura albero
    print("\n📋 Struttura Behavior Tree:")
    print(py_trees.display.unicode_tree(tree.root, show_status=True))
    print()

    input("⏸️  Premi INVIO per iniziare missione (Ctrl+C per fermare)...\n")

    tick_count = 0
    last_status_print = 0

    try:
        while True:
            tree.tick()
            tick_count += 1

            # Ottieni status dall'ultimo nodo eseguito
            status = tree.root.status
            
            # Stampa stato ogni 5 tick
            if tick_count - last_status_print >= 5:
                # Non stampare se siamo in movimento (troppo verbose)
                last_status_print = tick_count

            if status == py_trees.common.Status.SUCCESS:
                print("\n🎉 MISSIONE COMPLETATA!")
                simple_state.print_status()
                break
            elif status == py_trees.common.Status.FAILURE:
                print("\n❌ MISSIONE FALLITA")
                simple_state.print_status()
                break

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n⏸️  Missione interrotta dall'utente")

    finally:
        rover.stop()
        simple_state.print_status()
        print("🏁 Programma terminato\n")


if __name__ == "__main__":
    main()

