#!/usr/bin/env python3
"""
Test Navigazione Dinamica con Rilevamento Ostacoli
Ottimizzato per risparmio batteria
"""
import py_trees
import time
from bt.dynamic_navigation import create_battery_optimized_mission
from bt.sensors import robot_state
from bt.actions import rover

def main():
    print("="*60)
    print("🤖 NAVIGAZIONE DINAMICA - TEST")
    print("="*60)
    print(f"📍 Griglia: {robot_state.grid_size}×{robot_state.grid_size}")
    print(f"📏 Dimensione cella: {robot_state.cell_size}m")
    print(f"⚙️  Velocità: 65% lineare, 75% rotazioni")
    print(f"🔋 Batteria: {rover.getBatteryVoltage():.2f}V")
    print("="*60)
    
    # Configurazione missione
    print("\n📋 CONFIGURAZIONE:")
    print("   Start: (0, 0)")
    
    goal_input = input("   Goal [default (1,2)]: ").strip()
    if goal_input:
        try:
            goal = eval(goal_input)
        except:
            print("   ⚠️  Input invalido, uso default")
            goal = (1, 2)
    else:
        goal = (1, 2)
    
    print(f"   Goal: {goal}")
    print(f"   Ostacoli iniziali: {robot_state.obstacles}")
    
    # Mostra griglia iniziale
    robot_state.goal_cell = goal
    robot_state.print_grid()
    
    # Stima autonomia
    estimated_distance = abs(goal[0]) + abs(goal[1])  # Manhattan distance
    estimated_rotations = estimated_distance  # Worst case
    estimated_time = estimated_distance * 3.5 + estimated_rotations * 7
    
    print(f"\n⏱️  STIMA DURATA:")
    print(f"   Tempo viaggio: ~{estimated_time:.0f}s")
    print(f"   Con ritorni + ostacoli: ~{estimated_time * 2.5:.0f}s")
    
    # Warning batteria
    voltage = rover.getBatteryVoltage()
    if voltage < 7.2:
        print(f"\n⚠️  ATTENZIONE: Batteria bassa ({voltage:.2f}V)")
        print("   Consigliato ricaricare prima della missione!")
        risposta = input("   Continuo comunque? [s/N]: ").strip().lower()
        if risposta != 's':
            print("Missione annullata")
            return
    
    # Crea albero
    tree = create_battery_optimized_mission(goal_cell=goal)
    
    print("\n📊 Struttura Behavior Tree:")
    print(py_trees.display.unicode_tree(tree, show_status=True))
    
    input("\n⏸️  Premi INVIO per iniziare (Ctrl+C per annullare)...")
    
    # Esegui missione
    tick_rate = 5  # Hz (ridotto da 10 per risparmiare CPU/batteria)
    tick_interval = 1.0 / tick_rate
    start_mission = time.time()
    
    try:
        while True:
            loop_start = time.time()
            
            # Tick Behavior Tree
            tree.tick_once()
            
            # Controlla stato
            if tree.status == py_trees.common.Status.SUCCESS:
                elapsed = time.time() - start_mission
                print("\n" + "="*60)
                print("✅ MISSIONE COMPLETATA!")
                print("="*60)
                robot_state.print_grid()
                print(f"⏱️  Durata: {elapsed:.1f}s")
                print(f"🔋 Batteria finale: {rover.getBatteryVoltage():.2f}V")
                print(f"   Consumo: {voltage - rover.getBatteryVoltage():.2f}V")
                rover.stop()
                break
                
            elif tree.status == py_trees.common.Status.FAILURE:
                elapsed = time.time() - start_mission
                print("\n" + "="*60)
                print("❌ MISSIONE FALLITA!")
                print("="*60)
                robot_state.print_grid()
                print(f"⏱️  Durata: {elapsed:.1f}s")
                print(f"🔋 Batteria finale: {rover.getBatteryVoltage():.2f}V")
                print(f"🚧 Ostacoli trovati: {robot_state.obstacles}")
                rover.stop()
                break
            
            # Mantieni tick rate
            elapsed = time.time() - loop_start
            if elapsed < tick_interval:
                time.sleep(tick_interval - elapsed)
    
    except KeyboardInterrupt:
        print("\n⏸️  Missione interrotta")
        rover.stop()
    
    finally:
        rover.ser.close()
        print("🏁 Programma terminato")


if __name__ == "__main__":
    main()
