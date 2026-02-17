"""
Main controller per missione delivery con navigazione su griglia.
Integra il sistema esistente in ~/robot_code/bt/ con navigazione avanzata.
"""
import py_trees
import time
from bt.navigation_behaviours import create_delivery_mission, create_simple_navigation_test
from bt.sensors import robot_state
from bt.actions import rover


def main():
    """Main entry point per missione delivery."""
    
    print("="*60)
    print("🤖 COURIER ROBOT - DELIVERY MISSION")
    print("="*60)
    print(f"📍 Griglia: {robot_state.grid_size}×{robot_state.grid_size} celle da {robot_state.cell_size}m")
    print(f"🎯 Missione: {robot_state.start_cell} → {robot_state.goal_cell} → {robot_state.start_cell}")
    print(f"🚧 Ostacoli: {robot_state.obstacles}")
    print("="*60)
    print()
    
    # Chiedi quale missione eseguire
    print("Seleziona missione:")
    print("  1. Test navigazione semplice (vai a goal e torna)")
    print("  2. Missione completa (pickup + delivery)")
    
    choice = input("Scelta [1/2]: ").strip()
    
    if choice == "1":
        print("\n🧪 Avvio test navigazione...\n")
        tree = create_simple_navigation_test()
    else:
        print("\n🚀 Avvio missione completa...\n")
        tree = create_delivery_mission()
    
    # Visualizza albero
    print("Struttura Behavior Tree:")
    print(py_trees.display.unicode_tree(tree, show_status=True))
    print()
    
    input("⏸️  Premi INVIO per iniziare (Ctrl+C per annullare)...")
    
    # Esegui missione
    tick_rate = 10  # Hz
    tick_interval = 1.0 / tick_rate
    
    try:
        while True:
            loop_start = time.time()
            
            # Aggiorna sensori
            robot_state.update_sensors()
            
            # Tick Behavior Tree
            tree.tick_once()
            
            # Controlla stato
            if tree.status == py_trees.common.Status.SUCCESS:
                print("\n" + "="*60)
                print("✅ MISSIONE COMPLETATA CON SUCCESSO!")
                print("="*60)
                rover.stop()
                break
                
            elif tree.status == py_trees.common.Status.FAILURE:
                print("\n" + "="*60)
                print("❌ MISSIONE FALLITA!")
                print("="*60)
                rover.stop()
                break
            
            # Mantieni tick rate
            elapsed = time.time() - loop_start
            if elapsed < tick_interval:
                time.sleep(tick_interval - elapsed)
    
    except KeyboardInterrupt:
        print("\n⏸️  Missione interrotta dall'utente")
        rover.stop()
    
    finally:
        print("🏁 Programma terminato")


if __name__ == "__main__":
    main()
