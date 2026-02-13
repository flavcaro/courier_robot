#!/usr/bin/env python3
"""
Test Modalità ECO - Verifica consumi ridotti

Testa:
1. Movimento avanti (40% velocità)
2. Rotazione destra (35% velocità)  
3. Misura durata effettiva rotazione 90°
4. Misura durata effettiva movimento 60cm
"""

from bt.actions import move_forward, move_right, rover, DEFAULT_SPEED_LINEAR, DEFAULT_SPEED_TURN
from bt.sensors import robot_state
import time

def test_linear_movement():
    """Testa movimento avanti temporizzato."""
    print("\n" + "="*60)
    print("📏 TEST MOVIMENTO LINEARE (60cm)")
    print("="*60)
    print(f"⚙️  Velocità configurata: {DEFAULT_SPEED_LINEAR*100:.0f}%")
    print(f"⏱️  Durata configurata: {robot_state.base_cell_move_time}s")
    print("\n🚀 INIZIO movimento...")
    
    start = time.time()
    rover.moveTo('Forward', DEFAULT_SPEED_LINEAR)
    time.sleep(robot_state.base_cell_move_time)
    rover.stop()
    elapsed = time.time() - start
    
    print(f"✅ COMPLETATO in {elapsed:.2f}s")
    print(f"📐 Misura manualmente la distanza percorsa e confronta con 60cm")
    return elapsed


def test_rotation():
    """Testa rotazione 90° temporizzata."""
    print("\n" + "="*60)
    print("🔄 TEST ROTAZIONE 90° DESTRA")
    print("="*60)
    print(f"⚙️  Velocità configurata: {DEFAULT_SPEED_TURN*100:.0f}%")
    print(f"⏱️  Durata configurata: {robot_state.base_rotation_90_time}s")
    print("\n🚀 INIZIO rotazione...")
    
    start = time.time()
    rover.moveTo('Right', DEFAULT_SPEED_TURN)
    time.sleep(robot_state.base_rotation_90_time)
    rover.stop()
    elapsed = time.time() - start
    
    print(f"✅ COMPLETATO in {elapsed:.2f}s")
    print(f"📐 Verifica visivamente se ha ruotato esattamente 90°")
    return elapsed


def main():
    print("\n" + "🔋"*30)
    print("TEST MODALITÀ ECO - CONSUMI RIDOTTI")
    print("🔋"*30)
    print(f"\n📊 CONFIGURAZIONE:")
    print(f"   Velocità lineare:  {DEFAULT_SPEED_LINEAR*100:.0f}% (era 100%)")
    print(f"   Velocità rotazione: {DEFAULT_SPEED_TURN*100:.0f}% (era 100%)")
    print(f"   Tempo 90°: {robot_state.base_rotation_90_time}s (era 6.5s)")
    print(f"   Tempo 60cm: {robot_state.base_cell_move_time}s (era 2.0s)")
    
    input("\n⏸️  Premi INVIO per iniziare il test movimento...")
    elapsed_fwd = test_linear_movement()
    
    time.sleep(2)
    
    input("\n⏸️  Premi INVIO per iniziare il test rotazione...")
    elapsed_rot = test_rotation()
    
    print("\n" + "="*60)
    print("📋 RIEPILOGO TEST")
    print("="*60)
    print(f"Movimento 60cm: {elapsed_fwd:.2f}s (previsto {robot_state.base_cell_move_time}s)")
    print(f"Rotazione 90°:  {elapsed_rot:.2f}s (previsto {robot_state.base_rotation_90_time}s)")
    
    print("\n" + "🔧"*30)
    print("CALIBRAZIONE:")
    print("🔧"*30)
    
    # Se distanza percorsa NON è 60cm:
    print("\n1️⃣  Se il robot NON ha percorso 60cm:")
    print("   - Misura distanza effettiva (es: 45cm)")
    print("   - Calcola nuovo tempo: 5.0 * (60/45) = 6.67s")
    print("   - Modifica bt/sensors.py: base_cell_move_time = 6.67")
    
    # Se rotazione NON è 90°:
    print("\n2️⃣  Se il robot NON ha ruotato 90°:")
    print("   - Stima angolo effettivo (es: 70°)")
    print("   - Calcola nuovo tempo: 16.0 * (90/70) = 20.57s")
    print("   - Modifica bt/sensors.py: base_rotation_90_time = 20.57")
    
    print("\n3️⃣  Dopo calibrazione, rilancia questo test fino a precisione OK")
    
    print("\n✅ Test completato! Controlla i risultati e calibra se necessario.")
    
    rover.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n⚠️  Test interrotto dall'utente")
        rover.stop()
        rover.close()
    except Exception as e:
        print(f"\n❌ Errore durante test: {e}")
        rover.stop()
        rover.close()
