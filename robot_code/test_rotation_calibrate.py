#!/usr/bin/env python3
"""Calibrazione precisa tempo rotazione 90°"""

from bt.actions import rover, DEFAULT_SPEED_TURN
from bt.sensors import robot_state
from rover_API import RoverApi
import time

print("="*60)
print("🔧 CALIBRAZIONE ROTAZIONE 90°")
print("="*60)
print(f"⚙️  Velocità: {DEFAULT_SPEED_TURN*100:.0f}%")
print(f"🔋 Batteria: {rover.getBatteryVoltage():.2f}V")
print(f"⏱️  Tempo attuale: {robot_state.base_rotation_90_time}s")
print("="*60)

# Verifica batterie
voltage = rover.getBatteryVoltage()
if voltage < 7.2:
    print(f"\n⚠️  ATTENZIONE: Batteria bassa ({voltage:.2f}V)")
    print("    Consigliato ricaricare per calibrazione precisa")
    print("    (Continuo comunque...)")
else:
    print(f"\n✅ Batteria OK ({voltage:.2f}V)")

print("\n📋 PROCEDURA:")
print("1. Metti un segno sul pavimento allineato col robot")
print("2. Premi INVIO per iniziare rotazione")
print("3. Il robot girerà per tempo incrementale")
print("4. Misura visivamente l'angolo finale")

input("\n📍 Tutto pronto? Premi INVIO...")

# Test con tempo crescente per trovare il giusto
test_times = [5.5, 6.0, 6.5, 7.0, 7.5, 8.0]

for test_time in test_times:
    print(f"\n" + "="*60)
    print(f"⏱️  TEST: {test_time}s a {DEFAULT_SPEED_TURN*100:.0f}%")
    print("="*60)
    
    input("Premi INVIO per iniziare rotazione... ")
    
    print("🚀 Rotazione in corso...")
    rover.moveTo('Right', DEFAULT_SPEED_TURN)
    time.sleep(test_time)
    rover.stop()
    time.sleep(0.3)
    
    print(f"✅ Completato dopo {test_time}s")
    print("\n❓ Quanto ha girato?")
    print("  1 = ~45° (metà)")
    print("  2 = ~60-75° (quasi)")
    print("  3 = ~90° esatto! ✅")
    print("  4 = ~100-120° (troppo)")
    print("  5 = >135° (molto troppo)")
    
    risposta = input("\nRisultato [1-5]: ").strip()
    
    if risposta == '3':
        print(f"\n🎉 PERFETTO! Tempo ottimale trovato: {test_time}s")
        print(f"\n🔧 AGGIORNA in bt/sensors.py:")
        print(f"   self.base_rotation_90_time = {test_time}")
        break
    elif risposta == '4' or risposta == '5':
        print(f"\n⚠️  Ha girato troppo con {test_time}s")
        prev_time = test_times[test_times.index(test_time) - 1] if test_times.index(test_time) > 0 else test_time * 0.9
        print(f"💡 Prova valore intermedio: {(prev_time + test_time) / 2:.1f}s")
        break
    elif risposta == '1':
        print(f"⏫ Troppo poco, continuo con tempo maggiore...")
    else:
        print(f"⏫ Quasi, continuo...")

print("\n" + "="*60)
print("📊 RIEPILOGO")
print("="*60)
print(f"Velocità rotazione: {DEFAULT_SPEED_TURN*100:.0f}%")
print(f"Batteria: {rover.getBatteryVoltage():.2f}V")
print("\n💡 NOTA: Se batteria < 7.4V, tempo aumenterà con batterie cariche!")
print("   Ri-calibra dopo ricarica completa per precisione massima")

rover.ser.close()
print("\n👋 Calibrazione completata")
