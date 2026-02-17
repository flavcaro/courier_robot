#!/usr/bin/env python3
"""
Script di Calibrazione Deriva Laterale
======================================
Testa e compensa il disallineamento dei cingoli.
"""

from rover_API import RoverApi
import time
import sys

# ============================================================================
# PARAMETRI DI CALIBRAZIONE (MODIFICA QUESTI)
# ============================================================================

# Compensazione motori: % velocità da sottrarre al motore più veloce
# Se robot va a SINISTRA → aumenta LEFT_MOTOR_COMPENSATION
# Se robot va a DESTRA → aumenta RIGHT_MOTOR_COMPENSATION
LEFT_MOTOR_COMPENSATION = 0.05   # 5% più lento (aumenta se deriva a sx)
RIGHT_MOTOR_COMPENSATION = 0.00  # Motore di riferimento

# Tempi di test
TEST_FORWARD_TIME = 5.0  # Secondi di movimento rettilineo
TEST_ROTATION_TIME = 3.0  # Secondi di rotazione


# ============================================================================
# FUNZIONI TEST
# ============================================================================

def test_straight_line(rover, speed=0.75, duration=5.0):
    """
    Test movimento rettilineo con compensazione.
    
    Returns:
        str: Istruzioni per correzione deriva
    """
    print("\n" + "="*70)
    print("📏 TEST MOVIMENTO RETTILINEO")
    print("="*70)
    print(f"⚙️  Velocità base: {speed*100:.0f}%")
    print(f"⏱️  Durata: {duration}s")
    print(f"🔧 Compensazione SX: -{LEFT_MOTOR_COMPENSATION*100:.1f}%")
    print(f"🔧 Compensazione DX: -{RIGHT_MOTOR_COMPENSATION*100:.1f}%")
    print("\n📍 ISTRUZIONI:")
    print("1. Posiziona il robot su pavimento liscio")
    print("2. Metti un nastro adesivo dritto come riferimento")
    print("3. Allinea il robot al nastro")
    print("4. Osserva se devia durante il movimento")
    print("\n" + "-"*70)
    
    input("⏸️  Premi INVIO per avviare il test...")
    
    # MOVIMENTO COMPENSATO
    # Invece di mandare "Forward:50" mandiamo comandi separati ai motori
    # (NOTA: Richiede modifica firmware per accettare comandi per motore singolo)
    # Per ora usiamo approccio semplificato:
    
    print("\n🚀 Avvio movimento...")
    print("👀 Osserva attentamente la traiettoria!")
    
    # MOVIMENTO COMPENSATO con velocità separate per i due lati
    speed_left = speed * (1.0 - LEFT_MOTOR_COMPENSATION)
    speed_right = speed * (1.0 - RIGHT_MOTOR_COMPENSATION)
    
    print(f"   Velocità SX: {speed_left*100:.1f}%")
    print(f"   Velocità DX: {speed_right*100:.1f}%")
    
    rover.moveTo('Forward', speed_left, speed_right)
    time.sleep(duration)
    rover.stop()
    
    print("\n✅ Test completato!")
    print("\n" + "="*70)
    print("📊 ANALISI RISULTATO")
    print("="*70)
    print("\n❓ Il robot è andato dritto o ha deviato?")
    print("\nRispondi:")
    print("  L = Deviato a SINISTRA")
    print("  R = Deviato a DESTRA")
    print("  S = Andato DRITTO (perfetto!)")
    
    result = input("\nRisultato [L/R/S]: ").strip().upper()
    
    if result == 'L':
        new_comp = LEFT_MOTOR_COMPENSATION + 0.02
        return f"""
⚠️  DERIVA A SINISTRA rilevata!

🔧 CORREZIONE SUGGERITA:
   Modifica in questo file la linea:
   
   LEFT_MOTOR_COMPENSATION = {new_comp:.2f}  # (era {LEFT_MOTOR_COMPENSATION:.2f})
   
   Poi riesegui il test.
"""
    elif result == 'R':
        new_comp = RIGHT_MOTOR_COMPENSATION + 0.02
        return f"""
⚠️  DERIVA A DESTRA rilevata!

🔧 CORREZIONE SUGGERITA:
   Modifica in questo file la linea:
   
   RIGHT_MOTOR_COMPENSATION = {new_comp:.2f}  # (era {RIGHT_MOTOR_COMPENSATION:.2f})
   
   Poi riesegui il test.
"""
    else:
        return """
✅ MOVIMENTO RETTILINEO PERFETTO!

Valori di compensazione ottimali:
  LEFT_MOTOR_COMPENSATION = {:.2f}
  RIGHT_MOTOR_COMPENSATION = {:.2f}

Copia questi valori in bt/actions.py
""".format(LEFT_MOTOR_COMPENSATION, RIGHT_MOTOR_COMPENSATION)


def test_rotation_drift(rover, direction='right', speed=0.80, duration=3.0):
    """Test deriva durante rotazione."""
    print("\n" + "="*70)
    print(f"🔄 TEST ROTAZIONE {'DESTRA' if direction == 'right' else 'SINISTRA'}")
    print("="*70)
    print(f"⚙️  Velocità: {speed*100:.0f}%")
    print(f"⏱️  Durata: {duration}s")
    print("\n📍 ISTRUZIONI:")
    print("1. Segna la posizione delle ruote con nastro adesivo")
    print("2. Durante rotazione, osserva se il robot si sposta indietro/avanti")
    print("\n" + "-"*70)
    
    input("⏸️  Premi INVIO per avviare...")
    
    print(f"\n🚀 Rotazione {direction}...")
    
    cmd = 'Right' if direction == 'right' else 'Left'
    rover.moveTo(cmd, speed)
    time.sleep(duration)
    rover.stop()
    
    print("\n✅ Test completato!")
    print("\n❓ Durante la rotazione, il robot:")
    print("  1 = Si è spostato INDIETRO")
    print("  2 = È rimasto sul posto (ideale)")
    print("  3 = Si è spostato AVANTI")
    
    result = input("\nRisultato [1/2/3]: ").strip()
    
    if result == '1':
        return """
⚠️  Robot si sposta INDIETRO durante rotazione.

Possibili cause:
- Cingoli non sincronizzati
- Attrito pavimento diverso tra lati
- Centro di massa sbilanciato

🔧 NON serve compensazione software.
   Problema meccanico: verifica hardware!
"""
    elif result == '3':
        return """
⚠️  Robot si sposta AVANTI durante rotazione.

Questo è STRANO! Possibili cause:
- Motori ruotano in verso opposto
- Firmware scambia Left/Right

🔧 Verifica il firmware Arduino!
"""
    else:
        return "✅ Rotazione sul posto perfetta!"


def calibration_wizard(rover):
    """Wizard interattivo per calibrazione completa."""
    print("\n" + "🔧"*35)
    print("CALIBRATION WIZARD - DERIVA LATERALE")
    print("🔧"*35)
    
    print("\nQuesto wizard ti guiderà nella calibrazione step-by-step.")
    print("\n📋 CHECKLIST PRE-TEST:")
    print("  ✓ Batterie cariche (>7.0V)")
    print("  ✓ Pavimento liscio e piano")
    print("  ✓ Spazio libero (almeno 2m davanti)")
    print("  ✓ Nastro adesivo per riferimento")
    
    input("\n⏸️  Tutto pronto? Premi INVIO...")
    
    # Test 1: Movimento rettilineo
    print("\n\n" + "="*70)
    print("FASE 1/3: CALIBRAZIONE MOVIMENTO RETTILINEO")
    print("="*70)
    
    feedback = test_straight_line(rover, speed=0.70, duration=5.0)
    print(feedback)
    
    if "PERFETTO" not in feedback:
        print("\n⚠️  Deriva rilevata! Correggi i valori e riesegui.")
        return
    
    # Test 2: Rotazione destra
    input("\n⏸️  Premi INVIO per Fase 2...")
    print("\n\n" + "="*70)
    print("FASE 2/3: TEST ROTAZIONE DESTRA")
    print("="*70)
    
    feedback = test_rotation_drift(rover, 'right', speed=0.80, duration=3.0)
    print(feedback)
    
    # Test 3: Rotazione sinistra
    input("\n⏸️  Premi INVIO per Fase 3...")
    print("\n\n" + "="*70)
    print("FASE 3/3: TEST ROTAZIONE SINISTRA")
    print("="*70)
    
    feedback = test_rotation_drift(rover, 'left', speed=0.80, duration=3.0)
    print(feedback)
    
    # Riepilogo
    print("\n\n" + "="*70)
    print("📊 CALIBRAZIONE COMPLETATA!")
    print("="*70)
    print("\n✅ Valori finali di compensazione:")
    print(f"   LEFT_MOTOR_COMPENSATION  = {LEFT_MOTOR_COMPENSATION:.3f}")
    print(f"   RIGHT_MOTOR_COMPENSATION = {RIGHT_MOTOR_COMPENSATION:.3f}")
    
    print("\n🔧 PROSSIMI PASSI:")
    print("1. Copia i valori sopra in bt/actions.py")
    print("2. Applica compensazione nel metodo moveTo()")
    print("3. Esegui test_eco_mode.py per verificare")


# ============================================================================
# TEST PATTERN A QUADRATO
# ============================================================================

def test_square_pattern(rover, side_time=3.0, speed_linear=0.75, speed_turn=0.80):
    """
    Test pattern a quadrato: vai avanti → gira 90° destra → ripeti x4.
    Permette di vedere visivamente l'accumulo di errore.
    """
    print("\n" + "="*70)
    print("🔲 TEST PATTERN QUADRATO")
    print("="*70)
    print("Il robot eseguirà 4 movimenti formando un quadrato.")
    print("Se torna al punto di partenza = CALIBRAZIONE OK")
    print("Se devia = necessaria correzione")
    print("\n📍 Segna con nastro adesivo il punto di partenza!")
    
    input("\n⏸️  Premi INVIO per iniziare...")
    
    for i in range(4):
        print(f"\n🔄 Lato {i+1}/4:")
        
        # Muovi avanti
        print(f"   ➡️  Avanti per {side_time}s...")
        rover.moveTo('Forward', speed_linear)
        time.sleep(side_time)
        rover.stop()
        time.sleep(0.5)
        
        # Ruota 90° destra (tempo calibrato per 90°)
        rot_time = 10.5  # Da bt/sensors.py
        print(f"   🔄 Rotazione 90° destra ({rot_time}s)...")
        rover.moveTo('Right', speed_turn)
        time.sleep(rot_time)
        rover.stop()
        time.sleep(0.5)
    
    print("\n✅ Pattern completato!")
    print("\n📏 MISURA L'ERRORE:")
    print("   Distanza dal punto di partenza: ___ cm")
    print("   Angolo di rotazione residuo: ___ gradi")
    
    print("\n💡 INTERPRETAZIONE:")
    print("   < 10cm e < 5° = ECCELLENTE")
    print("   10-30cm o 5-15° = BUONO (accettabile)")
    print("   > 30cm o > 15° = NECESSARIA CALIBRAZIONE")


# ============================================================================
# MAIN
# ============================================================================

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
    
    print("🔌 Connessione a", port)
    rover = RoverApi(port=port)
    print("✅ Connesso!\n")
    
    print("Seleziona test:")
    print("  1. Wizard calibrazione completa")
    print("  2. Test movimento rettilineo")
    print("  3. Test rotazione destra")
    print("  4. Test rotazione sinistra")
    print("  5. Test pattern quadrato")
    
    choice = input("\nScelta [1-5]: ").strip()
    
    try:
        if choice == '1':
            calibration_wizard(rover)
        elif choice == '2':
            feedback = test_straight_line(rover, speed=0.75, duration=5.0)
            print(feedback)
        elif choice == '3':
            feedback = test_rotation_drift(rover, 'right')
            print(feedback)
        elif choice == '4':
            feedback = test_rotation_drift(rover, 'left')
            print(feedback)
        elif choice == '5':
            test_square_pattern(rover)
        else:
            print("❌ Scelta non valida")
    
    except KeyboardInterrupt:
        print("\n\n⚠️  Test interrotto")
        rover.stop()
    
    finally:
        rover.ser.close()
        print("\n👋 Connessione chiusa")


if __name__ == "__main__":
    main()