#!/usr/bin/env python3
"""
Test Diagnostico Motori: Verifica quale cingolo risponde a quale parametro
"""
import time
from bt.actions import rover

def test_motor_sides():
    """Test per capire se i parametri left/right sono invertiti"""
    print("="*60)
    print("🔬 TEST DIAGNOSTICO MOTORI")
    print("="*60)
    print("⚙️  Questo test verifica quale cingolo è LEFT e quale RIGHT")
    print("⚙️  Guarda quale cingolo gira in ogni test")
    print("="*60)
    
    input("\n📍 Metti il robot su un supporto (cingoli sospesi). Premi INVIO...")
    
    # Test 1: Solo parametro LEFT
    print("\n🧪 TEST 1: ForwardComp:100:0 (dovrebbe girare SOLO sinistro)")
    rover.ser.write("ForwardComp:100:0\n".encode())
    time.sleep(2.0)
    rover.stop()
    time.sleep(0.5)
    
    risposta1 = input("❓ Quale cingolo ha girato? [S=sinistro, D=destro]: ").strip().upper()
    
    # Test 2: Solo parametro RIGHT
    print("\n🧪 TEST 2: ForwardComp:0:100 (dovrebbe girare SOLO destro)")
    rover.ser.write("ForwardComp:0:100\n".encode())
    time.sleep(2.0)
    rover.stop()
    time.sleep(0.5)
    
    risposta2 = input("❓ Quale cingolo ha girato? [S=sinistro, D=destro]: ").strip().upper()
    
    # Analisi risultati
    print("\n" + "="*60)
    print("📊 RISULTATI:")
    print("="*60)
    
    if risposta1 == 'S' and risposta2 == 'D':
        print("✅ Parametri CORRETTI!")
        print("   ForwardComp:LEFT:RIGHT funziona come previsto")
        print("\n💡 Il problema è meccanico:")
        print("   - Cingolo destro con MOLTA più trazione")
        print("   - Oppure motore destro più potente")
        print("   - Oppure ruota destra molto più grande")
        return False
    elif risposta1 == 'D' and risposta2 == 'S':
        print("❌ Parametri INVERTITI nel firmware!")
        print("   ForwardComp:100:0 gira il DESTRO invece del SINISTRO")
        print("\n💡 Soluzione: Inverto i fattori nel codice Python")
        return True
    else:
        print("⚠️  Risposta ambigua. Ripeti il test con più attenzione")
        return None

def test_on_ground():
    """Test movimento dritto con parametri invertiti"""
    print("\n" + "="*60)
    print("🧪 TEST MOVIMENTO A TERRA CON INVERSIONE")
    print("="*60)
    
    risposta = input("\nVuoi testare con parametri INVERTITI? [s/N]: ").strip().lower()
    if risposta != 's':
        return
    
    print("\n📍 Metti robot a terra. Premi INVIO...")
    input()
    
    # Test con inversione: left 0.30, right 1.0
    print("\n🚀 Test: Left=0.30 (18%), Right=1.0 (60%)")
    print("   Se va dritto → parametri erano invertiti!")
    
    speed_left = 0.6 * 0.30  # 18%
    speed_right = 0.6 * 1.0  # 60%
    
    rover.moveTo('Forward', speed_left, speed_right)
    time.sleep(3.0)
    rover.stop()
    
    print("\n❓ Come si è comportato?")
    print("  1 = Devia SINISTRA (parametri ancora sbagliati)")
    print("  2 = Va DRITTO ✅ (parametri erano invertiti!)")
    print("  3 = Devia DESTRA (over-compensazione)")
    
    risultato = input("\nRisultato [1/2/3]: ").strip()
    
    if risultato == '2':
        print("\n🎯 BINGO! I parametri del firmware sono INVERTITI!")
        print("   Nel tuo firmware: ForwardComp riceve RIGHT:LEFT invece di LEFT:RIGHT")
        print("\n✅ Soluzione: Invertirò left_factor e right_factor nel codice")
        return True
    else:
        print("\n⚠️  Serve ulteriore analisi...")
        return False

def main():
    print("\n🤖 DIAGNOSTICA MOTORI - Verifica inversione parametri\n")
    
    # Test sospeso
    inverted = test_motor_sides()
    
    if inverted is None:
        print("\n⚠️  Test inconclusivo. Ripeti.")
        rover.ser.close()
        return
    
    # Test a terra se sospetti inversione
    if inverted:
        confirmed = test_on_ground()
        if confirmed:
            print("\n" + "="*60)
            print("✅ DIAGNOSI CONFERMATA: Parametri invertiti nel firmware")
            print("="*60)
    
    rover.ser.close()
    print("\n🏁 Diagnostica completata")

if __name__ == "__main__":
    main()
