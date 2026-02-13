#!/usr/bin/env python3
"""
Test Diagnostico Pin A0 Arduino
================================
Questo script ti aiuta a capire:
1. Se il pin A0 è connesso alla batteria
2. Se il voltage divider funziona
3. Quale valore aspettarsi
"""

import sys
import os
import time

# Aggiungi il path per importare rover_API
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from rover_API import RoverApi

def test_a0_diagnostics(port='/dev/ttyUSB0'):
    """Test diagnostico completo pin A0."""
    
    print("=" * 70)
    print("🔬 TEST usando RoverApi")
    print(f"📡 Connessione a {port}...")
    rover = RoverApi(port=port)
    ser = rover.ser  # Accesso diretto alla connessione seriale per comandi custom
    try:
        # Connetti
        print(f"📡 Connessione a {port}...")
        ser = serial.Serial(port, 115200, timeout=1)
        time.sleep(2)
        print("✅ Connesso!")
        print()
        
        # Test 1: Lettura RAW
        print("=" * 70)
        print("TEST 1: LETTURA VALORE RAW (0-1023)")
        print("=" * 70)
        print()
        print("Leggo il pin A0 per 10 secondi...")
        print("Se vedi valori che cambiano quando muovi la batteria → è collegato!")
        print()
        
        raw_values = []
        for i in range(20):
            ser.write(b"probeA0\n")
            time.sleep(0.15)
            
            # Leggi risposta
            data = ser.readline().decode('utf-8', errors='ignore').strip()
            
            if data.startswith("RAW:"):
                try:
                    raw = int(data.split(":")[1])
                    raw_values.append(raw)
                    
                    # Calcola tensioni possibili
                    v_direct = raw * 5.0 / 1023.0
                    v_divider_2x = v_direct * 2.0
                    v_divider_4x = v_direct * 4.0
                    
                    print(f"  [{i+1:2d}] RAW={raw:4d} → {v_direct:.2f}V diretti | "
                          f"{v_divider_2x:.2f}V (÷2) | {v_divider_4x:.2f}V (÷4)")
                          
                except:
                    print(f"  [{i+1:2d}] Errore parsing: {data}")
            else:
                print(f"  [{i+1:2d}] Risposta: {data}")
            
            time.sleep(0.35)
        
        print()
        
        # Analisi
        if len(raw_values) == 0:
            print("❌ PROBLEMA: Nessun valore letto!")
            print("   Controlla:")
            print("   - Firmware caricato correttamente")
            print("   - Baudrate corretto (115200)")
            return
        
        avg_raw = sum(raw_values) / len(raw_values)
        min_raw = min(raw_values)
        max_raw = max(raw_values)
        variance = max_raw - min_raw
        
        print("=" * 70)
        print("📊 ANALISI RISULTATI")
        print("=" * 70)
        print()
        print(f"📈 Statistica valori RAW:")
        print(f"   Media:     {avg_raw:.1f}")
        print(f"   Minimo:    {min_raw}")
        print(f"   Massimo:   {max_raw}")
        print(f"   Varianza:  {variance}")
        print()
        
        # Interpretazione
        print("🔍 INTERPRETAZIONE:")
        print()
        
        if avg_raw < 10:
            print("❌ Valore troppo basso (quasi 0V)")
            print("   → Pin A0 probabilmente NON collegato alla batteria")
            print("   → O batteria completamente scarica/assente")
            
        elif 10 <= avg_raw < 100:
            print("⚠️  Valore basso (~0-0.5V)")
            if variance > 20:
                print("   → Pin floating (rumore)")
            else:
                print("   → Possibile tensione residua o ground")
                
        elif 100 <= avg_raw < 400:
            print("🟡 Valore medio-basso (~0.5-2V)")
            voltage_2x = (avg_raw * 5.0 / 1023.0) * 2.0
            voltage_4x = (avg_raw * 5.0 / 1023.0) * 4.0
            print(f"   Se divider ÷2 → Batteria = {voltage_2x:.2f}V (SCARICA!)")
            print(f"   Se divider ÷4 → Batteria = {voltage_4x:.2f}V (OK)")
            
        elif 400 <= avg_raw < 700:
            print("✅ Valore medio (~2-3.5V)")
            voltage_2x = (avg_raw * 5.0 / 1023.0) * 2.0
            voltage_4x = (avg_raw * 5.0 / 1023.0) * 4.0
            print(f"   Se divider ÷2 → Batteria = {voltage_2x:.2f}V (OK per LiPo 2S)")
            print(f"   Se divider ÷4 → Batteria = {voltage_4x:.2f}V (troppo alto!)")
            print()
            print("   👉 Probabilmente hai voltage divider ÷2 (resistenze 10k+10k)")
            
        elif 700 <= avg_raw < 900:
            print("🟠 Valore alto (~3.5-4.5V)")
            voltage_2x = (avg_raw * 5.0 / 1023.0) * 2.0
            print(f"   Se divider ÷2 → Batteria = {voltage_2x:.2f}V (alta!)")
            print("   ⚠️  Vicino al limite 5V dell'ADC!")
            
        elif avg_raw >= 900:
            print("🔴 Valore molto alto (>4.4V)")
            print("   ⚠️  ATTENZIONE: Se batteria diretta >5V → RISCHIO DANNO ADC!")
            print("   → Scollega batteria e verifica circuito")
        
        print()
        
        if variance > 50:
            print("⚡ Varianza alta → tensione instabile o rumore")
        else:
            print("✅ Varianza bassa → lettura stabile")
        
        print()
        print("=" * 70)
        print("TEST 2: CONFRONTO CON COMANDO 'battery'")
        print("=" * 70)
        print()
        
        # Test comando battery usando RoverApi
        battery_v = rover.getBatteryVoltage()
        
        print(f"📊 Comando 'battery' restituisce: {battery_v:.2f}V")
        print()
        
        if battery_v < 1.0:
            print("❌ Tensione troppo bassa!")
            print("   → Voltage divider NON funzionante o batteria assente")
        elif 6.0 <= battery_v <= 8.4:
            print("✅ Tensione normale per batteria LiPo 2S (6-8.4V)")
            print(f"   Stato: {'CARICA' if battery_v > 7.4 else 'MEDIA' if battery_v > 7.0 else 'BASSA'}")
        elif battery_v > 8.4:
            print("⚠️  Tensione alta (>8.4V)")
            print("   → Batteria 3S? O voltage divider sbagliato?")
        else:
            print("🟡 Tensione fuori range tipico (1-6V)")
        
        print()
        print("=" * 70)
        print("🏁 TEST COMPLETATO")
        print("=" * 70)
        print()
        print("📋 PROSSIMI PASSI:")
        print()
        
        if avg_raw > 300:
            print("✅ 1. Il pin A0 è collegato a qualcosa")
            print("✅ 2. Usa il firmware con comando 'battery'")
            print("✅ 3. La compensazione tensione può funzionare")
        else:
            print("❌ 1. Verifica cablaggio batteria → pin A0")
            print("❌ 2. Controlla voltage divider (se presente)")
            print("⚠️  3. DISABILITA compensazione tensione fino a fix hardware")
        
        # Chiudi connessione
        rover.close()
        
    except FileNotFoundError:
        print(f"❌ Porta {port} non trovata!")
        print("   Su Raspberry Pi prova: /dev/ttyUSB0 o /dev/ttyACM0")
        print("   Su Windows prova: COM3, COM4, COM5...")
    except Exception as e:
        print(f"❌ Errore: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    # Rileva sistema operativo
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        import platform
        if platform.system() == "Windows":
            port = "COM3"  # Modifica se necessario
        else:
            port = "/dev/ttyUSB0"
    
    print(f"🔌 Porta seriale: {port}")
    print("   (puoi cambiarla con: python test_a0_diagnostics.py <porta>)")
    print()
    
    try:
        test_a0_diagnostics(port)
    except KeyboardInterrupt:
        print("\n\n⏸️  Test interrotto dall'utente")
