#!/usr/bin/env python3
"""
Test rotazioni precise con IMU MPU6050
Confronta rotazioni basate su tempo vs IMU feedback
"""
 
from bt.imu_sensor import IMUSensor
from bt.simple_state import simple_state
from bt.actions import rover, DEFAULT_SPEED_TURN
import time
 
 
def test_rotation_comparison():
    """Confronta rotazione tempo vs IMU"""
    print("=" * 70)
    print("  TEST ROTAZIONI: TEMPO vs IMU")
    print("=" * 70)
    
    # Inizializza IMU
    imu = IMUSensor()
    
    if not imu.is_available():
        print("❌ IMU non disponibile!")
        print("   Verifica: sudo i2cdetect -y 1")
        print("   Installa: pip install mpu6050-raspberrypi smbus2")
        return
    
    # Calibrazione IMU
    print("\n📍 FASE 1: CALIBRAZIONE IMU")
    print("-" * 70)
    input("⚠️  Robot FERMO su superficie piana. Premere INVIO...")
    
    imu.calibrate(samples=300)
    
    # Test drift
    print("\n📍 FASE 2: TEST DRIFT (10 secondi)")
    print("-" * 70)
    print("   Robot deve rimanere FERMO\n")
    
    imu.reset_heading(0.0)
    start = time.time()
    
    try:
        while time.time() - start < 10:
            imu.update_heading()
            h = imu.get_heading()
            gz = imu.get_gyro_z()
            print(f"   {time.time()-start:4.1f}s | Heading: {h:6.2f}° | Gyro Z: {gz:7.3f}°/s", end='\r')
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    
    drift = imu.get_heading()
    print(f"\n   Drift totale: {drift:.2f}°")
    
    if abs(drift) > 5:
        print(f"   ⚠️  ATTENZIONE: Drift elevato! Ripetere calibrazione o controllare superficie")
    else:
        print(f"   ✅ Drift accettabile")
    
    # Test rotazione a tempo
    print("\n📍 FASE 3: ROTAZIONE DESTRA A TEMPO")
    print("-" * 70)
    print(f"   Tempo calibrato: {simple_state.rotation_90_time:.2f}s")
    input("   Premere INVIO per test...")
    
    imu.reset_heading(0.0)
    
    # Calcola velocità
    lf, rf = simple_state.get_rotation_factors('R')
    speed_left = DEFAULT_SPEED_TURN * lf
    speed_right = DEFAULT_SPEED_TURN * rf
    
    print(f"   Inizio rotazione... (speed_L={speed_left:.2f}, speed_R={speed_right:.2f})")
    
    rover.rotate_differential_compensated('right', speed_left, speed_right)
    
    # Monitora durante rotazione
    start = time.time()
    while time.time() - start < simple_state.rotation_90_time:
        imu.update_heading()
        h = imu.get_heading()
        print(f"   {time.time()-start:4.2f}s | Heading: {h:6.2f}°", end='\r')
        time.sleep(0.02)
    
    rover.stop()
    time.sleep(0.5)
    
    # Leggi heading finale
    for _ in range(10):
        imu.update_heading()
        time.sleep(0.02)
    
    final_heading = imu.get_heading()
    # Per destra, heading diminuisce (o va verso 360)
    actual_rotation = 360 - final_heading if final_heading > 180 else abs(final_heading)
    
    print(f"\n   ✅ Rotazione completata:")
    print(f"      Heading finale: {final_heading:.2f}°")
    print(f"      Angolo ruotato: {actual_rotation:.2f}°")
    print(f"      Errore da 90°: {abs(actual_rotation - 90):.2f}°")
    
    time.sleep(2)
    
    # Test rotazione con IMU feedback
    print("\n📍 FASE 4: ROTAZIONE DESTRA CON IMU FEEDBACK")
    print("-" * 70)
    input("   Premere INVIO per test...")
    
    imu.reset_heading(0.0)
    target = 90.0
    timeout = 5.0
    
    print(f"   Target: {target}° (tolleranza ±2°)")
    
    rover.rotate_differential_compensated('right', speed_left, speed_right)
    
    start = time.time()
    reached = False
    
    try:
        while time.time() - start < timeout:
            imu.update_heading()
            current = imu.get_heading()
            
            # Per destra
            angle = 360 - current if current > 180 else abs(current)
            
            print(f"   {time.time()-start:4.2f}s | Heading: {current:6.2f}° | Ruotato: {angle:6.2f}°", end='\r')
            
            if abs(angle - target) <= 2.0:
                reached = True
                break
            
            time.sleep(0.02)
    finally:
        rover.stop()
        time.sleep(0.5)
    
    # Leggi finale
    for _ in range(10):
        imu.update_heading()
        time.sleep(0.02)
    
    final = imu.get_heading()
    actual = 360 - final if final > 180 else abs(final)
    
    print(f"\n   ✅ Rotazione IMU completata in {time.time()-start:.2f}s:")
    print(f"      Heading finale: {final:.2f}°")
    print(f"      Angolo ruotato: {actual:.2f}°")
    print(f"      Errore da 90°: {abs(actual - 90):.2f}°")
    print(f"      Status: {'✅ SUCCESSO' if reached else '⚠️ TIMEOUT'}")
    
    # Test 360° con IMU
    print("\n📍 FASE 5: TEST 360° CON IMU")
    print("-" * 70)
    print("   4 rotazioni di 90° con IMU - deve tornare a 0°")
    input("   Premere INVIO per test...")
    
    imu.reset_heading(0.0)
    
    for i in range(4):
        print(f"\n   Rotazione {i+1}/4...")
        
        rover.rotate_differential_compensated('right', speed_left, speed_right)
        
        start = time.time()
        angle_start = imu.get_heading()
        
        while time.time() - start < timeout:
            imu.update_heading()
            current = imu.get_heading()
            
            # Calcola angolo dalla posizione di partenza di questa rotazione
            delta = (current - angle_start) % 360
            if delta > 180:
                delta = delta - 360
            angle_from_start = abs(delta)
            
            if angle_from_start >= 88:  # Tolleranza per fermarsi vicino a 90
                break
            
            time.sleep(0.02)
        
        rover.stop()
        time.sleep(0.5)
        
        # Leggi heading
        for _ in range(5):
            imu.update_heading()
            time.sleep(0.02)
        
        print(f"   Heading dopo rot {i+1}: {imu.get_heading():.2f}°")
        time.sleep(1)
    
    final_360 = imu.get_heading()
    print(f"\n   ✅ Test 360° completato:")
    print(f"      Heading finale: {final_360:.2f}°")
    print(f"      Errore da 0°: {abs(final_360):.2f}° (o {abs(360-final_360):.2f}°)")
    
    error = min(abs(final_360), abs(360 - final_360))
    if error <= 5:
        print(f"      ✅ ECCELLENTE - Errore {error:.2f}°")
    elif error <= 10:
        print(f"      ✓ BUONO - Errore {error:.2f}°")
    else:
        print(f"      ⚠️  RIVEDERE - Errore {error:.2f}°")
    
    print("\n" + "=" * 70)
    print("  Test completato!")
    print("=" * 70)
    
    rover.ser.close()
 
 
if __name__ == "__main__":
    test_rotation_comparison()
 
 