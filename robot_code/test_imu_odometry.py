#!/usr/bin/env python3
"""
Test odometria IMU - Confronta distanza tempo vs accelerometro
"""
 
from bt.imu_sensor import IMUSensor
from bt.simple_state import simple_state
from bt.actions import rover, DEFAULT_SPEED_LINEAR
import time
import math
 
 
def test_odometry_comparison():
    """Confronta odometria tempo vs IMU"""
    print("=" * 70)
    print("  TEST ODOMETRIA: TEMPO vs IMU")
    print("=" * 70)
    
    # Inizializza IMU
    imu = IMUSensor()
    
    if not imu.is_available():
        print("❌ IMU non disponibile!")
        return
    
    # Calibrazione
    print("\n📍 CALIBRAZIONE IMU")
    print("-" * 70)
    input("⚠️  Robot FERMO su superficie piana. Premere INVIO...")
    
    imu.calibrate(samples=300)
    
    # Test 1: Movimento breve (0.5m)
    print("\n📍 TEST 1: MOVIMENTO 0.5m")
    print("-" * 70)
    input("   Premere INVIO per test...")
    
    target_distance = 0.5
    test_movement(imu, target_distance, simple_state.meters_per_second_forward)
    
    time.sleep(2)
    
    # Test 2: Movimento medio (1.0m)
    print("\n📍 TEST 2: MOVIMENTO 1.0m")
    print("-" * 70)
    input("   Premere INVIO per test...")
    
    target_distance = 1.0
    test_movement(imu, target_distance, simple_state.meters_per_second_forward)
    
    time.sleep(2)
    
    # Test 3: Movimento su superficie irregolare
    print("\n📍 TEST 3: MOVIMENTO SU SUPERFICIE IRREGOLARE")
    print("-" * 70)
    print("   Posizionare il robot su una superficie irregolare")
    print("   (es. tappeto, pavimento con irregolarità)")
    input("   Premere INVIO per test 0.5m...")
    
    target_distance = 0.5
    test_movement(imu, target_distance, simple_state.meters_per_second_forward)
    
    print("\n" + "=" * 70)
    print("  Test completato!")
    print("=" * 70)
    print("\n💡 CONCLUSIONI:")
    print("   - Se IMU e tempo concordano (±10%): superficie regolare, buona calibrazione")
    print("   - Se IMU < tempo: robot slitta o superficie irregolare")
    print("   - Se IMU > tempo: calibrazione velocità da rivedere")
    print("   - Se IMU molto impreciso: accelerometro rumoroso, calibrare meglio")
    
    rover.ser.close()
 
 
def test_movement(imu, target_meters, speed_mps):
    """
    Test singolo movimento con confronto tempo vs IMU
    """
    # Calcola tempo stimato
    time_needed = target_meters / speed_mps
    
    print(f"\n   Target: {target_meters:.2f}m")
    print(f"   Velocità calibrata: {speed_mps:.2f} m/s")
    print(f"   Tempo stimato: {time_needed:.2f}s")
    
    # Reset odometria IMU
    distance_imu = 0.0
    velocity_x = 0.0
    velocity_y = 0.0
    alpha_vel = 0.8  # Filtro velocità
    
    # Velocità motori
    speed_left = DEFAULT_SPEED_LINEAR * simple_state.left_factor
    speed_right = DEFAULT_SPEED_LINEAR * simple_state.right_factor
    
    print(f"\n   Inizio movimento...")
    
    # Inizia movimento
    rover.moveTo('Forward', speed_left, speed_right)
    
    start_time = time.time()
    last_time = start_time
    
    samples = []  # Salva campioni per analisi
    
    try:
        while time.time() - start_time < time_needed:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            
            if dt < 0.01:
                time.sleep(0.01)
                continue
            
            # Leggi accelerazioni
            ax, ay = imu.get_accel_xy()
            
            # Filtra e integra velocità
            velocity_x = alpha_vel * velocity_x + (1 - alpha_vel) * (ax * 9.81 * dt)
            velocity_y = alpha_vel * velocity_y + (1 - alpha_vel) * (ay * 9.81 * dt)
            
            velocity = math.sqrt(velocity_x**2 + velocity_y**2)
            distance_imu += velocity * dt
            
            # Salva campione
            samples.append({
                'time': current_time - start_time,
                'ax': ax,
                'ay': ay,
                'vel': velocity,
                'dist': distance_imu
            })
            
            print(f"   {current_time-start_time:4.2f}s | IMU: {distance_imu:.3f}m | "
                  f"Vel: {velocity:.2f}m/s | Acc: ({ax:.3f}, {ay:.3f})g", end='\r')
            
            time.sleep(0.02)
    
    finally:
        rover.stop()
        time.sleep(0.5)
    
    # Leggi ancora per stabilizzare
    for _ in range(10):
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time
        
        ax, ay = imu.get_accel_xy()
        velocity_x = alpha_vel * velocity_x + (1 - alpha_vel) * (ax * 9.81 * dt)
        velocity_y = alpha_vel * velocity_y + (1 - alpha_vel) * (ay * 9.81 * dt)
        velocity = math.sqrt(velocity_x**2 + velocity_y**2)
        distance_imu += velocity * dt
        
        time.sleep(0.02)
    
    actual_time = time.time() - start_time
    distance_time = actual_time * speed_mps
    
    print(f"\n\n   ✅ RISULTATI:")
    print(f"      Tempo reale: {actual_time:.2f}s")
    print(f"      Distanza (tempo): {distance_time:.3f}m")
    print(f"      Distanza (IMU):   {distance_imu:.3f}m")
    
    error_percent = abs(distance_imu - distance_time) / distance_time * 100
    print(f"      Differenza: {abs(distance_imu - distance_time):.3f}m ({error_percent:.1f}%)")
    
    if error_percent < 10:
        print(f"      ✅ ECCELLENTE - Errore < 10%")
    elif error_percent < 20:
        print(f"      ✓ BUONO - Errore < 20%")
    elif error_percent < 30:
        print(f"      ⚠️  ACCETTABILE - Errore < 30%")
    else:
        print(f"      ❌ ALTO - Rivedere calibrazione")
    
    # Statistiche accelerazione
    if samples:
        avg_ax = sum(s['ax'] for s in samples) / len(samples)
        avg_ay = sum(s['ay'] for s in samples) / len(samples)
        max_ax = max(abs(s['ax']) for s in samples)
        max_ay = max(abs(s['ay']) for s in samples)
        
        print(f"\n   📊 Statistiche accelerazione:")
        print(f"      Media: ({avg_ax:.3f}, {avg_ay:.3f})g")
        print(f"      Max:   ({max_ax:.3f}, {max_ay:.3f})g")
 
 
if __name__ == "__main__":
    test_odometry_comparison()
 
 