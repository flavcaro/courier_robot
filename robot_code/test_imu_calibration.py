#!/usr/bin/env python3
"""
Script per calibrare l'IMU MPU6050 e testare le letture.
"""
 
from bt.imu_sensor import IMUSensor
import time
 
 
def main():
    print("=" * 50)
    print("  CALIBRAZIONE E TEST IMU MPU6050")
    print("=" * 50)
    print()
    
    # Inizializza IMU
    imu = IMUSensor()
    
    if not imu.is_available():
        print("❌ IMU non disponibile!")
        print("\n🔍 Verifica:")
        print("   1. Connessione I2C: sudo i2cdetect -y 1")
        print("   2. Libreria: pip install mpu6050-raspberrypi")
        print("   3. I2C abilitato: sudo raspi-config > Interface > I2C")
        return
    
    # FASE 1: Calibrazione
    print("📍 FASE 1: CALIBRAZIONE")
    print("-" * 50)
    input("⚠️  Posizionare il robot FERMO su superficie PIANA.\n   Premere INVIO per iniziare...")
    
    imu.calibrate(samples=300)
    
    # FASE 2: Test heading statico
    print("\n📍 FASE 2: TEST DRIFT HEADING (30 sec)")
    print("-" * 50)
    print("   Il robot deve rimanere FERMO.")
    print("   Osservare quanto varia l'heading (drift del giroscopio).\n")
    
    imu.reset_heading(0.0)
    
    start_time = time.time()
    try:
        while time.time() - start_time < 30:
            imu.update_heading()
            heading = imu.get_heading()
            gyro_z = imu.get_gyro_z()
            
            elapsed = time.time() - start_time
            print(f"⏱ {elapsed:5.1f}s | Heading: {heading:7.2f}° | Gyro Z: {gyro_z:7.3f}°/s", end='\r')
            
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    
    print("\n\n✅ Test drift completato")
    final_heading = imu.get_heading()
    print(f"   Drift totale: {final_heading:.2f}° in 30 secondi")
    
    # FASE 3: Test rotazione manuale
    print("\n📍 FASE 3: TEST ROTAZIONE MANUALE")
    print("-" * 50)
    print("   Ruotare LENTAMENTE il robot di 90° in senso orario.")
    print("   Premere Ctrl+C quando completato.\n")
    
    imu.reset_heading(0.0)
    
    try:
        while True:
            imu.update_heading()
            heading = imu.get_heading()
            gyro_z = imu.get_gyro_z()
            temp = imu.get_temperature()
            
            print(f"Heading: {heading:7.2f}° | Gyro Z: {gyro_z:7.3f}°/s | Temp: {temp:5.1f}°C", end='\r')
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    
    print("\n\n✅ Test completato")
    final_heading = imu.get_heading()
    print(f"   Heading finale: {final_heading:.2f}°")
    print(f"   Errore da 90°: {abs(final_heading - 90):.2f}°")
    
    # FASE 4: Monitoraggio continuo
    print("\n📍 FASE 4: MONITORAGGIO CONTINUO")
    print("-" * 50)
    print("   Premere Ctrl+C per terminare.\n")
    
    imu.reset_heading(0.0)
    
    try:
        while True:
            imu.update_heading()
            data = imu.get_all_data()
            
            if data:
                print(f"Heading: {data['heading']:7.2f}° | "
                      f"Gyro [X:{data['gyro']['x']:6.2f} Y:{data['gyro']['y']:6.2f} Z:{data['gyro']['z']:6.2f}]°/s | "
                      f"Accel [X:{data['accel']['x']:6.3f} Y:{data['accel']['y']:6.3f} Z:{data['accel']['z']:6.3f}]g | "
                      f"Temp: {data['temp']:5.1f}°C", end='\r')
            
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n\n✅ Monitoraggio terminato")
    
    print("\n" + "=" * 50)
    print("  Test completato con successo!")
    print("=" * 50)
 
 
if __name__ == "__main__":
    main()
 
 