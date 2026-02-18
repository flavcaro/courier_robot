"""
Modulo per gestire l'IMU MPU6050.
Fornisce letture di accelerometro, giroscopio e heading calibrato.
"""
 
# Fix per smbus/smbus2
import sys
try:
    import smbus2 as smbus
    sys.modules['smbus'] = smbus
except ImportError:
    pass
 
try:
    from mpu6050 import mpu6050
except ImportError as e:
    print(f"⚠️ Libreria mpu6050 non trovata: {e}")
    print("   Installare con: pip install mpu6050-raspberrypi smbus2")
    mpu6050 = None
 
import time
import math
 
 
class IMUSensor:
    def __init__(self, address=0x68):
        """
        Inizializza l'IMU MPU6050.
        
        Args:
            address: Indirizzo I2C dell'IMU (default 0x68)
        """
        self.sensor = None
        self.address = address
        
        # Offset per calibrazione (da calcolare con calibrazione statica)
        self.gyro_offset_z = 0.0
        self.accel_offset_x = 0.0
        self.accel_offset_y = 0.0
        
        # Stato heading integrato
        self.heading_deg = 0.0  # Angolo rispetto a Nord (0-360°)
        self.last_time = time.time()
        
        # Filtro complementare
        self.alpha = 0.98  # Peso gyro vs accelerometro
        
        try:
            if mpu6050 is not None:
                self.sensor = mpu6050(address)
                print(f"✅ IMU MPU6050 inizializzato su indirizzo 0x{address:02x}")
            else:
                print("❌ Impossibile inizializzare IMU: libreria mancante")
        except Exception as e:
            print(f"❌ Errore inizializzazione IMU: {e}")
            self.sensor = None
    
    def is_available(self):
        """Ritorna True se l'IMU è disponibile."""
        return self.sensor is not None
    
    def calibrate(self, samples=100):
        """
        Calibra il giroscopio e l'accelerometro in posizione statica.
        
        Args:
            samples: Numero di campioni per calibrazione
        """
        if not self.is_available():
            print("❌ IMU non disponibile per calibrazione")
            return
        
        print(f"🔧 Calibrazione IMU in corso ({samples} campioni)...")
        print("   ⚠️ Mantenere il robot FERMO e in piano!")
        
        gyro_z_sum = 0.0
        accel_x_sum = 0.0
        accel_y_sum = 0.0
        
        for i in range(samples):
            try:
                gyro = self.sensor.get_gyro_data()
                accel = self.sensor.get_accel_data()
                
                gyro_z_sum += gyro['z']
                accel_x_sum += accel['x']
                accel_y_sum += accel['y']
                
                time.sleep(0.01)
            except Exception as e:
                print(f"❌ Errore durante calibrazione: {e}")
                return
        
        # Calcola offset medi
        self.gyro_offset_z = gyro_z_sum / samples
        self.accel_offset_x = accel_x_sum / samples
        self.accel_offset_y = accel_y_sum / samples
        
        print(f"✅ Calibrazione completata:")
        print(f"   Gyro Z offset: {self.gyro_offset_z:.3f} °/s")
        print(f"   Accel X offset: {self.accel_offset_x:.3f} g")
        print(f"   Accel Y offset: {self.accel_offset_y:.3f} g")
    
    def get_gyro_z(self):
        """Ritorna velocità angolare Z (yaw rate) calibrata in °/s."""
        if not self.is_available():
            return 0.0
        
        try:
            gyro = self.sensor.get_gyro_data()
            return gyro['z'] - self.gyro_offset_z
        except Exception as e:
            print(f"❌ Errore lettura giroscopio: {e}")
            return 0.0
    
    def get_accel_xy(self):
        """Ritorna accelerazioni X,Y calibrate in g."""
        if not self.is_available():
            return 0.0, 0.0
        
        try:
            accel = self.sensor.get_accel_data()
            ax = accel['x'] - self.accel_offset_x
            ay = accel['y'] - self.accel_offset_y
            return ax, ay
        except Exception as e:
            print(f"❌ Errore lettura accelerometro: {e}")
            return 0.0, 0.0
    
    def update_heading(self):
        """
        Aggiorna heading integrando il giroscopio.
        Chiamare periodicamente (es. ogni loop).
        """
        if not self.is_available():
            return
        
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Leggi velocità angolare
        gyro_z = self.get_gyro_z()
        
        # Integra per ottenere heading
        self.heading_deg += gyro_z * dt
        
        # Normalizza 0-360°
        self.heading_deg = self.heading_deg % 360.0
    
    def get_heading(self):
        """Ritorna heading corrente in gradi (0-360°)."""
        return self.heading_deg
    
    def reset_heading(self, initial_heading=0.0):
        """
        Resetta heading a un valore iniziale.
        
        Args:
            initial_heading: Angolo iniziale in gradi (default 0 = Nord)
        """
        self.heading_deg = initial_heading
        self.last_time = time.time()
        print(f"🧭 Heading resettato a {initial_heading}°")
    
    def get_temperature(self):
        """Ritorna temperatura in °C."""
        if not self.is_available():
            return None
        
        try:
            return self.sensor.get_temp()
        except Exception as e:
            print(f"❌ Errore lettura temperatura: {e}")
            return None
    
    def get_all_data(self):
        """Ritorna dizionario con tutti i dati IMU."""
        if not self.is_available():
            return None
        
        try:
            accel = self.sensor.get_accel_data()
            gyro = self.sensor.get_gyro_data()
            temp = self.get_temperature()
            
            return {
                'accel': accel,
                'gyro': gyro,
                'temp': temp,
                'heading': self.heading_deg
            }
        except Exception as e:
            print(f"❌ Errore lettura dati IMU: {e}")
            return None
 
 
# Test standalone
if __name__ == "__main__":
    print("=== Test IMU MPU6050 ===\n")
    
    imu = IMUSensor()
    
    if not imu.is_available():
        print("❌ IMU non disponibile. Verifica:")
        print("   1. Connessione I2C (sudo i2cdetect -y 1)")
        print("   2. Libreria installata (pip install mpu6050-raspberrypi)")
        exit(1)
    
    # Calibrazione
    imu.calibrate(samples=200)
    
    # Reset heading
    imu.reset_heading(0.0)
    
    print("\n📊 Lettura dati (Ctrl+C per terminare):\n")
    
    try:
        while True:
            imu.update_heading()
            
            data = imu.get_all_data()
            if data:
                print(f"Heading: {data['heading']:6.1f}° | "
                      f"Gyro Z: {data['gyro']['z']:7.2f}°/s | "
                      f"Accel X: {data['accel']['x']:6.3f}g Y: {data['accel']['y']:6.3f}g | "
                      f"Temp: {data['temp']:5.1f}°C", end='\r')
            
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n\n✅ Test terminato")
 
 