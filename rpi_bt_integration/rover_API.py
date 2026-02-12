import serial
import time


class RoverApi:
    """API per controllare il robot Makeblock Ultimate via seriale."""

    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        """
        Inizializza la connessione seriale con Arduino.
        
        Args:
            port: Porta seriale (es. '/dev/ttyUSB0' su Linux, 'COM3' su Windows)
            baudrate: Velocità comunicazione (default 115200)
        """
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.commands = ["Forward", "Back", "Left", "Right"]
        time.sleep(2)  # Attesa iniziale per apertura seriale
        print(f"✅ Connesso ad Arduino su {port}")

    def read(self):
        """Legge una riga dalla seriale."""
        return self.ser.readline().decode().strip()

    def moveTo(self, cmd, speed):
        """
        Invia comando di movimento al robot.
        
        Args:
            cmd: Comando ("Forward", "Back", "Left", "Right")
            speed: Velocità normalizzata (0.0 - 1.0)
        """
        # FIX: Logica corretta per validazione
        if cmd not in self.commands:
            print(f"⚠️  Comando non valido: {cmd}. Deve essere uno tra {self.commands}")
            return
            
        if speed > 1.0:
            print("⚠️  Velocità deve essere <= 1.0")
            speed = 1.0
        elif speed < 0:
            print("⚠️  Velocità deve essere >= 0")
            speed = 0
            
        speed_percent = int(speed * 100)
        to_send = f"{cmd}:{speed_percent}\n"
        self.ser.write(to_send.encode())
        time.sleep(0.05)  # Piccolo delay per stabilità

    def getUltrasonicSensor(self):
        """
        Legge distanza dal sensore ultrasuoni.
        
        Returns:
            Distanza in cm (400.0 se nessun ostacolo rilevato)
        """
        self.ser.write("ultrasonic\n".encode())
        time.sleep(0.1)
        data = self.ser.readline().decode().strip()
        try:
            return float(data)
        except ValueError:
            return 400.0  # Nessun ostacolo

    def getBatteryVoltage(self):
        """
        Legge tensione della batteria.
        
        Returns:
            float: Tensione in Volt (es. 7.4V per batteria LiPo 2S)
        """
        self.ser.write("battery\n".encode())
        time.sleep(0.1)
        data = self.ser.readline().decode().strip()
        try:
            return float(data)
        except ValueError:
            return 0.0  # Errore lettura

    def getIMU(self):
        """
        Legge dati dall'IMU (accelerometro e giroscopio).
        
        Returns:
            dict: Dizionario con 'accel' (ax, ay, az) e 'gyro' (gx, gy, gz)
                  Ritorna None se lettura fallisce
        """
        self.ser.write("imu\n".encode())
        time.sleep(0.1)
        
        try:
            # Leggi 2 righe: Acc e Gyro
            acc_line = self.ser.readline().decode().strip()
            gyro_line = self.ser.readline().decode().strip()
            
            # Parse "Acc: x, y, z"
            if "Acc:" in acc_line:
                acc_data = acc_line.split("Acc:")[1].strip().split(",")
                ax, ay, az = [int(x.strip()) for x in acc_data]
            else:
                return None
                
            # Parse "Gyro: x, y, z"
            if "Gyro:" in gyro_line:
                gyro_data = gyro_line.split("Gyro:")[1].strip().split(",")
                gx, gy, gz = [int(x.strip()) for x in gyro_data]
            else:
                return None
                
            return {
                'accel': {'x': ax, 'y': ay, 'z': az},
                'gyro': {'x': gx, 'y': gy, 'z': gz}
            }
        except (ValueError, IndexError) as e:
            print(f"⚠️  Errore lettura IMU: {e}")
            return None

    def getShutter(self):
        """
        Legge stato del sensore ME Shutter (bumper/collisione).
        
        Returns:
            int: 0 (LOW - ostacolo rilevato) o 1 (HIGH - libero)
                 Ritorna None se lettura fallisce
        """
        self.ser.write("shutter\n".encode())
        time.sleep(0.1)
        data = self.ser.readline().decode().strip()
        
        try:
            # Parse "Shutter state: 0" o "Shutter state: 1"
            if "Shutter state:" in data:
                state = int(data.split(":")[1].strip())
                return state
            return None
        except (ValueError, IndexError) as e:
            print(f"⚠️  Errore lettura Shutter: {e}")
            return None

    def stop(self):
        """Ferma tutti i motori."""
        self.ser.write("Stop:0\n".encode())
        time.sleep(0.05)

    def armUP(self):
        """Solleva il braccio."""
        self.ser.write("armUP\n".encode())
        time.sleep(0.05)

    def armDown(self):
        """Abbassa il braccio."""
        self.ser.write("armDown\n".encode())
        time.sleep(0.05)

    def openHand(self, duration_ms):
        """
        Apre la pinza.
        
        Args:
            duration_ms: Durata apertura in millisecondi
        """
        to_send = f"openHand:{duration_ms}\n"
        self.ser.write(to_send.encode())
        time.sleep(duration_ms / 1000.0)

    def closeHand(self, duration_ms):
        """
        Chiude la pinza.
        
        Args:
            duration_ms: Durata chiusura in millisecondi
        """
        to_send = f"closeHand:{duration_ms}\n"
        self.ser.write(to_send.encode())
        time.sleep(duration_ms / 1000.0)

    def close(self):
        """Chiude la connessione seriale."""
        self.stop()
        self.ser.close()
        print("🔌 Connessione seriale chiusa")
