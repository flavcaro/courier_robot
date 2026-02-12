import serial
import time


class RoverApi:
    """API per controllare il robot Makeblock Ultimate via seriale."""

    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, 
                 enable_voltage_compensation=True, 
                 reference_voltage=7.4, 
                 min_voltage=6.4):
        """
        Inizializza la connessione seriale con Arduino.
        
        Args:
            port: Porta seriale (es. '/dev/ttyUSB0' su Linux, 'COM3' su Windows)
            baudrate: Velocità comunicazione (default 115200)
            enable_voltage_compensation: Abilita compensazione automatica tensione (default True)
            reference_voltage: Tensione di riferimento per calibrazione (default 7.4V per LiPo 2S)
            min_voltage: Tensione minima sicura (default 6.4V)
        """
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.commands = ["Forward", "Back", "Left", "Right"]
        
        # Configurazione compensazione tensione
        self.enable_voltage_compensation = enable_voltage_compensation
        self.reference_voltage = reference_voltage
        self.min_voltage = min_voltage
        self.current_voltage = reference_voltage  # Inizialmente assume batteria carica
        self.last_voltage_check = 0
        self.voltage_check_interval = 10  # Controlla tensione ogni 10 secondi
        
        time.sleep(2)  # Attesa iniziale per apertura seriale
        print(f"✅ Connesso ad Arduino su {port}")
        
        # Leggi tensione iniziale
        if self.enable_voltage_compensation:
            self._update_voltage()
            print(f"🔋 Tensione batteria: {self.current_voltage:.2f}V (compensazione attiva)")

    def read(self):
        """Legge una riga dalla seriale."""
        return self.ser.readline().decode().strip()

    def _update_voltage(self):
        """Aggiorna la lettura della tensione batteria (uso interno)."""
        voltage = self.getBatteryVoltage()
        if voltage > 0:  # Lettura valida
            self.current_voltage = voltage
            self.last_voltage_check = time.time()
            
            # Avvisa se batteria bassa
            if voltage < self.min_voltage:
                print(f"⚠️  BATTERIA BASSA: {voltage:.2f}V - Ricaricare!")
            elif voltage < self.min_voltage + 0.3:
                print(f"⚠️  Batteria in esaurimento: {voltage:.2f}V")

    def _get_voltage_compensation_factor(self):
        """
        Calcola il fattore di compensazione basato sulla tensione.
        
        Returns:
            float: Fattore moltiplicativo per compensare il calo di tensione
                   (es. 1.0 = nessuna compensazione, 1.15 = +15%)
        """
        # Aggiorna tensione se è passato l'intervallo
        if time.time() - self.last_voltage_check > self.voltage_check_interval:
            self._update_voltage()
        
        if not self.enable_voltage_compensation or self.current_voltage <= 0:
            return 1.0
        
        # Calcola compensazione: speed_compensated = speed * (Vref / Vcurrent)
        # Limita la compensazione a max 1.3x (30% aumento)
        compensation = min(self.reference_voltage / self.current_voltage, 1.3)
        
        return compensation

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
        
        # Applica compensazione tensione
        compensation_factor = self._get_voltage_compensation_factor()
        compensated_speed = speed * compensation_factor
        
        # Limita a 1.0 massimo
        if compensated_speed > 1.0:
            compensated_speed = 1.0
        
        # Log compensazione se significativa
        if compensation_factor > 1.05:
            print(f"🔋 Compensazione batteria: {speed:.2f} → {compensated_speed:.2f} "
                  f"({self.current_voltage:.2f}V)")
            
        speed_percent = int(compensated_speed * 100)
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

    def getBatteryStatus(self):
        """
        Ottiene stato dettagliato della batteria.
        
        Returns:
            dict: Dizionario con voltage, percentage, status, compensation_factor
        """
        voltage = self.getBatteryVoltage()
        
        # Calcola percentuale (approssimativa per LiPo 2S: 8.4V=100%, 6.4V=0%)
        voltage_range = self.reference_voltage + 1.0 - self.min_voltage  # es. 8.4 - 6.4 = 2.0V
        percentage = max(0, min(100, ((voltage - self.min_voltage) / voltage_range) * 100))
        
        # Determina stato
        if voltage < self.min_voltage:
            status = "CRITICAL"
        elif voltage < self.min_voltage + 0.3:
            status = "LOW"
        elif voltage < self.reference_voltage - 0.5:
            status = "GOOD"
        else:
            status = "FULL"
        
        # Calcola compensazione attuale
        self.current_voltage = voltage
        compensation = self._get_voltage_compensation_factor()
        
        return {
            'voltage': voltage,
            'percentage': round(percentage, 1),
            'status': status,
            'compensation_factor': round(compensation, 3),
            'compensation_enabled': self.enable_voltage_compensation
        }

    def setVoltageCompensation(self, enabled):
        """
        Abilita/disabilita la compensazione tensione.
        
        Args:
            enabled: True per abilitare, False per disabilitare
        """
        self.enable_voltage_compensation = enabled
        print(f"🔋 Compensazione tensione: {'ATTIVA' if enabled else 'DISATTIVA'}")

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
