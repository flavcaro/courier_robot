import serial
import time


class RoverApi:
    def __init__(self, port):
        self.ser = serial.Serial(port, 115200, timeout=1)
        self.commands = ["Forward", "Back", "Left", "Right", "Stop"]
        time.sleep(2)

    def read(self):
        return self.ser.readline().decode(errors="ignore").strip()

    def moveTo(self, cmd, speed, speed_right=None):
        """
        cmd: Forward/Back/Left/Right
        speed: 0.0..1.0 (velocità lato sinistro o entrambi se speed_right=None)
        speed_right: 0.0..1.0 (velocità lato destro, opzionale per compensazione)
        """
        if cmd not in self.commands:
            print(f"❌ il comando deve essere uno tra {self.commands}")
            return

        if not isinstance(speed, (int, float)):
            print("❌ speed deve essere un numero (0..1)")
            return

        # clamp velocità
        if speed < 0.0:
            speed = 0.0
        if speed > 1.0:
            speed = 1.0

        # Se speed_right specificato, usa comando compensato
        if speed_right is not None and cmd in ['Forward', 'Back']:
            if speed_right < 0.0:
                speed_right = 0.0
            if speed_right > 1.0:
                speed_right = 1.0
            
            pwm_left = int(speed * 100)
            pwm_right = int(speed_right * 100)
            to_send = f"{cmd}Comp:{pwm_left}:{pwm_right}\n"
        else:
            # Comando normale
            pwm = int(speed * 100)
            to_send = f"{cmd}:{pwm}\n"
        
        self.ser.write(to_send.encode())
        # NON dormire qui: la durata la gestisce chi chiama

    def stop(self):
        self.ser.write("Stop:0\n".encode())

    def rotate_differential(self, direction, speed):
        """
        Rotazione differenziale: un cingolo avanti, altro indietro.
        RICHIEDE FIRMWARE CON COMANDI RotateRight/RotateLeft!
        
        Args:
            direction: 'right' o 'left'
            speed: 0.0-1.0
        """
        if speed < 0.0:
            speed = 0.0
        if speed > 1.0:
            speed = 1.0
        
        pwm = int(speed * 100)
        
        if direction == 'right':
            to_send = f"RotateRight:{pwm}\n"
        elif direction == 'left':
            to_send = f"RotateLeft:{pwm}\n"
        else:
            print("⚠️  Direzione deve essere 'right' o 'left'")
            return
        
        self.ser.write(to_send.encode())

    def rotate_differential_compensated(self, direction, speed_left, speed_right):
        """
        Rotazione differenziale COMPENSATA per differenze di potenza tra i cingoli.
        RICHIEDE FIRMWARE CON COMANDI RotateRightComp/RotateLeftComp!
        
        Args:
            direction: 'right' o 'left'
            speed_left: 0.0-1.5 (velocità cingolo sinistro, può superare 1.0 per compensare debolezza)
            speed_right: 0.0-1.5 (velocità cingolo destro)
        """
        # Clamp a 0.0 minimo
        if speed_left < 0.0:
            speed_left = 0.0
        if speed_right < 0.0:
            speed_right = 0.0
        
        # Clamp a 1.5 massimo (150% potenza per compensazione estrema)
        if speed_left > 1.5:
            speed_left = 1.5
        if speed_right > 1.5:
            speed_right = 1.5
        
        pwm_left = int(speed_left * 100)
        pwm_right = int(speed_right * 100)
        
        if direction == 'right':
            to_send = f"RotateRightComp:{pwm_left}:{pwm_right}\n"
        elif direction == 'left':
            to_send = f"RotateLeftComp:{pwm_left}:{pwm_right}\n"
        else:
            print("⚠️  Direzione deve essere 'right' o 'left'")
            return
        
        self.ser.write(to_send.encode())

    def getUltrasonicSensor(self):
        self.ser.write("ultrasonic\n".encode())
        time.sleep(0.1)
        data = self.read()
        try:
            return float(data)
        except ValueError:
            return 400.00

    def armUP(self):
        self.ser.write("armUP\n".encode())

    def armDown(self):
        self.ser.write("armDown\n".encode())

    def openHand(self, t):
        self.ser.write(f"openHand:{t}\n".encode())

    def closeHand(self, t):
        self.ser.write(f"closeHand:{t}\n".encode())

    def getBatteryVoltage(self):
        self.ser.write("getBattery\n".encode())
        time.sleep(0.1)
        data = self.read()
        try:
            return float(data)
        except ValueError:
            print(f"Errore lettura tensione batteria: {data}")
            return 7.4

    def testMovementWithLidar(self, speed=0.5, obstacle_threshold=40):
        """
        Test: muove il robot in avanti monitorando il lidar.
        Si ferma automaticamente se rileva un ostacolo.
        
        speed: velocità movimento (0.0-1.0), default 0.5 = 50%
        obstacle_threshold: distanza minima in cm, default 40cm
        """
        print("\n" + "="*60)
        print("🤖 TEST MOVIMENTO CON MONITORAGGIO LIDAR")
        print("="*60)
        print(f"⚙️  Velocità: {speed*100:.0f}%")
        print(f"🚧 Soglia ostacolo: {obstacle_threshold}cm")
        print(f"🔋 Batteria: {self.getBatteryVoltage():.2f}V")
        print("\n🚀 Avvio movimento... (Premi Ctrl+C per fermare)")
        print("-"*60)
        
        try:
            # Avvia movimento
            self.moveTo('Forward', speed)
            sample = 0
            
            while True:
                # Leggi sensore
                distance = self.getUltrasonicSensor()
                sample += 1
                
                # Formatta output con barra visiva
                if distance >= 100:
                    status = "🟢 LIBERO"
                    bar = "█" * min(20, int(distance / 20))
                elif distance >= obstacle_threshold:
                    status = "🟡 VICINO"
                    bar = "█" * int(distance / 5)
                else:
                    status = "🔴 OSTACOLO!"
                    bar = "█" * max(1, int(distance / 5))
                
                print(f"[{sample:3d}] {distance:6.1f}cm  {bar:20s} {status}", end='\r')
                
                # Ferma se ostacolo
                if distance < obstacle_threshold:
                    print(f"\n\n⚠️  OSTACOLO RILEVATO A {distance:.1f}cm!")
                    self.stop()
                    print("🛑 Robot fermato")
                    break
                
                time.sleep(0.15)  # ~6-7 letture al secondo
                
        except KeyboardInterrupt:
            print("\n\n⏸️  Test interrotto dall'utente")
            self.stop()
        
        print("\n" + "="*60)
        print(f"✅ Test completato - {sample} campioni letti")
        print(f"🔋 Batteria finale: {self.getBatteryVoltage():.2f}V")
        print("="*60)


# Test rapido - esegui con: python3 rover_API.py
if __name__ == "__main__":
    print("\n🔌 Connessione a /dev/ttyUSB0...")
    rover = RoverApi(port='/dev/ttyUSB0')
    print("✅ Connesso!\n")
    
    # Test lidar + movimento
    rover.testMovementWithLidar(speed=0.70, obstacle_threshold=35)
    
    print("\n👋 Chiusura connessione...")
    rover.ser.close()
    print("✅ Fatto!")
