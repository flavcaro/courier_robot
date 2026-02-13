import serial
import time


class RoverApi:

    def __init__(self, port):
        self.ser = serial.Serial(port, 9600, timeout=1)
        self.commands = ["Forward", "Back", "Left", "Right"]
        time.sleep(2)
        # attesa iniziale per apertura seriale

    def read(self):
        return self.ser.readline().decode().strip()

    def moveTo(self, cmd, speed):
        if speed <= 1 or cmd in self.commands:
            speed *= 100
            to_send = f"{cmd}:{speed}\n"
            self.ser.write(to_send.encode())
            print(f"Comando inviato {cmd}:{speed}")
            time.sleep(0.2)
        elif speed >= 1:
            print("la velocita deve essere minore di 1")
        elif cmd not in self.commands:
            print(f"il comando deve essere uno tra {self.commands}")

    def getUltrasonicSensor(self):
        self.ser.write("ultrasonic".encode())
        time.sleep(0.5)
        data = self.ser.readline().decode().strip()
        if data != "Comando non riconosciuto: sonic":
            return float(data)
        return 400.00

    def stop(self):
        self.ser.write("Stop:0\n".encode())

    def armUP(self):
        to_send = f"armUP\n"
        self.ser.write(to_send.encode())

    def armDown(self):
        to_send = f"armDown\n"
        self.ser.write(to_send.encode())

    def openHand(self, time):
        to_send = f"openHand:{time}\n"
        self.ser.write(to_send.encode())

    def closeHand(self, time):
        to_send = f"closeHand:{time}\n"
        self.ser.write(to_send.encode())

    def getBatteryVoltage(self):
        """
        Legge la tensione della batteria in Volt.
        
        Returns:
            float: Tensione batteria in Volt (es. 7.4V per batteria carica)
        """
        self.ser.write("getBattery\n".encode())
        time.sleep(0.1)  # Attesa per lettura ADC
        data = self.ser.readline().decode().strip()
        try:
            return float(data)
        except ValueError:
            print(f"Errore lettura tensione batteria: {data}")
            return 7.4  # Valore di fallback (batteria carica)

