import serial
import time


class RoverApi:
    def __init__(self, port):
        self.ser = serial.Serial(port, 9600, timeout=1)
        self.commands = ["Forward", "Back", "Left", "Right", "Stop"]
        time.sleep(2)

    def read(self):
        return self.ser.readline().decode(errors="ignore").strip()

    def moveTo(self, cmd, speed):
        """
        cmd: Forward/Back/Left/Right
        speed: 0.0..1.0
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

        pwm = int(speed * 100)
        to_send = f"{cmd}:{pwm}\n"
        self.ser.write(to_send.encode())
        # NON dormire qui: la durata la gestisce chi chiama

    def stop(self):
        self.ser.write("Stop:0\n".encode())

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
