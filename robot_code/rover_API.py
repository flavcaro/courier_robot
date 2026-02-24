#!/usr/bin/env python3
import os
import glob
import serial
import time


def find_rover_port(preferred=None):
    """
    Trova automaticamente la porta seriale del rover.
    Ordine:
      1) preferred (se passato)
      2) env var ROVER_PORT
      3) /dev/ttyACM* poi /dev/ttyUSB* (più comune per Arduino)
    Ritorna stringa tipo '/dev/ttyACM0' oppure None.
    """
    candidates = []

    if preferred:
        candidates.append(preferred)

    env = os.environ.get("ROVER_PORT", "").strip()
    if env:
        candidates.append(env)

    # comuni
    candidates += ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2",
                   "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2"]

    # match reali
    candidates += sorted(glob.glob("/dev/ttyACM*"))
    candidates += sorted(glob.glob("/dev/ttyUSB*"))

    # de-dup preservando ordine
    seen = set()
    uniq = []
    for c in candidates:
        if c and c not in seen:
            seen.add(c)
            uniq.append(c)

    # tieni solo quelli che esistono
    uniq = [c for c in uniq if os.path.exists(c)]
    return uniq[0] if uniq else None


class RoverApi:
    def __init__(self, port=None, baud=115200, timeout=1, connect=True):
        """
        port: '/dev/ttyACM0' o '/dev/ttyUSB0' ecc. Se None -> autodetect
        connect: se False, non apre la seriale (utile per test)
        """
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser = None

        # comandi supportati (base)
        self.commands = ["Forward", "Back", "Left", "Right", "Stop"]

        if connect:
            self.connect()

    def connect(self):
        """Apre la seriale. Se port None, fa autodetect."""
        if self.ser and self.ser.is_open:
            return True

        if self.port is None:
            self.port = find_rover_port()

        if not self.port:
            raise FileNotFoundError(
                "Nessuna porta rover trovata. "
                "Collega l'arduino e controlla /dev/ttyACM* o /dev/ttyUSB*. "
                "Oppure esporta ROVER_PORT=/dev/ttyACM0"
            )

        # apre seriale
        self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
        time.sleep(2)  # attesa reset Arduino
        return True

    def is_connected(self):
        return self.ser is not None and self.ser.is_open

    def close(self):
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass

    def read(self):
        if not self.is_connected():
            return ""
        return self.ser.readline().decode(errors="ignore").strip()

    def _write(self, s: str):
        if not self.is_connected():
            raise RuntimeError("RoverApi non connesso: seriale non aperta")
        self.ser.write(s.encode())

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

        # clamp
        speed = max(0.0, min(1.0, float(speed)))

        # comando compensato solo per Forward/Back
        if speed_right is not None and cmd in ["Forward", "Back"]:
            speed_right = max(0.0, min(1.0, float(speed_right)))
            pwm_left = int(speed * 100)
            pwm_right = int(speed_right * 100)
            to_send = f"{cmd}Comp:{pwm_left}:{pwm_right}\n"
        else:
            pwm = int(speed * 100)
            to_send = f"{cmd}:{pwm}\n"

        self._write(to_send)

    def stop(self):
        # compatibile col firmware: Stop:0
        try:
            self._write("Stop:0\n")
        except Exception:
            pass

    def rotate_differential(self, direction, speed):
        """
        Rotazione differenziale: un cingolo avanti, altro indietro.
        Richiede comandi firmware RotateRight/RotateLeft.
        """
        speed = max(0.0, min(1.0, float(speed)))
        pwm = int(speed * 100)

        if direction == "right":
            to_send = f"RotateRight:{pwm}\n"
        elif direction == "left":
            to_send = f"RotateLeft:{pwm}\n"
        else:
            print("⚠️  Direzione deve essere 'right' o 'left'")
            return

        self._write(to_send)

    def rotate_differential_compensated(self, direction, speed_left, speed_right):
        """
        Rotazione differenziale compensata.
        speed_left/speed_right: 0.0..1.5 (max 150%)
        """
        speed_left = max(0.0, min(1.5, float(speed_left)))
        speed_right = max(0.0, min(1.5, float(speed_right)))

        pwm_left = int(speed_left * 100)
        pwm_right = int(speed_right * 100)

        if direction == "right":
            to_send = f"RotateRightComp:{pwm_left}:{pwm_right}\n"
        elif direction == "left":
            to_send = f"RotateLeftComp:{pwm_left}:{pwm_right}\n"
        else:
            print("⚠️  Direzione deve essere 'right' o 'left'")
            return

        self._write(to_send)

    def getUltrasonicSensor(self):
        self._write("ultrasonic\n")
        time.sleep(0.1)
        data = self.read()
        try:
            return float(data)
        except ValueError:
            return 400.0

    def armUP(self):
        self._write("armUP\n")

    def armDown(self):
        self._write("armDown\n")

    def openHand(self, t):
        self._write(f"openHand:{int(t)}\n")

    def closeHand(self, t):
        self._write(f"closeHand:{int(t)}\n")

    def beep(self):
        """Emette 3 beep di errore"""
        self._write("beep\n")
        time.sleep(1.0)  # Attende fine beep (3 beep x 300ms)

    def testMovementWithLidar(self, speed=0.5, obstacle_threshold=40):
        print("\n" + "=" * 60)
        print("🤖 TEST MOVIMENTO CON MONITORAGGIO LIDAR")
        print("=" * 60)
        print(f"⚙️  Velocità: {speed*100:.0f}%")
        print(f"🚧 Soglia ostacolo: {obstacle_threshold}cm")
        print("\n🚀 Avvio movimento... (Premi Ctrl+C per fermare)")
        print("-" * 60)

        try:
            self.moveTo("Forward", speed)
            sample = 0
            while True:
                distance = self.getUltrasonicSensor()
                sample += 1

                if distance >= 100:
                    status = "🟢 LIBERO"
                    bar = "█" * min(20, int(distance / 20))
                elif distance >= obstacle_threshold:
                    status = "🟡 VICINO"
                    bar = "█" * int(distance / 5)
                else:
                    status = "🔴 OSTACOLO!"
                    bar = "█" * max(1, int(distance / 5))

                print(f"[{sample:3d}] {distance:6.1f}cm  {bar:20s} {status}", end="\r")

                if distance < obstacle_threshold:
                    print(f"\n\n⚠️  OSTACOLO RILEVATO A {distance:.1f}cm!")
                    self.stop()
                    print("🛑 Robot fermato")
                    break

                time.sleep(0.15)

        except KeyboardInterrupt:
            print("\n\n⏸️  Test interrotto dall'utente")
            self.stop()

        print("\n" + "=" * 60)
        print(f"✅ Test completato - {sample} campioni letti")
        print("=" * 60)


if __name__ == "__main__":
    print("\n🔌 Connessione rover (autodetect)...")
    try:
        rover = RoverApi(port=None)
        print(f"✅ Connesso su {rover.port}!\n")
        rover.testMovementWithLidar(speed=0.70, obstacle_threshold=35)
    finally:
        try:
            rover.close()
        except Exception:
            pass
        print("\n👋 Chiusura connessione... ✅")
