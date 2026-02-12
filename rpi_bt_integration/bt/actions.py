from rover_API import RoverApi
import time

# Inizializza il rover con compensazione tensione attiva
rover = RoverApi(
    port='/dev/ttyUSB0',
    enable_voltage_compensation=True,  # Compensazione automatica per calo batteria
    reference_voltage=7.4,              # Tensione di riferimento (batteria carica)
    min_voltage=6.4                     # Tensione minima per avviso
)
print(f"Arduino connesso su /dev/ttyUSB0")

def move_forward(duration=1.0):
    print("Sto andando avanti...")
    rover.moveTo('Forward', duration)
    time.sleep(duration)
    rover.stop()
    time.sleep(0.2)

def move_back(duration=1.0):
    print("Sto andando indietro...")
    rover.moveTo('Back', duration)
    time.sleep(duration)
    rover.stop()
    time.sleep(0.2)

def move_left(duration=1.0):
    print("Sto girando a sinistra...")
    rover.moveTo('Left', duration)
    time.sleep(duration)
    rover.stop()
    time.sleep(0.2)

def move_right(duration=1.0):
    print("Sto girando a destra...")
    rover.moveTo('Right', duration)
    time.sleep(duration)
    rover.stop()
    time.sleep(0.2)

def arm_up():
    print("Braccio su")
    rover.armUP()
    time.sleep(0.5)

def arm_down():
    print("Braccio giù")
    rover.armDown()
    time.sleep(0.5)

def open_hand(pwm=1500):
    print("Apro la mano")
    rover.openHand(pwm)
    time.sleep(0.5)

def close_hand(pwm=1750):
    print("Chiudo la mano")
    rover.closeHand(pwm)
    time.sleep(0.5)
