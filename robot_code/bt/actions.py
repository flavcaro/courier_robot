from rover_API import RoverApi
import time

rover = RoverApi(port='/dev/ttyUSB0')
print("Arduino connesso su /dev/ttyUSB0")

# ===== PROFILO ECO - RISPARMIO BATTERIA =====
DEFAULT_SPEED_LINEAR = 0.60  # Lineare: 60% (ridotto per massimo risparmio)
DEFAULT_SPEED_TURN = 0.75    # Rotazioni: 75% (per superficie liscia)
STOP_PAUSE = 0.05

# COMPENSAZIONE DERIVA: motore sinistro leggermente più lento
# Valore ottimale: 0.97 (3% più lento) per compensare deriva verso sinistra
LEFT_MOTOR_COMPENSATION = 0.97
RIGHT_MOTOR_COMPENSATION = 1.00

def move_forward(duration=1.0, speed=DEFAULT_SPEED_LINEAR):
    # Applica compensazione deriva
    speed_left = speed * LEFT_MOTOR_COMPENSATION
    speed_right = speed * RIGHT_MOTOR_COMPENSATION
    rover.moveTo('Forward', speed_left, speed_right)
    time.sleep(duration)
    rover.stop()
    time.sleep(STOP_PAUSE)


def move_back(duration=1.0, speed=DEFAULT_SPEED_LINEAR):
    # Applica compensazione deriva anche in retro
    speed_left = speed * LEFT_MOTOR_COMPENSATION
    speed_right = speed * RIGHT_MOTOR_COMPENSATION
    rover.moveTo('Back', speed_left, speed_right)
    time.sleep(duration)
    rover.stop()
    time.sleep(STOP_PAUSE)


def move_left(duration=1.0, speed=DEFAULT_SPEED_TURN):
    rover.moveTo('Left', speed)
    time.sleep(duration)
    rover.stop()
    time.sleep(STOP_PAUSE)


def move_right(duration=1.0, speed=DEFAULT_SPEED_TURN):
    rover.moveTo('Right', speed)
    time.sleep(duration)
    rover.stop()
    time.sleep(STOP_PAUSE)


def arm_up():
    rover.armUP()
    time.sleep(0.5)


def arm_down():
    rover.armDown()
    time.sleep(0.5)


def open_hand(pwm=1500):
    rover.openHand(pwm)
    time.sleep(0.5)


def close_hand(pwm=1750):
    rover.closeHand(pwm)
    time.sleep(0.5)
