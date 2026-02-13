from rover_API import RoverApi
import time

rover = RoverApi(port='/dev/ttyUSB0')
print("Arduino connesso su /dev/ttyUSB0")

# ===== PROFILO ECO =====
DEFAULT_SPEED_LINEAR = 0.40  # avanti/indietro
DEFAULT_SPEED_TURN = 0.35    # rotazioni
STOP_PAUSE = 0.05


def move_forward(duration=1.0, speed=DEFAULT_SPEED_LINEAR):
    rover.moveTo('Forward', speed)
    time.sleep(duration)
    rover.stop()
    time.sleep(STOP_PAUSE)


def move_back(duration=1.0, speed=DEFAULT_SPEED_LINEAR):
    rover.moveTo('Back', speed)
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
