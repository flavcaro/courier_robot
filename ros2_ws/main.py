from rover_API import RoverApi
import time 

rover = RoverApi('/dev/ttyUSB0')


#comandi basilari di movimento 

rover.moveTo('Forward',0.5)
time.sleep(1)
rover.stop()
time.sleep(1)
rover.moveTo('Back',0.5)
time.sleep(1)
rover.stop()
time.sleep(1)
rover.moveTo('Left',0.5)
time.sleep(1)
rover.stop()
time.sleep(1)
rover.moveTo('Right',0.5)
time.sleep(1.25)
rover.stop() 


time.sleep(1)
rover.armUP()
time.sleep(0.5)
rover.openHand(1500)
rover.armDown()
time.sleep(5)
rover.closeHand(1750)
time.sleep(2)
rover.armUP() 
time.sleep(1)

while True :
    print(rover.getUltrasonicSensor())

