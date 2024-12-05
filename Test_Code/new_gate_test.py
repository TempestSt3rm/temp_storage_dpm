import brickpi3
from utils.brick import TouchSensor,EV3UltrasonicSensor,wait_ready_sensors,EV3GyroSensor, Motor, reset_brick
from time import sleep

BP = brickpi3.BrickPi3()
Motor1 = Motor("C")

def rotateup():
    angle = Motor1.get_position()
    Motor1.set_dps(-15)
    while angle >= -90:
        angle = Motor1.get_position()
    
    Motor1.set_dps(-1)
    sleep(1)

def rotatedown():
    angle = Motor1.get_position()
    Motor1.set_dps(15)
    while angle <= -2:
        angle = Motor1.get_position()
    
    Motor1.set_dps(0)
    sleep(4)
    
try: 
    wait_ready_sensors(True)
    Motor1.reset_encoder()
    print("Ready")
    sleep(1)
    rotateup()
    sleep(1)
    rotatedown()
    BP.reset_all()
except KeyboardInterrupt:
    BP.reset_all()