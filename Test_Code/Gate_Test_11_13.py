import brickpi3
from utils.brick import TouchSensor,EV3UltrasonicSensor,wait_ready_sensors,EV3GyroSensor, Motor, reset_brick
from time import sleep

BP = brickpi3.BrickPi3()
Motor1 = Motor("B")

def rotate (angle):
    
    position = Motor1.get_position()
    if angle < 0:
        Motor1.set_dps(-120)
    else:
        Motor1.set_dps(120)   
    while True:
        value = Motor1.get_position() 
        if abs(value-position) == abs(angle):
            Motor1.set_dps(0)
            break
        
wait_ready_sensors(True)
print("Ready")
sleep(1)
rotate(91)
rotate(-89)
BP.reset_all()
        
