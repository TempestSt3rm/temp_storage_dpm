from collections import defaultdict
import brickpi3
from utils.brick import TouchSensor,EV3UltrasonicSensor,wait_ready_sensors,EV3GyroSensor, Motor, reset_brick
from time import sleep

print("start")
BP = brickpi3.BrickPi3()
US_Motor = Motor("B")
US_bottom = EV3UltrasonicSensor(1)
#US_top = EV3UltrasonicSensor(2)



try:
    wait_ready_sensors(True)
    #dist = US_top.get_cm()
    dist = US_bottom.get_cm()
    print(dist)
    BP.reset_all()
    
except:
    
    print("a")