import brickpi3
from utils.brick import TouchSensor,EV3UltrasonicSensor,wait_ready_sensors,EV3GyroSensor, Motor, reset_brick
from time import sleep


print("start")
BP = brickpi3.BrickPi3()
Motor1 = Motor("A")
Motor2 = Motor("D")
POWER = 30
Motor1.set_limits(70, 30)
Motor2.set_limits(70, 30)
print("end setup")

        
def Rotate (time):
    Motor1.set_power((-1)*POWER)
    Motor2.set_power((-1)*POWER)
    sleep(time)
try:
    wait_ready_sensors(True)
    print("Ready")
    Rotate(2)
    Motor1.set_power(0)
    Motor2.set_power(0)
            
except :
    print("error")
    BP.reset_all()
    Motor1.set_power(0)
    Motor2.set_power(0)


