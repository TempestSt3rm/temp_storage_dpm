import brickpi3
from utils.brick import TouchSensor,EV3UltrasonicSensor,wait_ready_sensors,EV3GyroSensor, Motor, reset_brick
from time import sleep


print("start")
BP = brickpi3.BrickPi3()
Motor1 = Motor("A")
Motor2 = Motor("D")
POWER = -30
Motor1.set_limits(70, POWER)
Motor2.set_limits(70, POWER)
print("end setup")

def MoveDistFwd(time):
    Motor1.set_power(POWER)
    Motor2.set_power(POWER)
    sleep(time)
        
def Rotate (time):
    Motor1.set_power(POWER)
    Motor2.set_power(POWER * (-1))
    sleep(time)
try:
    wait_ready_sensors(True)
    #Motor1.offset_encoder(Motor1.get_encoder())
        #Motor2.offset_encoder(Motor2.get_encoder())
    print("Ready")
    MoveDistFwd(2)
    Rotate(2)
    Motor1.set_power(0)
    Motor2.set_power(0)
            
except :
    print("error")
    BP.reset_all()
    Motor1.set_power(0)
    Motor2.set_power(0)


