import brickpi3
from utils.brick import TouchSensor,EV3UltrasonicSensor,wait_ready_sensors,EV3GyroSensor, Motor, reset_brick
from time import sleep

print("start")
BP = brickpi3.BrickPi3()
Motor1 = Motor("B")
US_bottom = EV3UltrasonicSensor(1)
US_top = EV3UltrasonicSensor(2)

#US = EV3UltrasonicSensor(2)
POWER = 40
#Motor1.set_(70, 45)
ANGLE = 45
DPS = 10
inverted = False
Motor1.reset_encoder()
print("end setup")

#data_file = "/home/pi/Desktop/Team10Data/Radar_Rotation_Test.csv"

try:
    wait_ready_sensors(True)
    #BP.reset_all()
    #BP.set_motor_limits(BP.PORT_B, POWER)
    #BP.set_motor_limits(BP.PORT_D, POWER)
    #Motor1.set_position(0)
    while (Motor1.get_encoder() != 0):
        Motor1.set_position(0)
    
    position = 0
    print(DPS)
    Motor1.set_dps(DPS)
    
    while True:
        position = Motor1.get_encoder()
        print(US_bottom.get_cm())
        while ((((position) > 45)) and not(inverted)):
            DPS = -DPS
            #print(DPS)
            Motor1.set_dps(DPS)
            position = Motor1.get_encoder()
            print("turning right" + str(position))
            inverted = True
        
        while (((position) < -45) and inverted):
            DPS = -DPS
            #print(DPS)
            Motor1.set_dps(DPS)
            position = Motor1.get_encoder()
            print("turning left" + str(position))
            inverted = False
            
          
        sleep(1) 
except:
    Motor1.reset_encoder()
    BP.reset_all()