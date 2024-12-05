from collections import defaultdict
import brickpi3
from utils.brick import TouchSensor,EV3UltrasonicSensor,wait_ready_sensors,EV3GyroSensor, Motor, reset_brick
from time import sleep

print("start")
BP = brickpi3.BrickPi3()
US_Motor = Motor("B")
US_bottom = EV3UltrasonicSensor(1)
US_top = EV3UltrasonicSensor(2)

POWER = 30
ANGLE = 45
DPS = 5
inverted = False
dist_threshold = 25
US_Motor.reset_encoder()

Angle_Dist_Dict = defaultdict(list)

print("end setup")

def take_median_reading(Sensor):
    readings = []
    for i in range(10):
        value = Sensor.get_cm()
        readings.append(value)
    readings = sorted(readings)
    median = (readings[5] + readings[6])/2
    readings.clear()
    return median

def save_US_dist_angle():
    angle = US_Motor.get_encoder()
    #US_bottom.set_mode("cm")
    US_top.set_mode("listen")
    sleep(0.2)
    #bottom_dist = take_median_reading(US_bottom)
    bottom_dist = 0
    US_top.set_mode("cm")
    US_bottom.set_mode("listen")
    sleep(0.2)
    top_dist = take_median_reading(US_top)
    if (top_dist-bottom_dist) > dist_threshold:
        Angle_Dist_Dict[angle] = [bottom_dist,top_dist]
        print("Save angle into dictionary   CUBE DETECTED")
    print("Detected Angle:" + str(angle) + "\n" + "Bottom Dist:" + str(bottom_dist) + "\n" + "Top Dist:" + str(top_dist) + "\n")

try:
    wait_ready_sensors(True)
    while (US_Motor.get_encoder() != 0):
        US_Motor.set_position(0)
    
    position = 0
    print(DPS)
    US_Motor.set_dps(DPS)
    
    while True:
        position = US_Motor.get_encoder()
        save_US_dist_angle()
        while ((((position) > 65)) and not(inverted)):
            DPS = -DPS
            #print(DPS)
            US_Motor.set_dps(DPS)
            position = US_Motor.get_encoder()
            print("turning right" + str(position))
            inverted = True
        
        while (((position) < -65) and inverted):
            DPS = -DPS
            #print(DPS)
            US_Motor.set_dps(DPS)
            position = US_Motor.get_encoder()
            print("turning left" + str(position))
            inverted = False
            
          
        sleep(1) 
except:
    print("returning")

    while US_Motor.get_encoder() > 2 or US_Motor.get_encoder() < -2:
        position = US_Motor.get_encoder()
        print("returning loop")
        mini_dps = 5
        while (position < -2):
                mini_dps = 5
                #print(DPS)
                US_Motor.set_dps(mini_dps)
                position = US_Motor.get_encoder()
                #print("turning right" + str(position))
        
        while ((position) > 2):
            mini_dps = -5
            #print(DPS)
            US_Motor.set_dps(mini_dps)
            position = US_Motor.get_encoder()
            #print("turning left" + str(position))
            inverted = False
finally:
    US_Motor.reset_encoder()
    US_Motor.set_power(0)
    BP.reset_all()




