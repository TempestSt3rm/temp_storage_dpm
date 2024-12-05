from collections import defaultdict
import brickpi3
from utils.brick import *
from time import sleep
import statistics
import math


print("start")
BP = brickpi3.BrickPi3()
US_Motor = Motor("B")
US_bottom = EV3UltrasonicSensor("1")
US_top = EV3UltrasonicSensor("2")
Motor1 = Motor("A")
Motor2 = Motor("D")
gateMotor = Motor("C")
CSRight = EV3ColorSensor("4")
CSLeft = EV3ColorSensor("3")
US_top = EV3UltrasonicSensor("2")

US_bottom.set_mode("cm")

x_offset = 17
y_offset = 5.5
POWER = 20
ANGLE = 75
DPS = 90
INVERTED = False
dist_threshold = 10
US_Motor.reset_encoder()
Motor1.reset_encoder()
Motor2.reset_encoder()

Angle_Dist_Dict = defaultdict(list)
Dist_readings = []
Ang_readings = []
Wall_readings = []
threshold = 1.2

print("end setup")


def isPoo():
    #first we check for blocks
    sleep(0.1)
    averageR = 0
    for i in range(10):
        c = CSRight.get_rgb()
        averageR += normalizeRed(c)
        sleep(0.01)
    averageR /= 10
    if averageR >= 0.5:
        return True
    else:
        return False
    
def isBlock():
    sleep(0.1)
    average = 0
    for i in range(20):
        c = CSRight.get_rgb()
        average = c[0] + c[1] + c[2] + average
    average /= 20
    if average < 15:
        return False
    else:
        return True
    
def normalizeRed(colorList):
    denominator = (colorList[0] + colorList[1] + colorList[2])
    if denominator == 0:
        return 0
    return colorList[0]/(colorList[0] + colorList[1] + colorList[2])

def moveDistFwd():
    Motor1.set_power((-1)*POWER)
    Motor2.set_power((-1)*POWER)

def moveDistBack():
    Motor1.set_power((1)*POWER)
    Motor2.set_power((1)*POWER)
        
def rotateRight(time):
    Motor1.set_power(POWER * (-1))
    Motor2.set_power(POWER * (1))
    sleep(time)
    Motor1.set_power(0)
    Motor2.set_power(0)
    
def rotateLeft(time):
    Motor1.set_power(POWER * (1))
    Motor2.set_power(POWER * (-1))
    sleep(time)
    Motor1.set_power(0)
    Motor2.set_power(0)
    
def rotateGateUp():
    angle = gateMotor.get_position()
    gateMotor.set_dps(-60)
    while angle >= -80:
        sleep(0.1)
        angle = gateMotor.get_position()
    
    gateMotor.set_dps(0)

def rotateGateDown():
    
    angle = gateMotor.get_position()
    gateMotor.set_dps(60)
    while angle <= -2:
        sleep(0.1)
        angle = gateMotor.get_position()
    gateMotor.set_dps(0)
    gateMotor.set_power(0)
    
def save_US_dist_to_arr(arr):

    distance = US_bottom.get_cm()
    arr.append(distance)
    

def save_US_angle_to_arr(arr):
    angle = US_Motor.get_encoder()
    #print("Angle detected is:" + str(angle) + "\n")
    arr.append(angle)


def save_US_walldist_to_arr(arr):
    distance = US_top.get_cm()
    arr.append(distance)

def sweep_right(angle_right):
    
    US_Motor.set_dps(-DPS)
    while US_Motor.get_encoder() > angle_right:
        sleep(0.2)
    US_Motor.set_dps(0)

def sweep_left(angle_left):
    
    US_Motor.set_dps(DPS)
    while US_Motor.get_encoder() < angle_left:
        sleep(0.2)
    US_Motor.set_dps(0)

def sweep_to_zero():
    #Positive values in degrees are to the left
    #Negative values in degrees are to the right
    if US_Motor.get_encoder() > 2:
        sweep_right(2)
    else:
        sweep_left(-2)

def mean(distances):
    #sum all the distances
    total = sum(distances)
    
    #Calculate the mean by dividing the total by the number of elements
    mean_distance = total / len(distances) if distances else 0
    return mean_distance
    
#def median_US_dist():
#    arr = []
#    for i in range(4):
#        dist = US_bottom.get_cm()     
#        arr.append(dist)
    #number of elements
#    sorted_distances = sorted(arr)
#    arr.clear()
#    n = len(sorted_distances)
    
#    if n%2 == 1:
#        return sorted_distances[n//2]
        
#    else:
 #       mid1, mid2 = sorted_distances[n//2 - 1], sorted_distances[n//2]
 #       return (mid1 + mid2) / 2
  
#def median_US_angle():
#    arr = []
#    for i in range(4):
#        angle = US_Motor.get_encoder()     
#        arr.append(angle)
    #number of elements
#    sorted_angles = sorted(arr)
#    arr.clear()
#    n = len(sorted_angles)

#    if n%2 == 1:
#        return sorted_angles[n//2]
        
 #   else:
 #       mid1, mid2 = sorted_angles[n//2 - 1], sorted_angles[n//2]
#        return (mid1 + mid2) / 2
    
def find_min_distance(distances):
    if not distances:
        return None
    
    return min(distances)
    

def has_significant_different_distance(distances, threshold=1.2):
    
    # Calculate the mean of the distances
    mean_distance = mean(distances)
    
    for distance in distances:
        if (abs(distance - mean_distance) > threshold and distance != 255):
            print("the mean is" + str(mean_distance) + "\n")
            print("The threshold is" + str(threshold) + "\n")
            index = distances.index(distance)
            return Ang_readings[index]
            
    return -500

def sweepAndDetect():
    
    angle1 = -500
    angle2 = -500
    
    Dist_readings.clear()
    Ang_readings.clear()
    Wall_readings.clear()
    US_Motor.reset_encoder()
    US_Motor.set_dps(DPS)
    while US_Motor.get_encoder() < ANGLE:
        
        sleep(0.10)
        save_US_dist_to_arr(Dist_readings)
        save_US_angle_to_arr(Ang_readings)
        save_US_walldist_to_arr(Wall_readings)
    US_Motor.set_dps(0)
    angle = has_significant_different_distance(Dist_readings, threshold)
    if (angle != -500):
        angle1 = angle
    
    Dist_readings.clear()
    Ang_readings.clear()
    Wall_readings.clear()
    US_Motor.reset_encoder()
    US_Motor.set_dps(-DPS)
    while US_Motor.get_encoder() > -ANGLE:
        sleep(0.10)      
        save_US_angle_to_arr(Ang_readings)
        save_US_dist_to_arr(Dist_readings)
        save_US_walldist_to_arr(Wall_readings)
    US_Motor.set_dps(0)
    angle = has_significant_different_distance(Dist_readings, threshold)
    Wall_distance = find_min_distance(Wall_readings)
    if (angle != -500):
        angle2 = angle + ANGLE
        
        US_Motor.set_power(0)
        blockAngle = (angle1 + angle2) / 2
        if blockAngle >= 0:
            return blockAngle, Wall_distance
        

    return -500, Wall_distance

            
def follow_wall():
    US_Motor.reset_encoder()
    US_Motor.set_position_relative(45)
    distance = US_top.get_cm()
    
    while True:
        distList = []
        if distance > 10:
            rotateLeft(0.01)
        elif distance < 7:
            rotateRight(0.01)
        else:
            moveDistFwd()
            sleep(0.5)
            Motor1.set_power(0)
            Motor2.set_power(0)
            
        for i in range(5):
            distance = US_top.get_cm()
            distList.append(distance)
        distance = statistics.median(distList) 
        
    
try:

    wait_ready_sensors(True)
    
    while True:
        #print(US_Motor.get_encoder())
        (blockAngle, Wall_dist) = sweepAndDetect()
        print("The distance between the robot and the wall is:" + str(Wall_dist) + "\n")
        print("The block angle is" + str(blockAngle) + "\n")
        

        
        if blockAngle >= 0:
            blockAngle = math.radians(blockAngle)
            x_f = math.cos(blockAngle)* (13.5)
            y_f = math.sin(blockAngle) * (13.5)
            if (y_f > 5.5):             
                RobotAngle = math.atan((y_f - y_offset)/(x_f + x_offset))
            elif (y_f < 5.5):
                RobotAngle = math.atan((y_offset - y_f)/(x_f + x_offset))
            
            RobotAngle = math.degrees(RobotAngle)
            print("the robot angle is" + str(RobotAngle) + "\n")
            print(str(x_f) + "," + str(y_f) + "\n")
            print("y_f value:" + str(y_f) + "\n")
            if (y_f) > 5.5:
                rotateRight(4/90*(RobotAngle))
                      
            elif(y_f) < 5.5:
                rotateLeft(4/90*(RobotAngle))
                          
            gateMotor.reset_encoder()
            rotateGateUp()
        
            POWER = 15 
    
            moveDistFwd()
    
            while not isBlock():
                continue
    
            POWER = 20
    
            Motor1.set_power(0)
            Motor2.set_power(0)
    
            if isPoo():
                moveDistFwd()
            else:
                moveDistBack()
           
            sleep(1.3)
            Motor1.set_power(0)
            Motor2.set_power(0)
            
            rotateGateDown()
            
  
  
        else:
            moveDistFwd()
            sleep(0.5)
            Motor1.set_power(0)
            Motor2.set_power(0)
    
        
        
        
except KeyboardInterrupt:
    pass
finally:
    Dist_readings.clear()
    Ang_readings.clear()
    Wall_readings.clear()
    US_Motor.reset_encoder()
    US_Motor.set_power(0)
    Motor1.set_power(0)
    Motor2.set_power(0)
    BP.reset_all()


