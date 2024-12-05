from collections import defaultdict
import brickpi3
from utils.brick import TouchSensor,EV3UltrasonicSensor,wait_ready_sensors,EV3GyroSensor, Motor, reset_brick, EV3ColorSensor
from time import sleep

#brjfjnafjk

Motor1 = Motor("A")
Motor2 = Motor("D")
POWER = 30 #DONT CHANGE
Motor1.set_limits(70, POWER)
Motor2.set_limits(70, POWER)


US_Motor = Motor("B")
US_Motor.set_limits(70,POWER)
US = EV3UltrasonicSensor(2)

POWER = 30
ANGLE = 60
DPS = 5
US_Motor.reset_encoder()

BP = brickpi3.BrickPi3()

BP = brickpi3.BrickPi3()
US_Motor = Motor("B")
US_bottom = EV3UltrasonicSensor(1)
US_top = EV3UltrasonicSensor(2)

inverted = False
US_Motor.reset_encoder()

Angle_Dist_Dict = defaultdict()
gate_motor = Motor('C')



print("end setup")

def MoveDistFwd(distance):
    #Power 30 2 seconds is about 16 cm
    Motor1.set_power((-1)*POWER)
    Motor2.set_power((-1)*POWER)
    sleep(distance/8)
    Motor1.set_power(0)
    Motor2.set_power(0)
        
def Rotate_Robot_Right(angle):
    #2 seconds at a power of 30 is approximately 90 degrees
    #1 second at a power of 30 is about 45 degrees
    # Motor 1-A +1  Motor 2-D -1 Left Turn
    # Motor 1-A -1  Motor 2-D +1 Right Turn
    Motor1.set_power(POWER * (-1))
    Motor2.set_power(POWER * (1))
    sleep(round((angle/45),3)) 
    Motor1.set_power(0)
    Motor2.set_power(0)

def Rotate_Robot_Left(angle):
    #2 seconds at a power of 30 is approximately 90 degrees
    #1 second at a power of 30 is about 35 degrees
    # Motor 1-A +1  Motor 2-D -1 Left Turn
    # Motor 1-A -1  Motor 2-D +1 Right Turn
    Motor1.set_power(POWER)
    Motor2.set_power(POWER * (-1))
    sleep(round((angle/45),3))
    Motor1.set_power(0)
    Motor2.set_power(0)

print("end setup")



def save_US_dist_angle():
    angle = US_Motor.get_encoder()
    sleep(0.1)
    bottom_dist = US_bottom.get_cm()
    sleep(0.1)
    top_dist = US_top.get_cm()
    if (top_dist-bottom_dist) > 10:
        Angle_Dist_Dict[angle] = bottom_dist
        print("Save angle into dictionary   CUBE DETECTED")
    print("Detected Angle:" + str(angle) + "\n" + "Bottom Dist:" + str(bottom_dist) + "\n" + "Top Dist:" + str(top_dist) + "\n")


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
    if US_Motor.get_encoder() > 2:
        sweep_right(2)
    else:
        sweep_left(-2)

def sweep_right_measure(angle_right):
    
    US_Motor.set_dps(-DPS)
    while US_Motor.get_encoder() > angle_right:
        save_US_dist_angle()
        sleep(0.2)
    US_Motor.set_dps(0)


def find_cube_angle_from_dict(angle_dist_dict):
    sorted_angles = sorted(angle_dist_dict.items())
    # .items will give a list of tuples (angle, distance)
    intitial_angle = sorted_angles[0][0]
    range_of_ranges = []
    cube_range = [sorted_angles[0]]
    for i in range(1,len(sorted_angles)):
        print(abs((intitial_angle) - (sorted_angles[i][0])))
        if abs((intitial_angle) - (sorted_angles[i][0])) < 5:
            cube_range.append(sorted_angles[i])
        else:
            range_of_ranges.append(cube_range)
            cube_range = [sorted_angles[i]]
        intitial_angle = sorted_angles[i][0]
    range_of_ranges.append(cube_range)
    cube_range = [sorted_angles[i]]
            

    cube_distance_filtered = []
    for r in range_of_ranges:
        distances = []
        angles = []
        for angle,dist in r:
            angles.append(angle)
            distances.append(dist)

        sorted_distance = sorted(distances)
        sorted_angles = sorted(angles)
        tupple = ( sorted_angles[len(sorted_angles) // 2],sorted_distance[len(sorted_distance) // 2])
        cube_distance_filtered.append(tupple)
    print(cube_distance_filtered)
    return cube_distance_filtered
        

def rotateup():
    gate_motor.reset_encoder()
    angle = Motor1.get_position()
    gate_motor.set_dps(-15)
    while angle >= -90:
        angle = Motor1.get_position()
    
    gate_motor.set_dps(-1)
    sleep(1)

def rotatedown():
    angle = gate_motor.get_position()
    gate_motor.set_dps(15)
    while angle <= -2:
        angle = Motor1.get_position()
    
    Motor1.set_dps(0)
    sleep(4)

try:
    wait_ready_sensors(True)
    
    while (US_Motor.get_encoder() != 0):
        US_Motor.set_position(0)
    
    position = US_Motor.get_encoder()
    US_Motor.set_dps(DPS)

    #Move to the left side 
    
    sweep_left(ANGLE)
    sweep_right_measure(-ANGLE)
    arr_medians = find_cube_angle_from_dict(Angle_Dist_Dict) #list of tuples where each tuple is the median (angle,distance) of the cube
    
    sweep_to_zero()

    distance_arr = []
    for tupple in arr_medians:
        distance_arr.append(tupple[1])

    distance_arr = sorted(distance_arr)
    target_distance = distance_arr[0]
    print("target" + str(target_distance))

    target_angle = 0
    for tupple in arr_medians:
        if tupple[1] == target_distance:
            target_angle = tupple[0]

    if target_angle > 0:
        Rotate_Robot_Left(target_angle)
    else:
        Rotate_Robot_Right(abs(target_angle))

    MoveDistFwd(round(target_distance/2))
    
    rotateup()
    MoveDistFws(1)
    rotatedown()


    sleep(1) 
except:
    print("returning")
    sweep_to_zero()
finally:
    US_Motor.reset_encoder()
    BP.reset_all()

