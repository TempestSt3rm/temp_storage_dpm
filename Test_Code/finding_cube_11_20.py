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
DPS = 10
inverted = False
US_Motor.reset_encoder()

Angle_Dist_Dict = defaultdict()

print("end setup")


def save_US_dist_angle():
    angle = US_Motor.get_encoder()
    bottom_dist = US_bottom.get_cm()
    top_dist = US_top.get_cm()
    if abs(top_dist-bottom_dist) > 8:
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


def find_cube_angle_from_dict(angle_dist_dict: dict):
    sorted_angles = sorted(angle_dist_dict.items())
    # .items will give a list of tuples (angle, distance)
    intitial_angle = sorted_angles[0][0]
    range_of_ranges = []
    cube_range = [sorted_angles[0]]
    for i in range(1,len(sorted_angles)):
        print(abs((intitial_angle) - (sorted_angles[i][0])))
        if abs((intitial_angle) - (sorted_angles[i][0])) < 8:
            cube_range.append(sorted_angles[i])
        else:
            range_of_ranges.append(cube_range)
            cube_range = [sorted_angles[i]]
        intitial_angle = sorted_angles[i][0]
    range_of_ranges.append(cube_range)
    cube_range = [sorted_angles[i]]
            

    cube_distance_filtered = []
    print(range_of_ranges)
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
        





try:
    wait_ready_sensors(True)
    while (US_Motor.get_encoder() != 0):
        US_Motor.set_position(0)
    
    position = US_Motor.get_encoder()
    US_Motor.set_dps(DPS)

    #Move to the left side 
    
    sweep_left(45)
    sweep_right_measure(-45)
    find_cube_angle_from_dict(Angle_Dist_Dict)

    sweep_to_zero()


          
    sleep(1) 
except:
    print("returning")
    sweep_to_zero()
finally:
    US_Motor.reset_encoder()
    BP.reset_all()



