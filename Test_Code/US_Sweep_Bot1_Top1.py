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
DPS = 15
inverted = False
dist_threshold = 10
US_Motor.reset_encoder()

Angle_Dist_Dict = defaultdict(list)
Top_readings,Bot_readings = [],[]

print("end setup")

def take_median_reading(Sensor):
    readings = []
    for i in range(20):
        value = Sensor.get_cm()
        readings.append(value)
    median = (readings[9] + readings[10])/2
    readings.clear()
    return median

def save_US_dist_angle_to_arr(arr,motor_top_or_bot:str):
    if motor_top_or_bot == "top":
        dist = US_top.get_cm()
    else:
        dist = US_bottom.get_cm()
    angle = US_Motor.get_encoder()
    arr.append((angle,dist))

    print("Save angle into: " + motor_top_or_bot + "array")
    print("Detected Angle:" + str(angle) + "\n" + "Detected Distance:" + str(dist) + "\n")

def differentiate_top_bottom_values(arr_top,arr_bot):
    arr_bot = arr_bot[::-1]
    print("Top array:", arr_top)
    print("Bottom array:", arr_bot)
    for i in range(0,min(len(arr_top),len(arr_bot))):
        angle = (arr_top[i][0] + arr_bot[i][0]) // 2
        if (arr_top[i][1] - arr_bot[i][1]) > dist_threshold:
            Angle_Dist_Dict[angle] = arr_bot[i][1]
            #print("CUBE at angle: " + str(angle) + "distnace " + str(arr_bot[i][1]) )
    return Angle_Dist_Dict

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
        


def find_cube_angle_from_dict(angle_dist_dict: dict):
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
        tupple = (sorted_angles[len(sorted_angles) // 2], sorted_distance[len(sorted_distance) // 2])
        cube_distance_filtered.append(tupple)
    #print(cube_distance_filtered)
    return cube_distance_filtered

try:
    wait_ready_sensors(True)
    
    while (US_Motor.get_encoder() != 0):
        US_Motor.set_position(0)
    
    position = 0
    sweep_left(ANGLE)
    sleep(1)
    position = US_Motor.get_encoder()
    US_top.set_mode("cm")
    US_bottom.set_mode("listen")
    US_Motor.set_dps(-DPS)
 
    while US_Motor.get_encoder() > -ANGLE:
        save_US_dist_angle_to_arr(Top_readings,"top")
        
    sleep(1)
    US_top.set_mode("listen")
    US_bottom.set_mode("cm")
    US_Motor.set_dps(DPS)
    
    while US_Motor.get_encoder() < ANGLE:
       save_US_dist_angle_to_arr(Bot_readings,"bot")
    sweep_to_zero()
    print("final")
    print(find_cube_angle_from_dict(differentiate_top_bottom_values(Top_readings,Bot_readings)))
          
    sleep(1) 
except:
    print("returning")

    sweep_to_zero()
finally:
    US_Motor.reset_encoder()
    US_Motor.set_power(0)
    BP.reset_all()
