import datetime
import brickpi3
from utils.brick import EV3UltrasonicSensor, wait_ready_sensors, Motor, EV3GyroSensor, EV3ColorSensor
from time import sleep
import threading
import math

class Robot:
    def __init__(self):
        #General
        self.BP = brickpi3.BrickPi3()
        self.power = 20

        #Motor 
        self.motor_left = Motor("A")
        self.motor_right = Motor("D")
        #Gate
        self.gate_motor = Motor("C")
        #Block Sensing
        self.motor_sweep = Motor("B")

        #Readings from the sweep
        self.Ang_Readings = []
        self.Color_Readings = []

        self.sweep_retracted = True 

        #Color sensing
        self.color_left = EV3ColorSensor(3)
        self.color_sweep = EV3ColorSensor(4)
        self.color_sensor_thread = None
        self.color_sensor_active = False

        #Encoder 
        self.encoder_moving_slider = [0,2,5,8,12,19,30,40]
        self.is_stuck = False
        
        self.isWaterRight = False
        self.isWaterLeft = False

        #NAVIGATION
        self.US = EV3UltrasonicSensor(1)
        self.moving_slider = [50,50,50,50,50,50,50,50,50]

        self.US_thread = None
        self.US_sensor_active = False
        self.wall= False

        self.gyro_port = 2
        self.gyro = EV3GyroSensor(2)
        self.movement_queue = []
        #[("f",90) ]
        
        self.initial_angle = 0
        self.final_angle = 0
        self.water_movement = 0

        self.trash_right = False
        self.trash_left = False
        self.going_home = False
        self.block_found = False
        wait_ready_sensors()
        self.start_time = datetime.datetime.now()
        self.setup_motors()

    def close(self):
        print("closing")
        self.Ang_Readings.clear()
        self.motor_left.set_power(0)
        self.motor_right.set_power(0)
        self.gate_motor.set_dps(0)
        self.motor_sweep.set_dps(0)
        self.BP.reset_all()

    def clear_Readings(self):
        self.Ang_Readings.clear()
        self.Color_Readings.clear()
    
    def time_difference(self):
        
        difference = datetime.datetime.now() - self.start_time

        # Get the total difference in seconds
        seconds = difference.total_seconds()
        if seconds > 30:
            return False
        return True

    #SETUP
    def setup_motors(self):
        self.motor_left.set_limits(self.power, 20)
        self.motor_right.set_limits(self.power, 20)
        self.gate_motor.set_limits(self.power, 20)
        
        self.motor_left.reset_encoder()
        self.motor_right.reset_encoder()
        self.motor_sweep.reset_encoder()
        self.gate_motor.reset_encoder()
        
    #COLORS STUFF
    def normalizeRed(self,colorList):
        denominator = (colorList[0] + colorList[1] + colorList[2])
        if denominator == 0:
            return 0
        return colorList[0]/(colorList[0] + colorList[1] + colorList[2])

    def normalizeGreen(self,colorList):
        denominator = (colorList[0] + colorList[1] + colorList[2])
        if denominator == 0:
            return 0
        return colorList[1]/(colorList[0] + colorList[1] + colorList[2])

    def normalizeBlue(self,colorList):
        denominator = (colorList[0] + colorList[1] + colorList[2])
        if denominator == 0:
            return 0
        return colorList[2]/(colorList[0] + colorList[1] + colorList[2])
    
    def identify_block(self,colorList):
        while colorList[0] == None:
            colorList = self.color_sweep.get_rgb()
        red = colorList[0]
        green = colorList[1]
        blue = colorList[2]
        total = red + green + blue
        if red  > 50:
            
            return "poop"
        
        normalizedGreen = self.normalizeGreen(colorList)
        normalizedBlue = self.normalizeBlue(colorList)
        normalizedRed = self.normalizeRed(colorList)
        if normalizedRed > 0.5 and red > 40 and total > 120:
            return "poop"
        if normalizedGreen > 0.5 and green < 50:
            return "ground"
        if normalizedBlue > 0.4:
            return "water"
        if total > 40 and normalizedRed > 0.4 and normalizedGreen > 0.3:
            return "trash"
        if (total > 70):
            return "obstacle"
        
        #print(colorList)
        #print("cant identify")
        return "ground"

    #GATE ROTATION
    def rotateGateUp(self):
        angle = self.gate_motor.get_position()
        self.gate_motor.set_dps(-60)
        while angle >= -80:
            sleep(0.1)
            angle = self.gate_motor.get_position()
        self.gate_motor.set_dps(0)
    
    def rotateGateDown(self):
        angle = self.gate_motor.get_position()
        self.gate_motor.set_dps(60)
        while angle <= 0:
            sleep(0.1)
            angle = self.gate_motor.get_position()
        self.gate_motor.set_dps(0)
        self.gate_motor.set_power(0)

    #Color Sensors
    def identify_ground_left(self):
        while self.color_left.get_rgb() == None:
            sleep(0.1)
        total = [0,0,0]
        for i in range(5):
            rgb = self.color_left.get_rgb()
            while rgb[0] == None:
                rgb = self.color_left.get_rgb()
                # print("color value is none")
            total[0] += rgb[0]
            total[1] += rgb[1]
            total[2] += rgb[2]
            sleep(0.1)
        average = [0,0,0]
        for i in range(3):
            average[i] += total[i]/3
        
        if average[0] > average[1] and average[0] > average[2]:
            #RED is the max
            if average[1] > 100:
                #If the green is also high then its yellow
                return "trash"
            return "red"
        if average[1] > average[2] and average[1] > average[0]:
            return "ground"
        else:
            return "water"

    def poll_color_sensors(self):
        #Continuously polls color sensors when active
        while self.color_sensor_active:
            left_ground = self.identify_ground_left() 
            right_ground = self.identify_block(self.color_sweep.get_rgb())
            if self.going_home:
                if left_ground == "trash":
                    self.trash_left = True
                if right_ground == "trash":
                    self.trash_right = True
            if left_ground == "water":
                self.isWaterLeft = True
            #MIGHT NEED CHANGE
            if right_ground == "water" and self.sweep_retracted:
                self.isWaterRight = True

            print(f"Left Color Sensor Water: {self.isWaterLeft}, Right Color Sensor Water: {self.isWaterRight}")
            sleep(0.1)  # Adjust polling rate if needed

    def start_color_sensor_polling(self):
        #Start the color sensor polling in a separate thread.
        if not self.color_sensor_active:
            self.color_sensor_active = True
            self.color_sensor_thread = threading.Thread(target=self.poll_color_sensors, daemon=True)
            self.color_sensor_thread.start()

    def stop_color_sensor_polling(self):
        #Stop the color sensor polling.
        self.color_sensor_active = False
        if self.color_sensor_thread is not None:
            self.color_sensor_thread.join()
            self.color_sensor_thread = None



    #Navigation
    def save_movement(self,direction,distance,smart = False):
        history = [direction[0].lower(),distance]
        self.movement_queue.append(history)
    
    def consolidate_movement(self):
    
        if not self.movement_queue:
            return []

        consolidated = [self.movement_queue[0]]  # Start with the first element
        for direction, value in self.movement_queue[1:]:
            # Compare current direction with the last direction in consolidated list
            if direction == consolidated[-1][0]:
                # Merge by adding the values
                consolidated[-1] = (direction, consolidated[-1][1] + value)
            else:
                # Add new direction as a separate block
                consolidated.append((direction, value))
        self.movement_queue = consolidated

    def go_home(self):
        reverse_dict = {"f":"b",
                        "b":"f",
                        "r":"l",
                        "l":"r"}
        print("reseting water variables going home using the stack",self.movement_queue)
        self.block_found = False
        self.isWaterLeft = False
        self.isWaterRight = False
        self.wall = False
        self.consolidate_movement()
        print("reseting water variables going home using the stack",self.movement_queue)
        while len(self.movement_queue) != 0:
            old_direction, old_distance = self.movement_queue.pop()
            reverse_direction = reverse_dict[old_direction]
            if reverse_direction == "f":
                self.move_forward(old_distance)
            elif reverse_direction == "b":
                self.move_backwards(old_distance)
            elif reverse_direction == "r" or reverse_direction == "l":
                self.rotate_robot(reverse_direction,old_distance)
            else:
                raise ValueError()
            self.movement_queue.pop()

    def check_water(self):

        if self.water_movement == 1:
            
            self.isWaterLeft = True
            self.isWaterRight = True
            return "water_both"
        self.water_movement += 1


        if self.isWaterLeft and self.isWaterRight:
            print("Very bad both sensor are in the water")
            return "water_both"
        
        if self.isWaterLeft:
            print("water avoidance on the left")
            self.dumb_backward(3)
            self.rotate_robot("r",15)
            self.isWaterLeft = False
            return "water_left"
            
        if self.isWaterRight:
            print("water avoidance on the right")
            self.dumb_backward(3)
            self.rotate_robot("l",15)
            self.isWaterRight = False
            return "water_right"
                
    def get_median_moving_slider(self):
        self.moving_slider.pop(0)
        distance = self.US.get_cm()
        while not distance:
            distance = self.US.get_cm()
        self.moving_slider.append(distance)
        #Slider has size 7
        median = sorted(self.moving_slider)[3]
        sleep(0.05)
        return median

    def wall_in_front(self):
        while self.US_sensor_active:
            distance = self.get_median_moving_slider()
            #print("distance found",distance)
            if self.block_found:
                if distance < 6:
                    self.wall = True
            else:
                if distance < 20:
                    self.wall = True
                    print("wall found")
                    self.moving_slider = [50,50,50,50,50,50,50,50,50]
            
    def start_US_sensor_polling(self):
        if not self.US_sensor_active:
            self.US_sensor_active = True
            self.US_thread = threading.Thread(target=self.wall_in_front, daemon=True)
            self.US_thread.start()

    def stop_US_sensor_polling(self):
        #Stop the color sensor polling.
        self.US_sensor_active = False
        if self.US_sensor_active is not None:
            self.US_thread.join()
            self.US_thread = None
    
    def check_wall(self):
        if self.wall:
            raise ValueError()
        
    #Movement
    def move_forward(self, distance):
        # Moves the robot forward and checks for water flags
        self.start_color_sensor_polling()  # Start polling color sensors
        self.start_US_sensor_polling()
        sleep(0.1)
        try:
            traveled_distance = 0
            while traveled_distance < distance:
                
                if self.isWaterLeft or self.isWaterRight:
                    self.block_found = False
                    break
                if self.wall:
                    break
                
                # Move forward in small increments
                self.motor_left.set_power(-self.power)
                self.motor_right.set_power(-self.power)
                sleep(0.1)  # Incremental movement
                traveled_distance += 0.1 * 8  # Convert time to distance (adjust scale factor if needed)
        finally:
            
            # Ensure motors stop and sensor polling ends
            self.motor_left.set_power(0)
            self.motor_right.set_power(0)
            self.stop_color_sensor_polling()  # Stop polling color sensors
            self.stop_US_sensor_polling() 
            self.save_movement("f",traveled_distance)
            self.check_water()
            return ("wall",self.wall,"right",self.isWaterRight,"left",self.isWaterLeft)
    
    
    def move_backwards(self,distance):
        traveled_distance = 0
        if self.going_home:
            self.start_color_sensor_polling()
            try:
                traveled_distance = 0
                while traveled_distance < distance:
                    # Check for water flags before proceeding
                    #self.check_water()
                    self.motor_left.set_power(self.power)
                    self.motor_right.set_power(self.power)
                    sleep(0.1)  # Incremental movement
                    traveled_distance += 0.1 * 8  # Convert time to distance (adjust scale factor if needed)
            finally:
                # Ensure motors stop and sensor polling ends
                self.motor_left.set_power(0)
                self.motor_right.set_power(0)
                self.stop_color_sensor_polling()  # Stop polling color sensors
                self.save_movement("b",traveled_distance)

        else:
            try:
                self.motor_left.set_power(self.power)
                self.motor_right.set_power(self.power)
                while traveled_distance < distance:
                    sleep(0.1)  # Incremental movement
                    traveled_distance += 0.1 * 8
            finally:
                self.motor_left.set_power(0)
                self.motor_right.set_power(0)
                self.save_movement("b",traveled_distance)

    def get_initial_angle(self):
        
        
        # Retrieve the initial angle
        gyro_measurements = self.gyro.get_both_measure()
        while gyro_measurements == None:
            print("No angle detected")
            gyro_measurements = self.gyro.get_both_measure()
        self.initial_angle = gyro_measurements[0]  # Save the initial angle
        #print("the intial angle" + str(self.initial_angle))
        return self.initial_angle
    
    def get_final_angle(self):

        gyro_measurements = self.gyro.get_both_measure()
        if not gyro_measurements:
            print("no final angle from the gyro",self.gyro.get_both_measure() )
            return -900000
        self.final_angle = self.gyro.get_both_measure()[0]
        #print("the final angle" + str(self.final_angle))
        return self.final_angle
    
    def rotate_robot(self, direction, target_angle):
        wait_ready_sensors()
        #Rotates the robot and polls the color sensors.
        try:
            self.get_initial_angle()  
            if direction[0].lower() == "l":
                left_power = 15 
                right_power = -15
                target_angle = -target_angle 
                # going left makes the final more negative than the itial
                #Therefore final - intial = -90
            else:
                left_power = -15
                right_power = 15   
            self.motor_left.set_power(left_power)
            self.motor_right.set_power(right_power)
            GIGACHAD=False
            rotation_slider = [1,10,20,30,40,50,60,70]

            while True:
                current_angle = self.get_final_angle()
                correction = 0
                
                if current_angle == -900000:
                    self.motor_left.set_power(0)
                    self.motor_right.set_power(0)
                    correction = 20
                    print("correction time GIGACHAD")
                    GIGACHAD = True
                elif GIGACHAD:
                    self.motor_left.set_power(left_power)
                    self.motor_right.set_power(right_power)
                    GIGACHAD = False
                if correction == 20:
                    if direction[0].lower() == "r":
                        diff = current_angle + correction - target_angle
                    if direction[0].lower() == "l":
                        diff = current_angle - correction - target_angle
                else:
                    diff = current_angle - self.initial_angle
                #print("keep turning" + str(diff))
                rotation_slider.pop(0)
                rotation_slider.append(diff)
                #print("rotation_slider",rotation_slider)
                if rotation_slider[0] == rotation_slider[-1] and rotation_slider[-1] == rotation_slider[1]:
                    if direction[0].lower() == "r":
                        print("we are stuck on right wall")
                        print(rotation_slider)
                    else:
                        print("we are stuck on left wall")
                    break

                if direction[0].lower() == "r" and diff >= target_angle:
                    break
                if direction[0].lower() == "l" and diff <= target_angle:
                    break
                sleep(0.1)

        finally:
            self.save_movement(direction[0].lower(),abs(target_angle))
            self.motor_left.set_power(0)
            self.motor_right.set_power(0)

    #SWEEP AND FIND STUFF
    def save_color_tup_to_arr(self,arr):
        color_tup = self.color_sweep.get_rgb()
        if not color_tup[0]:
            print("color is none")
            color_tup = [2,12,4]
        #print("appending",color_tup)
        arr.append(color_tup)

    def save_color_angle_to_arr(self,arr):
        angle = self.motor_sweep.get_encoder()
        arr.append(angle)
        
    def has_cubes_in_range(self,colors_array,angle_array):

        #Colors arry = [[1,2,3], [1,2,3]]
        cubes_arr = []
        for i in range(len(colors_array)):
            #print("identifying arr",colors_array[i])
            identity = self.identify_block(colors_array[i])
            #print(identity)
            if (identity == "poop"):
                print("poop" + str(colors_array[i]))
                return (angle_array[i],"p")
            if (identity == "obstacle"):
                print("found obstacle"+ str(colors_array[i]))
                
                return (angle_array[i],"o")
                
        # Colors array [(21,p)]
        return (-500,"n")

    def save_encoder_to_slider(self,value):
        self.encoder_moving_slider.pop(0)
        self.encoder_moving_slider.append(value)
        #print(self.encoder_moving_slider)
        diff1 = abs(self.encoder_moving_slider[-1] - self.encoder_moving_slider[1])
        diff2 = abs(self.encoder_moving_slider[-1] - self.encoder_moving_slider[0]) 
        if diff1 < 1 and diff2 < 1:
            self.is_stuck = True
        return value
    
    def outwardSweep(self,DPS_SWEEP,ANGLE_SWEEP):
        angle1 = -500
        identity1 = "n"
        self.clear_Readings()
        self.motor_sweep.reset_encoder()
        self.motor_sweep.set_dps(DPS_SWEEP)
        if not self.sweep_retracted:
            return (angle1,identity1)
        
        while self.motor_sweep.get_encoder() < ANGLE_SWEEP:
            self.save_encoder_to_slider(self.motor_sweep.get_encoder())
            if self.is_stuck:
                break    
            sleep(0.01)      
            self.save_color_tup_to_arr(self.Color_Readings)
            self.save_color_angle_to_arr(self.Ang_Readings)
        if self.is_stuck:
            self.is_stuck = False
            print("THE SWEEPER GOT STUCK")
        self.motor_sweep.set_dps(0)
        angle,identity1 = self.has_cubes_in_range(self.Color_Readings,self.Ang_Readings)
        #21,p  or 
        angle1 = angle
        print("angle detected first sweep " + str(angle) + " identity: " + identity1)
        self.sweep_retracted = False
        return (angle1,identity1)
    
    def inwardSweep(self,DPS_SWEEP,ANGLE_SWEEP):
        angle2 = -500
        identity2 = "n"
        self.clear_Readings()
        if self.sweep_retracted:
            return (angle2,identity2)
        self.motor_sweep.reset_encoder()
        self.motor_sweep.set_dps(-DPS_SWEEP)
        
        while self.motor_sweep.get_encoder() > -ANGLE_SWEEP-1:
            sleep(0.01)  
            self.save_color_tup_to_arr(self.Color_Readings)
            self.save_color_angle_to_arr(self.Ang_Readings)

        
        self.motor_sweep.set_dps(0)
        angle,identity2 = self.has_cubes_in_range(self.Color_Readings,self.Ang_Readings)
        #21,p  or 
        angle2 = angle
        print("angle detected second sweep " + str(angle) + " identity: " + identity2)
        self.sweep_retracted = True
        sleep(0.2)
        return (int(angle2) + int(ANGLE_SWEEP),identity2)

    def sweepAndDetect(self):
        DPS_SWEEP = 120
        ANGLE_SWEEP = 92
        
        #ASSUMING WE START IN RETRACTED POSITION
        #print(self.sweep_retracted)
        if self.sweep_retracted:
            angle1,identity1 = self.outwardSweep(DPS_SWEEP,ANGLE_SWEEP)
            if identity1 == "n":
                return (angle1,identity1)
            angle2,identity2 = self.inwardSweep(DPS_SWEEP,ANGLE_SWEEP)
            avg_angle = (angle1 + angle2) /2
            
        elif not self.sweep_retracted:
            angle2,identity2 = self.inwardSweep(DPS_SWEEP,ANGLE_SWEEP)
            if identity2 == "n":
                return (angle2,identity2)
            angle1,identity1 = self.outwardSweep(DPS_SWEEP,ANGLE_SWEEP)
            avg_angle = (angle1 + angle2) /2
        
        if identity1 == "o" and identity2 == "o":
            return (avg_angle,"o")
        if identity1 == "p" and identity2 == "p":
            return (avg_angle,"p")
        if identity1 == "p" or identity2 == "p":
            if angle1 == -500 and identity2 == "p":
                self.rotate_robot("l",10)
                self.sweepAndDetect()
            if angle2 == -500 and identity1 == "p": 
                self.rotate_robot("r",10)
                self.sweepAndDetect()
            diff = angle1 - angle2
            if abs(diff) > 15:
                return (avg_angle,"o")
            return (avg_angle,"p")
       
                        
    def capture_block(self,blockAngle):
        #Assume that we already are rotate facing the block
        self.block_found = True
        self.inwardSweep(120,85)
        self.rotateGateUp()
        print(self.movement_queue)
        self.dumb_forward(10)
        self.movement_queue.pop()
        print("block angle",blockAngle)
        additional_movement = abs(math.sin(blockAngle) * 15)
        print("additional movement",additional_movement)
        self.move_forward(additional_movement)
        
        self.rotateGateDown()

        self.dumb_backward(10)
        print(self.movement_queue)
        self.movement_queue.pop()
        print(self.movement_queue,"adjfnakjdf akdjfaskfjd hh")
        self.block_found = False
        
    def search(self):
        while self.time_difference():
            angle,identity = self.sweepAndDetect()
            print(f"searching {angle} + {identity}")
            
            if identity == "p":
                self.do_math_on_angle(angle)
                return "p"
            if identity == "o":
                return "o"

            status = self.move_forward(3)
            if status[1]:
                return "wall"
            if status[3] and status[5]:
                return "both"
            if status [3] and self.sweep_retracted:
                return "right"
            if status[5]:
                return "left"
    
    def do_math_on_angle(self,blockAngle):

        y_offset = 7
        x_offset = 15
        # print("nloc angle is " + str(blockAngle))
        
        if blockAngle >= 0:
            # print(blockAngle)
            blockAngle = math.radians(blockAngle)
            y_f = math.cos(blockAngle)* (14.5)
            x_f = math.sin(blockAngle) * (14.5)
                 
            RobotAngle = math.atan(abs(y_f - y_offset)/(x_f + x_offset))
            math.atan((y_offset - y_f)/(x_f + x_offset))
            # print(y_f)
            
            RobotAngle = math.degrees(RobotAngle)
            print("the robot angle is" + str(RobotAngle) + "\n")
            print(str(x_f) + "," + str(y_f) + "\n")
            print("y_f value:" + str(y_f) + "\n")
            if (y_f) < 7.5:
                print(f"rotating left to angle {blockAngle}")
                self.rotate_robot("l",RobotAngle)
                self.capture_block(blockAngle)
                self.rotate_robot("r",RobotAngle)
                
            elif(y_f) > 7.5:
                print(f"rotating right to angle {blockAngle}")
                self.rotate_robot("r",RobotAngle)
                self.capture_block(blockAngle)
                self.rotate_robot("L",RobotAngle)
            
    def dumb_forward(self,distance):
        print("dumb forward")
        try:
            traveled = 0
            while traveled < distance:
                if self.wall:
                    break
                # Move forward in small increments
                self.motor_left.set_power(-self.power)
                self.motor_right.set_power(-self.power)
                sleep(0.1)  # Incremental movement
                traveled += 0.1 * 5  # Convert time to distance (adjust scale factor if needed)

        finally:
            # Ensure motors stop and sensor polling ends
            print("travled",traveled)
            self.save_movement("f",traveled)
            self.motor_left.set_power(0)
            self.motor_right.set_power(0)
            
    def dumb_backward(self,distance):
        print("dumb forward")
        try:
            traveled = 0
            while traveled < distance:
                if self.wall:
                    break
                # Move forward in small increments
                self.motor_left.set_power(self.power)
                self.motor_right.set_power(self.power)
                sleep(0.1)  # Incremental movement
                traveled += 0.1 * 5 # Convert time to distance (adjust scale factor if needed)

        finally:
            # Ensure motors stop and sensor polling ends
            print("travled",traveled)
            self.save_movement("f",traveled)
            self.motor_left.set_power(0)
            self.motor_right.set_power(0)
           
    def edge_search(self):
        #NO WATER
        poop = 0
        walls = 0
        while True:
            thing = self.search()
            
            if thing == "0":
                self.go_home()
            if thing == "p":
                poop += 1
            if thing == "wall":
                print("we see a wall")
                self.wall = False
                walls += 1
                self.dumb_backward(3)
                self.rotate_robot("r",45)
                self.dumb_forward(5)
                self.rotate_robot("r",45)
                self.dumb_backward(3)
                
            if walls == 4:
                break

    def spray_search(self):
        while True:
            thing = self.search()
            if thing == "o":
                self.go_home()
                break
            if thing == "p":
                print("found poop")
            if thing == "wall":
                self.go_home()
                break
            if thing == "both":
                self.go_home()
                break
            if thing == "left" or thing == "right":
                status = self.check_water()
                if status == "water_both" or self.water_movement > 3:
                    self.go_home()
                    break
        
    def straight_search(self):
        while True:
            thing = self.search()
            if thing == "o":
                self.go_home()
                return "o"
            if thing == "p":
                print("found poop")
                self.go_home()
                return "p"
            if thing == "wall":
                self.go_home()
                return "wall"
            if thing == "both":
                self.go_home()
                return "both"
            if thing == "left" or thing == "right":
                self.go_home()
                return "water"

    def recursive_straight_search(self,angle):
        
        for i in range(2):
            while robot.straight_search() == "p":
                if angle != 0:
                    robot.dumb_forward(3)
                    robot.rotate_robot("l",angle)
                pass
        return 
        

if __name__ == "__main__":
    robot = Robot()
    angs = [0,30,50]
    
    
    try:
        for ang in angs:
            robot.recursive_straight_search(ang)
                
    except Exception as e:
        print(e)
    finally:
        robot.close()

