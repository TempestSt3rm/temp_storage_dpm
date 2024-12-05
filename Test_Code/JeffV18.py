import brickpi3
from utils.brick import EV3UltrasonicSensor, wait_ready_sensors, Motor, EV3GyroSensor, EV3ColorSensor
from time import sleep
import threading
import math

class Robot:
    def __init__(self):
        self.BP = brickpi3.BrickPi3()
        #self.BP.reset_all()
        self.power = 30
        self.dps = 20

        self.motor_left = Motor("A")
        self.motor_right = Motor("D")
        #Gate
        self.gate_motor = Motor("C")
        #Block Sensing
        self.motor_sweep = Motor("B")

        self.Ang_Readings = []
        self.Color_Readings = []

        #Color sensing
        self.color_left = EV3ColorSensor(3)
        self.color_sweep = EV3ColorSensor(4)
        self.color_sensor_thread = None
        self.color_sensor_active = False

        self.encoder_moving_slider = [0,20,30]
        self.is_stuck = False
        
        self.isWaterRight = False
        self.isWaterLeft = False

        #NAVIGATION
        self.US = EV3UltrasonicSensor(1)
        self.moving_slider = [50,50,50,50,50,50,50]

        self.US_thread = None
        self.US_sensor_active = False
        self.wall= False

        self.gyro_port = 2
        self.gyro = EV3GyroSensor(2)
        self.movement_queue = []
        #[("f",90) ]
        
        self.initial_angle = 0
        self.final_angle = 0

        self.trash_right = False
        self.trash_left = False
        self.going_home = False
        self.block_found = False
        wait_ready_sensors()
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
    
    def stop(self):
        self.motor_left.set_power(0)
        self.motor_right.set_power(0)

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
    
    # Return values:
    # 0: Not identified
    # 1: Yellow/orange cube (poop)
    # 2: Purple/green cube (person/bench)
    # 3: Green map
    # 4: Blue map (water)
    # 5: Yellow map (garbage)
    def identify_block(self,colorList):
        while colorList[0] == None:
            colorList = self.color_sweep.get_rgb()
        red = colorList[0]
        green = colorList[1]
        blue = colorList[2]
        total = red + green + blue
        if red  > 80:
            return "poop"
        normalizedGreen = self.normalizeGreen(colorList)
        normalizedBlue = self.normalizeBlue(colorList)
        normalizedRed = self.normalizeRed(colorList)
        if normalizedGreen > 0.5 and green < 50:
            return "ground"
        if normalizedBlue > 0.4:
            return "water"
        if (total > 70):
            return "obstacle"
        if total > 40 and normalizedRed > 0.4 and normalizedGreen > 0.3:
            return "trash"
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
        while self.color_left.get_rgb()[0] == None:
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
            if right_ground == "water":
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
    def save_movement(self,direction,distance):
        history = [direction[0].lower(),distance]
        if len(self.movement_queue) != 0:
            prev_dir, prev_dist = self.movement_queue[-1]
            if direction[0].lower() == prev_dir:
                self.movement_queue.pop()
                history[1] = distance + prev_dist
        self.movement_queue.append(history)

    def go_home(self):
        reverse_dict = {"f":"b",
                        "b":"f",
                        "r":"l",
                        "l":"r"}
        print(self.movement_queue)
        self.block_found = False
        
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
        
        if self.isWaterLeft:
            self.move_backwards(1)
            self.rotate_robot("r",10)
            self.isWaterLeft = False
        if self.isWaterRight:
            self.rotate_robot("l",10)
            self.isWaterRight = False

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
    
    def reset_wall(self):
        self.moving_slider.clear()

    def wall_in_front(self):
        while self.US_sensor_active:
            distance = self.get_median_moving_slider()
            print("distance found",distance)
            if self.block_found:
                if distance < 6:
                    self.wall = True
            else:
                if distance < 13:
                    self.wall = True
            
        
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
                # Check for water flags before proceeding
                self.check_water()

                #Check wall flage
                self.check_wall()
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
    
    def move_backwards(self,distance):
        traveled_distance = 0
        if self.going_home:
            self.start_color_sensor_polling()
            try:
                traveled_distance = 0
                while traveled_distance < distance:
                    # Check for water flags before proceeding
                    self.check_water()
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
        wait_ready_sensors()  # Ensure the sensor is ready after reset
        
        # Retrieve the initial angle
        gyro_measurements = self.gyro.get_both_measure()
        while gyro_measurements[0] == None:
            print("No angle detected")
        self.initial_angle = gyro_measurements[0]  # Save the initial angle
        #print("the intial angle" + str(self.initial_angle))
        return self.initial_angle

    def get_final_angle(self):
        while not self.gyro.get_both_measure():
            unconfirmed_angle = self.gyro.get_both_measure()
            print("no angle")
        self.final_angle = self.gyro.get_both_measure()[0]
        #print("the final angle" + str(self.final_angle))
        return self.final_angle

    def rotate_robot(self, direction, target_angle):
        #Rotates the robot and polls the color sensors.
        try:
            self.get_initial_angle()  
            if direction[0].lower() == "l":
                left_power = self.power 
                right_power = -self.power
                target_angle = -target_angle 
                # going left makes the final more negative than the itial
                #Therefore final - intial = -90
            else:
                left_power = -self.power   
                right_power = self.power   
            self.motor_left.set_power(left_power)
            self.motor_right.set_power(right_power)

            while True:
                current_angle = self.get_final_angle()  
                diff = current_angle - self.initial_angle
                #print("keep turning" + str(diff))

                if direction[0].lower() == "r" and diff >= target_angle:
                    break
                if direction[0].lower() == "l" and diff <= target_angle:
                    break

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
        return (0,"n")
    
    def save_encorder_to_slider(self,value):
        self.encoder_moving_slider.pop(0)
        self.encoder_moving_slider.append(value)
        diff1 = abs(self.encoder_moving_slider[2] - self.encoder_moving_slider[1])
        diff2 = abs(self.encoder_moving_slider[2] - self.encoder_moving_slider[0]) 
        if diff1 < 1 and diff2 < 1:
            self.is_stuck = True
        return value
        

    def sweepAndDetect(self):
        angle1 = -500
        angle2 = -500
        identity1 = "n"
        identity2 = "n"
        DPS_SWEEP = 80
        ANGLE_SWEEP = 100

        self.clear_Readings()
        #ASSUMING WE START IN RETRACTED POSITION
        
        self.motor_sweep.reset_encoder()
        self.motor_sweep.set_dps(DPS_SWEEP)
        print(self.motor_sweep.get_encoder())

        while self.motor_sweep.get_encoder() < ANGLE_SWEEP:
            sleep(0.1)      
            self.save_color_tup_to_arr(self.Color_Readings)
            self.save_color_angle_to_arr(self.Ang_Readings)
        self.motor_sweep.set_dps(0)
        angle,identity1 = self.has_cubes_in_range(self.Color_Readings,self.Ang_Readings)
        #21,p  or 
        angle1 = angle
        print("angle detected first sweep " + str(angle) + " identity: " + identity1)
        

        self.motor_sweep.set_dps(-DPS_SWEEP)
        self.clear_Readings()
        
        while self.save_encoder_to_slider(self.motor_sweep.get_encoder()) > 2:
            sleep(0.10)
            if self.is_stuck:
                self.is_stuck = False
                break      
            self.save_color_tup_to_arr(self.Color_Readings)
            self.save_color_angle_to_arr(self.Ang_Readings)

        if self.is_stuck:
            print("THE SWEEPER GOT STUCK")
        self.motor_sweep.set_dps(0)
        angle,identity2 = self.has_cubes_in_range(self.Color_Readings,self.Ang_Readings)
        #21,p  or 
        angle2 = angle
        print("angle detected second sweep " + str(angle) + " identity: " + identity2)

        avg_angle = (angle1 + angle2) /2
        if identity1 == "o" and identity2 == "o":
            
            return (avg_angle,"o")
        if identity1 == "p" and identity2 == "p":
            return (avg_angle,"p")
        
        if identity1 == "p" or identity2 == "p":
            diff = angle1 - angle2
            if abs(diff) > 15:
                return (avg_angle,"o")
            return (avg_angle,"p")
        
        return (-500,"n")

    def do_math_on_angle(self,blockAngle):

        y_offset = 5.5
        x_offset = 17
        # print("nloc angle is " + str(blockAngle))
        
        if blockAngle >= 0:
            # print(blockAngle)
            blockAngle = math.radians(blockAngle)
            x_f = math.cos(blockAngle)* (13.5)
            y_f = math.sin(blockAngle) * (13.5)
            if (y_f > 5.5):             
                RobotAngle = math.atan((y_f - y_offset)/(x_f + x_offset))
            elif (y_f < 5.5):
                RobotAngle = math.atan((y_offset - y_f)/(x_f + x_offset))
            # print(y_f)
            
            RobotAngle = math.degrees(RobotAngle)
            print("the robot angle is" + str(RobotAngle) + "\n")
            print(str(x_f) + "," + str(y_f) + "\n")
            print("y_f value:" + str(y_f) + "\n")
            if (y_f) > 5.5:
                print(f"rotating left to angle {blockAngle}")
                self.rotate_robot("l",blockAngle + 3)
                      
            elif(y_f) < 5.5:
                print(f"rotating right to angle {blockAngle}")
                self.rotate_robot("r",blockAngle)
                        
    def capture_block(self,blockAngle):
        #Assume that we already are rotate facing the block
        self.block_found = True
        self.rotateGateUp()
        self.move_forward(10)
        self.movement_queue.pop()
        additional_movement = abs(math.sin(math.degrees(blockAngle)) * 7)
        print("additional movement",additional_movement)
        self.move_forward(additional_movement)

        self.rotateGateDown()

        self.move_forward(3)
        self.movement_queue.pop()

        self.move_backwards(5)
        self.movement_queue.pop()

        self.save_movement("f",2)
         
    def search_return(self):
        while True:
            angle,identity = self.sweepAndDetect()
            if identity == "p":
                self.do_math_on_angle(angle)
                self.capture_block(angle)
                break
            if identity == "o":
                break
            self.move_forward(2)
        self.go_home()
        self.rotateGateUp()
        sleep(1)
        self.rotateGateDown()


if __name__ == "__main__":
    
    robot = Robot()
    try:
        # robot.rotate_robot("L", 90)
        #robot.rotate_robot("R", 90)
        #robot.sweepAndDetect()
        arr_ang = [0,20,60]
        robot.search_return()
        
    except Exception as e:
        print(e)
    finally:
        robot.close()
        
