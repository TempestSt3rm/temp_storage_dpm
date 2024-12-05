import brickpi3
from utils.brick import wait_ready_sensors, Motor, EV3GyroSensor, EV3ColorSensor
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
        
        self.isWaterRight = False
        self.isWaterLeft = False

        #NAVIGATION
        self.gyro_port = 2
        self.gyro = EV3GyroSensor(2)
        self.movement_queue = []
        #[("f",90) ]
        
        self.initial_angle = 0
        self.final_angle = 0
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
        red = colorList[0]
        green = colorList[1]
        blue = colorList[2]
        total = red + green + blue
        if red  > 50:
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
        if normalizedRed > 0.4 and red < 40:
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
    def isWater(self,CS):
        blueTotal = 0
        for i in range(5):
            rgb = CS.get_rgb()
            print(rgb)
            while not rgb[0]:
                print("rgb detected is" + str(rgb))
                rgb = CS.get_rgb()
                
            blueTotal += self.normalizeBlue(rgb)
            print("detected" + str(rgb))
            sleep(0.001)
        blueTotal /= 5
        if blueTotal >= 0.4:
            return True
        else:
            return False

    def poll_color_sensors(self):
        #Continuously polls color sensors when active
        while self.color_sensor_active:
            self.isWaterLeft = self.isWater(self.color_left)
            #MIGHT NEED CHANGE
            self.isWaterRight = self.isWater(self.color_sweep)
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
        
    #Movement
    def move_forward(self, distance):
        # Moves the robot forward and checks for water flags
        self.start_color_sensor_polling()  # Start polling color sensors
        try:
            traveled_distance = 0
            while traveled_distance < distance:
                # Check for water flags before proceeding
                if self.isWaterLeft or self.isWaterRight:
                    print("Water detected! Stopping forward movement.")
                    raise ValueError()
                      # Stop movement if water is detected
                
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
            self.save_movement("f",traveled_distance)

    def move_backwards(self,distance):
        traveled_distance = 0
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
        while gyro_measurements is None:
            print("No angle detected")
        self.initial_angle = gyro_measurements[0]  # Save the initial angle
        print("the intial angle" + str(self.initial_angle))
        return self.initial_angle

    def get_final_angle(self):
        while not self.gyro.get_both_measure():
            unconfirmed_angle = self.gyro.get_both_measure()
            print("no angle")
        self.final_angle = self.gyro.get_both_measure()[0]
        print("the final angle" + str(self.final_angle))
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
                print("keep turning" + str(diff))

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
            color_tup = [9,10,4]
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
    
    def sweepAndDetect(self):
        angle1 = -500
        angle2 = -500
        identity1 = "n"
        identity2 = "n"
        DPS_SWEEP = 80
        ANGLE_SWEEP = 90

        self.clear_Readings()
        #ASSUMING WE START IN RETRACTED POSITION
        
        self.motor_sweep.reset_encoder()
        self.motor_sweep.set_dps(DPS_SWEEP)
        print(self.motor_sweep.get_encoder())

        while self.motor_sweep.get_encoder() < ANGLE_SWEEP:
            sleep(0.10)      
            self.save_color_tup_to_arr(self.Color_Readings)
            self.save_color_angle_to_arr(self.Ang_Readings)
        self.motor_sweep.set_dps(0)
        angle,identity1 = self.has_cubes_in_range(self.Color_Readings,self.Ang_Readings)
        #21,p  or 
        angle1 = angle
        print("angle detected first sweep " + str(angle) + " identity: " + identity1)

        self.motor_sweep.set_dps(-DPS_SWEEP)
        self.clear_Readings()
        
        while self.motor_sweep.get_encoder() > 0:
            sleep(0.10)      
            self.save_color_tup_to_arr(self.Color_Readings)
            self.save_color_angle_to_arr(self.Ang_Readings)
        self.motor_sweep.set_dps(0)
        angle,identity2 = self.has_cubes_in_range(self.Color_Readings,self.Ang_Readings)
        #21,p  or 
        angle2 = angle
        print("angle detected second sweep " + str(angle) + " identity: " + identity2)

        avg_angle = (angle1 + angle2) /2
        if identity1 == "o" and identity2 == "o":
            
            return (avg_angle,"o")
        if identity1 == "p" or identity2 == "p":
            return (avg_angle,"p")
        
        return (-500,"n")

    def do_math_on_angle(self,blockAngle):

        y_offset = 5.5
        x_offset = 17
        print("nloc angle is " + str(blockAngle))
        
        if blockAngle >= 0:
            print(blockAngle)
            blockAngle = math.radians(blockAngle)
            x_f = math.cos(blockAngle)* (13.5)
            y_f = math.sin(blockAngle) * (13.5)
            if (y_f > 5.5):             
                RobotAngle = math.atan((y_f - y_offset)/(x_f + x_offset))
            elif (y_f < 5.5):
                RobotAngle = math.atan((y_offset - y_f)/(x_f + x_offset))
            print(y_f)
            
            RobotAngle = math.degrees(RobotAngle)
            print("the robot angle is" + str(RobotAngle) + "\n")
            print(str(x_f) + "," + str(y_f) + "\n")
            print("y_f value:" + str(y_f) + "\n")
            if (y_f) > 5.5:
                print(f"rotating right to angle {blockAngle}")
                self.rotate_robot("r",blockAngle + 5)
                      
            elif(y_f) < 5.5:
                print(f"rotating left to angle {blockAngle}")
                self.rotate_robot("l",blockAngle + 5)
                        
    def capture_block(self):
        #Assume that we already are rotate facing the block
        self.rotateGateUp()
        self.move_forward(15)
        self.rotateGateDown()
        self.move_backwards(2)
        self.rotateGateDown()
        self.move_backwards(3)
         

if __name__ == "__main__":
    
    robot = Robot()
    try:
        # robot.rotate_robot("L", 90)
        #robot.rotate_robot("R", 90)
        robot.sweepAndDetect()
        # while True:
        #     angle,identity = robot.sweepAndDetect()
        #     if identity == "p":
        #         robot.do_math_on_angle( angle)
        #         robot.capture_block()
        #         break
        #     if identity == "o":
        #         break
        #     robot.move_forward(2)
        # robot.go_home()
        while True:
            colors = robot.color_sweep.get_rgb()
            robot.identify_block(colors)
        
    except Exception as e:
        print(e)
    finally:
        robot.close()
        