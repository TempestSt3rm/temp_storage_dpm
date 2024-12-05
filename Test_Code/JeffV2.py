from collections import defaultdict
import brickpi3
from utils.brick import EV3UltrasonicSensor, wait_ready_sensors, Motor, EV3GyroSensor, EV3ColorSensor
from time import sleep
import threading

class Robot:
    def __init__(self):
        self.BP = brickpi3.BrickPi3()
        self.motor_left = Motor("A")
        self.motor_right = Motor("D")
        self.motor_sweep = Motor("B")
        self.gate_motor = Motor("C")

        self.us_bottom = EV3UltrasonicSensor(1)
        self.gyro_port = 2
        self.gyro = EV3GyroSensor(2)
        self.color_left = EV3ColorSensor(3)
        self.color_right = EV3ColorSensor(4)
        
        self.isWaterRight = False
        self.isWaterLeft = False

        self.movement_dict = defaultdict()
        #{"f": 390, "L":90, "r",49}
        self.initial_angle = 0
        self.final_angle = 0

        self.power = 20
        self.dps = 5
        self.angle_range = 60
        self.color_sensor_active = False
        self.color_sensor_thread = None
        self.setup_motors()

    def setup_motors(self):
        self.motor_left.set_limits(70, self.power)
        self.motor_right.set_limits(70, self.power)
        self.motor_sweep.set_limits(70, self.power)
        self.motor_sweep.reset_encoder()
        self.gate_motor.reset_encoder()

    def rotateRight(self,time):
        self.motor_left.set_power(self.power * (-1))
        self.motor_right.set_power(self.power * (1))
        sleep(time)
        self.motor_left.set_power(0)
        self.motor_right.set_power(0)
    
    def rotateLeft(self,time):
        self.motor_left.set_power(self.power * (1))
        self.motor_right.set_power(self.power * (-1))
        sleep(time)
        self.motor_left.set_power(0)
        self.motor_right.set_power(0)
    
    def normalizeBlue(self,colorList):
        denominator = (colorList[0] + colorList[1] + colorList[2])
        if denominator == 0:
            return 0
        return colorList[2]/(colorList[0] + colorList[1] + colorList[2])
            
    def isWater(self,CS):
        blueTotal = 0
        for i in range(5):
            rgb = CS.get_rgb()
            print(rgb)
            while not rgb[0]:
                print(rgb)
                rgb = CS.get_rgb()
            blueTotal += self.normalizeBlue(rgb)
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
            self.isWaterRight = self.isWater(self.color_right)
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

    def move_forward(self, distance):
        # Moves the robot forward and checks for water flags
        self.start_color_sensor_polling()  # Start polling color sensors
        try:
            traveled_distance = 0
            while traveled_distance < distance:
                # Check for water flags before proceeding
                if self.isWaterLeft or self.isWaterRight:
                    print("Water detected! Stopping forward movement.")
                    break  # Stop movement if water is detected
                
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

    def rotate_robot(self, direction, target_angle):
        #Rotates the robot and polls the color sensors.
        self.get_initial_angle()  
        if direction[0].lower() == "l":
            left_power = self.power 
            right_power = -self.power
        else:
            left_power = -self.power   
            right_power = self.power   
        self.motor_left.set_power(left_power)
        self.motor_right.set_power(right_power)
        while True:
            current_angle = self.get_final_angle()  
            diff = abs(current_angle - self.initial_angle)
            if diff >= target_angle:
                break
        self.motor_left.set_power(0)
        self.motor_right.set_power(0)

if __name__ == "__main__":
    robot = Robot()
    # robot.rotate_robot("L", 90)
    # robot.rotate_robot("R", 90)
    robot.move_forward(50)
    robot.rotate_robot("l",30)
    robot.move_forward(50)

