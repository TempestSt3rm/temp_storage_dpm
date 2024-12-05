from collections import defaultdict
import brickpi3
from utils.brick import EV3UltrasonicSensor, wait_ready_sensors, Motor, EV3GyroSensor,EV3ColorSensor
from time import sleep

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

        self.angle_dist_dict = defaultdict()
        self.initial_angle = 0
        self.final_angle = 0

        self.power = 20
        self.dps = 5
        self.angle_range = 60
        self.setup_motors()

    def get_initial_angle(self):
        """
        Resets the gyro sensor and retrieves the initial angle.
        """
        # Reset the gyro sensor by disabling and re-enabling its type
        self.BP.set_sensor_type(self.gyro_port, self.BP.SENSOR_TYPE.NONE)  # Disable the sensor
        sleep(0.5)  # Allow time for the reset
        self.BP.set_sensor_type(self.gyro_port, self.BP.SENSOR_TYPE.EV3_GYRO_ABS_DPS)  # Reinitialize as gyro
        
        wait_ready_sensors(True)  # Ensure the sensor is ready after reset
        
        
        # Retrieve the initial angle
        gyro_measurements = self.gyro.get_both_measure()
        if gyro_measurements is None:
            print("No angle detected")
            return 0
        self.initial_angle = gyro_measurements[0]  # Save the initial angle
        print("the intial angle" + str(self.initial_angle))
        return self.initial_angle
    
    def get_final_angle(self):
        # Reset the gyro by recalibrating
        # self.BP.set_sensor_type(self.gyro_port, self.BP.SENSOR_TYPE.NONE)  # Reset the sensor
        # sleep(0.5)
        # self.BP.set_sensor_type(self.gyro_port, self.BP.SENSOR_TYPE.EV3_GYRO_ABS_DPS)  # Reinitialize gyro
        while not self.gyro.get_both_measure():
            unconfirmed_angle = self.gyro.get_both_measure()
            print("no angle")
        self.final_angle = self.gyro.get_both_measure()[0]
        print("the final angle" + str(self.final_angle))
        return self.final_angle
            

    def get_diff_of_angles(self):
        diff = self.get_final_angle() - self.initial_angle
        #If the diff is positive it means the robot moved to far to the right (clockwise), need to rotate back left ccw
        #IF the diff is negative vice versa
        self.initial_angle = 0
        return diff

    def setup_motors(self):
        self.motor_left.set_limits(70, self.power)
        self.motor_right.set_limits(70, self.power)
        self.motor_sweep.set_limits(70, self.power)
        self.motor_sweep.reset_encoder()
        self.gate_motor.reset_encoder()

    def move_forward(self, distance):
        self.motor_left.set_power(-self.power)
        self.motor_right.set_power(-self.power)
        sleep(distance / 8)
        self.motor_left.set_power(0)
        self.motor_right.set_power(0)
        
    def rotate_robot(self, direction, target_angle):
        self.get_initial_angle()  # Reset the gyro sensor and get the current angle
    
        # Set power based on direction
        left_power = self.power if direction[0].lower() == "l" else -self.power
        right_power = -self.power if direction[0].lower() == "l" else self.power
        self.motor_left.set_power(left_power)
        self.motor_right.set_power(right_power)

        while True:
            current_angle = self.get_final_angle()  # Get the current angle from the gyro
            diff = abs(current_angle - self.initial_angle)  # Calculate the rotation difference
        
            if diff >= target_angle:  # Stop when the desired angle is reached
                break

        # Stop the motors
        self.motor_left.set_power(0)
        self.motor_right.set_power(0)


    
        
    def rotate_motor_small_angle(self, motor, angle):
        
        initial_position = motor.get_encoder()  # Get the current encoder position
        target_position = initial_position + angle  # Calculate the target position
        
        # Set the motor power and direction
        self.motor_left.set_dps(50)  # Set a low speed for precision
        
        while True:
            current_position = motor.get_encoder()
            if (angle > 0 and current_position >= target_position) or (angle < 0 and current_position <= target_position):
                break  # Stop when the target is reached
            
        motor.set_power(0)  # Stop the motor

    def correct_rotation(self,angle):
        diff = self.get_diff_of_angles()
        print("the diff is" + str(diff))
        diff = diff + angle
        if diff > 0:
            print("correct by rotating right")
            self.rotate_robot("right",abs(diff))
        else:
            self.rotate_robot("left",abs(diff))
            print("correct by rotating left")
        return diff

    def rotate_robot_accurate(self, direction, angle):
        self.get_initial_angle()
        self.rotate_robot(direction,angle)
        diff = self.correct_rotation(angle)
        print(f"Intended Rotatation: {angle} Corrected a diff of {diff}")

    # def save_distance

    def rotate_gate(self, direction, target_angle):
        dps = -15 if direction == "up" else 15
        self.gate_motor.set_dps(dps)
        while True:
            position = self.gate_motor.get_position()
            if (direction == "up" and position <= target_angle) or (direction == "down" and position >= target_angle):
                break
        self.gate_motor.set_dps(0)

    def sweep(self, direction, angle_limit):
        # dps = self.dps if direction == "left" else -self.dps
        # self.motor_sweep.set_dps(dps)
        # while True:
        #     position = self.motor_sweep.get_encoder()
        #     if (direction == "left" and position >= angle_limit) or (direction == "right" and position <= angle_limit):
        #         break
        # self.motor_sweep.set_dps(0)
        print("NOT READY")

    def collect_data(self):
        # angle = self.motor_sweep.get_encoder()
        # bottom_dist = self.us_bottom.get_cm()
        # self.angle_dist_dict[angle] = bottom_dist
        print(f"NEED SWEEP CODE ")

    def sweep_and_measure(self, direction, angle_limit):
        # dps = self.dps if direction == "left" else -self.dps
        # self.motor_sweep.set_dps(dps)
        # while True:
        #     position = self.motor_sweep.get_encoder()
        #     if (direction == "left" and position >= angle_limit) or (direction == "right" and position <= angle_limit):
        #         break
        #     self.collect_data()
        #     sleep(0.2)
        # self.motor_sweep.set_dps(0)
        print("NOT READY")

    def find_closest_object(self):
        # if not self.angle_dist_dict:
        #     return None, None
        # sorted_angles = sorted(self.angle_dist_dict.items(), key=lambda x: x[1])
        # return sorted_angles[0]
        print("NOT READY")

    def execute(self):
        try:
            wait_ready_sensors(True)
            
        except Exception as e:
            print(f"Error: {e}")
        finally:
            self.motor_sweep.reset_encoder()
            self.BP.reset_all()

if __name__ == "__main__":
    robot = Robot()
    # robot.execute()
    #robot.rotate_robot_accurate("L",90)
    robot.rotate_robot("L",90)
    robot.rotate_robot("R",90)

