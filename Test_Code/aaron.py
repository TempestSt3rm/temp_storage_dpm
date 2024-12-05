from collections import defaultdict
import brickpi3
from utils.brick import EV3UltrasonicSensor, wait_ready_sensors, Motor
from time import sleep

class Robot:
    def __init__(self):
        self.BP = brickpi3.BrickPi3()
        self.motor_left = Motor("A")
        self.motor_right = Motor("D")
        self.motor_sweep = Motor("B")
        self.gate_motor = Motor("C")
        self.us_bottom = EV3UltrasonicSensor(1)
        self.us_top = EV3UltrasonicSensor(2)
        self.angle_dist_dict = defaultdict()
        self.power = 30
        self.dps = 5
        self.angle_range = 60
        self.setup_motors()

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

    def rotate(self, direction, angle):
        left_power = self.power if direction == "left" else -self.power
        right_power = -self.power if direction == "left" else self.power
        self.motor_left.set_power(left_power)
        self.motor_right.set_power(right_power)
        sleep(angle / 45)
        self.motor_left.set_power(0)
        self.motor_right.set_power(0)

    def rotate_gate(self, direction, target_angle):
        dps = -15 if direction == "up" else 15
        self.gate_motor.set_dps(dps)
        while True:
            position = self.gate_motor.get_position()
            if (direction == "up" and position <= target_angle) or (direction == "down" and position >= target_angle):
                break
        self.gate_motor.set_dps(0)

    def sweep(self, direction, angle_limit):
        dps = self.dps if direction == "left" else -self.dps
        self.motor_sweep.set_dps(dps)
        while True:
            position = self.motor_sweep.get_encoder()
            if (direction == "left" and position >= angle_limit) or (direction == "right" and position <= angle_limit):
                break
        self.motor_sweep.set_dps(0)

    def collect_data(self):
        angle = self.motor_sweep.get_encoder()
        bottom_dist = self.us_bottom.get_cm()
        top_dist = self.us_top.get_cm()
        if abs(top_dist - bottom_dist) > 10:
            self.angle_dist_dict[angle] = bottom_dist
        if top_dist < 30:
            print(f"Wall detected at angle {angle} with distance {top_dist}")

    def sweep_and_measure(self, direction, angle_limit):
        dps = self.dps if direction == "left" else -self.dps
        self.motor_sweep.set_dps(dps)
        while True:
            position = self.motor_sweep.get_encoder()
            if (direction == "left" and position >= angle_limit) or (direction == "right" and position <= angle_limit):
                break
            self.collect_data()
            sleep(0.2)
        self.motor_sweep.set_dps(0)

    def find_closest_object(self):
        if not self.angle_dist_dict:
            return None, None
        sorted_angles = sorted(self.angle_dist_dict.items(), key=lambda x: x[1])
        return sorted_angles[0]

    def execute(self):
        try:
            wait_ready_sensors(True)
            self.sweep("left", self.angle_range)
            self.sweep_and_measure("right", -self.angle_range)
            target_angle, target_distance = self.find_closest_object()

            if target_angle is not None:
                if target_angle > 0:
                    self.rotate("left", target_angle)
                else:
                    self.rotate("right", abs(target_angle))

                self.move_forward(target_distance / 2)
                self.rotate_gate("up", -90)
                self.move_forward(1)
                self.rotate_gate("down", -2)

        except Exception as e:
            print(f"Error: {e}")
        finally:
            self.motor_sweep.reset_encoder()
            self.BP.reset_all()

if __name__ == "__main__":
    robot = Robot()
    robot.execute()
