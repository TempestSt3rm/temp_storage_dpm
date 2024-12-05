import brickpi3
from utils.brick import TouchSensor,EV3UltrasonicSensor,wait_ready_sensors,EV3GyroSensor, Motor, reset_brick
from time import sleep


print("start")
BP = brickpi3.BrickPi3()
GYRO = EV3GyroSensor("1")
POWER = 40

print("end setup")

BP.reset_all()

try:
    GYRO.reset_measure()
    for i in range(100):
        print(GYRO.get_both_measure())
        sleep(0.5)
    GYRO.reset_measure()
except:
    GYRO.reset_measure()
