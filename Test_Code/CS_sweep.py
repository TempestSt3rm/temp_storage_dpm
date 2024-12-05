#Orange 170 40 30
#Yellow 160 100 13
#Purple Cube 40 25 44
# Green Cube 15 60 18

#Green map 9 10 4
#Blue Map 3 3 7
#Yellow Map 25 20 4


from utils.brick import EV3ColorSensor, Motor, wait_ready_sensors
from time import sleep
import brickpi3

Sweep = Motor('B')

wait_ready_sensors(True)
sleep(2)

def collect_motor_data():
    Sweep.reset_encoder()
    for i in range(50):
        print(Sweep.get_encoder())
        sleep(0.5)
    return

try:
    BP = brickpi3.BrickPi3()
    collect_motor_data()
except:
    BP.reset_all()

#Orange 170 40 30
#Yellow 160 100 13

#Purple Cube 40 25 44
#Green Cube 15 60 18

#Green map 9 10 4
#Blue Map 3 3 7
#Yellow Map 25 20 4

