from utils.brick import EV3ColorSensor, wait_ready_sensors
from time import sleep
import brickpi3

data_file = "./CS.csv"
Color = EV3ColorSensor(4)

wait_ready_sensors(True)
sleep(2)

def collect_CS_data():
    for i in range(50):
        sleep(0.5)
    return

try:
    BP = brickpi3.BrickPi3()
    collect_CS_data()
except:
    BP.reset_all()

#Orange 170 40 30
#Yellow 160 100 13

#Purple Cube 40 25 44
#Green Cube 15 60 18

#Green map 9 10 4
#Blue Map 3 3 7
#Yellow Map 25 20 4

