import brickpi3
from utils.brick import TouchSensor,EV3UltrasonicSensor,wait_ready_sensors,EV3GyroSensor, Motor, reset_brick, EV3ColorSensor
from time import sleep
data_file = "Final_Project_Test_CSVs/WHATEVER.csv"
BP = brickpi3.BrickPi3()
Motor1 = Motor("C")
Color = EV3ColorSensor(4)

def collect_CS_data():
    for i in range(50):
        print(Color.get_rgb())

    # with open(data_file, "w") as file:
    #     file.write("Red,Green,Blue\n")
    #     for i in range(50):
    #         line = ""
    #         for value in Color.get_rgb():
    #             line += str(value) + ","
    #         print(Color.get_rgb())
    #         file.write(line + "\n")
    #         sleep(0.5)
    return

def rotateup():
    angle = Motor1.get_position()
    Motor1.set_dps(-30)
    print("rotating up")
    while angle >= -80:
        sleep(0.1)
        angle = Motor1.get_position()
    
    Motor1.set_dps(0)
    sleep(1)

def rotatedown():
    
    angle = Motor1.get_position()
    Motor1.set_dps(15)
    print("rotating down")
    while angle <= -2:
        sleep(0.1)
        angle = Motor1.get_position()
    Motor1.set_dps(0)
    sleep(4)
 
 
try: 
    wait_ready_sensors(True)
    sleep(2)
    Motor1.reset_encoder()
    print("Ready")
    sleep(1)
    rotateup()
    collect_CS_data()
    sleep(2)
    print("going down")
    rotatedown()
    BP.reset_all()
except:
    print("interupt")
    Motor1.set_dps(0)
    BP.reset_all()