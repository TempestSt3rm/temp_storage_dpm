import brickpi3
from utils.brick import TouchSensor,EV3UltrasonicSensor,wait_ready_sensors,EV3GyroSensor, Motor, reset_brick, EV3ColorSensor
from time import sleep

BP = brickpi3.BrickPi3()
gate_Motor = Motor("C")
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
    angle = gate_Motor.get_position()
    gate_Motor.set_dps(-30)
    print("rotating up")
    while angle >= -75:
        sleep(0.1)
        angle = gate_Motor.get_position()
    
    gate_Motor.set_dps(0)
    sleep(1)

def rotatedown():
    angle = gate_Motor.get_position()
    gate_Motor.set_dps(15)
    print("rotating down")
    while angle <= -2:
        sleep(0.1)
        angle = gate_Motor.get_position()
    gate_Motor.set_dps(0)
    sleep(4)


Motor1 = Motor("A")
Motor2 = Motor("D")
POWER = 30
Motor1.set_limits(70, POWER)
Motor2.set_limits(70, POWER)
print("end setup")

def MoveDistFwd(time):
    Motor1.set_power((-1)*POWER)
    Motor2.set_power((-1)*POWER)
    sleep(time)
    Motor1.set_power(0)
    Motor2.set_power(0)
        
def Rotate(time):
    Motor1.set_power(POWER)
    Motor2.set_power(POWER * (-1))
    sleep(time) 
    Motor1.set_power(0)
    Motor2.set_power(0)
 
try: 
    wait_ready_sensors(True)
    sleep(1)
    gate_Motor.reset_encoder()
    print("Ready")
    sleep(1)
    rotateup()
    MoveDistFwd(1)
    collect_CS_data()
    sleep(2)
    MoveDistFwd(1)
    print("going down")
    rotatedown()
    BP.reset_all()
except:
    print("interupt")
    gate_Motor.set_dps(0)
    BP.reset_all()