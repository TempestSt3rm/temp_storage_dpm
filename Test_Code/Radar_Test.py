import brickpi3
from utils.brick import TouchSensor,EV3UltrasonicSensor,wait_ready_sensors,EV3GyroSensor, Motor, reset_brick
from time import sleep


print("start")
BP = brickpi3.BrickPi3()
Motor1 = Motor("B")
US = EV3UltrasonicSensor(2)
POWER = 40
#Motor1.set_(70, 45)
print("end setup")

data_file = "/home/pi/Desktop/Team10Data/Radar_Rotation_Test.csv"


def Rotate ():
    #Motor1.set_position(30)
    
    position = Motor1.get_position()
    Motor1.set_dps(60)   
    while True:
        value = Motor1.get_position() 
        if (value-position) == 20:
            Motor1.set_dps(0)
            break
               
            
    

def collect_data():
    "Rotating the motor for 5 seconds"
    with open(data_file, "w") as file:
        file.write("Sensor Reading Medium - Motor Encoder\n")
        Motor1.reset_encoder()
        while True:
            
            for i in range(20):
                """read US sensor reading and motor encoding value"""
                US_reading = US.get_value()
                MediumMotor_reading = Motor1.get_encoder()
                print(str(US_reading) + " " + str(MediumMotor_reading) + "\n")
                file.write(str(US_reading) + " " + str(MediumMotor_reading) + "\n")
                sleep(0.05)
                
            Rotate()
            v = Motor1.get_position()
            if (v >= 90):
                break
            
              
    
    return


try:
    wait_ready_sensors(True)
    #motor1.reset_encoder()
    print("Ready")
    sleep(1)

    collect_data()
    BP.reset_all()
    Motor1.set_dps(0)

            
except :
    print("error")
    BP.reset_all()
    Motor1.set_dps(0)