from utils.brick import EV3UltrasonicSensor, wait_ready_sensors
from time import sleep

data_file = "Final_Project_Test_CSVs/US_Empty_Map.csv"

US = EV3UltrasonicSensor(1)

wait_ready_sensors(True)
sleep(2)

def collect_US_data():
    with open(data_file, "w") as file:
        file.write("Distance\n")
        for i in range(300):
            data = US.get_value()
            print(data)
            file.write(str(data) + "\n")
            sleep(0.05)
    return

collect_US_data()