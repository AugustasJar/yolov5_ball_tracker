import adafruit_bno055
import board
import re
from time import sleep


def IMU():
    
    i2c = board.I2C()
    sensor = adafruit_bno055.BNO055_I2C(i2c)
    
    global imu
    
    while True:
        try:
            imu=int(float(re.sub('[()]', '', str(sensor.euler)).split(',')[0]))
            print(imu)
        except:
            print("Error Retrieving IMU Data")

IMU()
