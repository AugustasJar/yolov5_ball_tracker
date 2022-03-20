import adafruit_bno055
import re
import board
from time import sleep
i2c = board.I2C()

sensor = adafruit_bno055.BNO055_I2C(i2c)


while True:
    data=re.sub('[()]', '', str(sensor.euler)).split(',')[0]
    print(data)
    sleep(0.1)
    
