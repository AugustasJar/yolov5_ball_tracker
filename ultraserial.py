#!/usr/bin/env python3



import serial
from threading import Thread

class Sensor:
    
    def __init__(self, portnum):
        self.portnum = portnum
        
        
    def serialsensor(self):
        ser = serial.Serial(self.portnum, 9600, timeout=1)
        ser.reset_input_buffer()
        
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').rstrip()
                print(line)
            
            
s1 = Sensor('/dev/ttyACM0')

t1 = Thread(target=s1.serialsensor)
t1.start()
