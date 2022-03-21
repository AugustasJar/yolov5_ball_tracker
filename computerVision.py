from imutils.video import VideoStream
import numpy as np
import cv2 #pip install opencv-contrib-python
import imutils
import time
import math
from threading import Thread
import serial
from threading import Thread
from time import sleep

import adafruit_bno055
import board
import re

class computerVision:

    def __init__(self, ranges):
        self.vs = VideoStream(0)
        self.vs.start()
        time.sleep(1.0)
        self.minimum_countour_area = 300
        self.ranges = ranges
        self.gaussian_blur_kernel = (7, 7)
        self.image_width = 240
        self.Ki_memory = 5
        self.servo_angle = 20
        self.servo_limit = 0
        self.active = True
        self.I = 0
        self.default_turn_left = True
        self.collision_distance = 6


        #TESTING
        i2c = board.I2C()
        self.sensor = adafruit_bno055.BNO055_I2C(i2c)
        self.startingPosition = imu=int(float(re.sub('[()]', '', str(self.sensor.euler)).split(',')[0]))
        #TESTING
        self.Kp = 10
        self.Ki = 0.1
        self.Kd = 0
        self.detections = []
        self.pastInputs = []
        self.s1 = Sensor('/dev/ttyACM0')
        
        self.control_logic()
        
        

#    def start(self):
#        Thread(target=self.videoLoop(),args=()).start()
#        return self


#    def stop(self):
#        self.active = False
#
    def videoLoop(self):
        # grab the current frame
        frame = self.vs.read()
        frame = imutils.resize(frame, width=self.image_width)
        mask = self.create_mask(frame, self.ranges)

        cnts = self.findCountours(mask)
        frame = self.displayDetection(frame, cnts)
        self.addDetections(cnts)
        #cv2.imshow("Frame", frame)
        #key = cv2.waitKey(10) & 0xFF
        if (len(self.detections) > 0):
            y= 9999
            for detection in self.detections:
                if (detection[1] < y):
                    y = detection[0]
                    current_x = detection[0]/240
                    current_y = detection[1]
            if (current_x is not None):
               # print(current_x/290)

               if (len(self.pastInputs) < self.Ki_memory):
                   self.pastInputs.insert(0, current_x)
               else:
                   self.pastInputs.pop()
                   self.pastInputs.insert(0, current_x)
                
               return (current_x,current_y)
        else:
            return None


    def control_logic(self):
        while self.active:
            #gets position of the nearest ball
            nearest_ball_position = self.videoLoop()
            #checks if there is no position - no ball found.
            detects_target = False if nearest_ball_position == None else True
            #a test, ignore
            position = self.getPosition()
            print(position)
            #sensor array holds all the sensor values
            #sensor_array = self.s1.serialSensor()
            #a test ,ignore
            sensor_array = [111,111,111]
            #print(sensor_array)
            #collision detected function proccesses the sensor array and outputs a boolean value based on
            #if there is a collision or not
            if self.collisionDetected(sensor_array):
                #logic of the threat is on the left
                if sensor_array[0]:
                    self.setPWM(1)
                    time.sleep(1)
                    # to prevent banging againts wall it switches the direction of turning ( to be implemented better )
                    self.default_turn_left = not self.default_turn_left
                #logic of the threat is on the right
                elif sensor_array(1):
                    self.setPWM(0)
                    time.sleep(1)
                     # to prevent banging againts wall it switches the direction of turning ( to be implemented better )
                    self.default_turn_left = not self.default_turn_left
                #logic if the threat is straight ahead
                elif sensor_array[2]:
                    #go backwards
                    print("going backwards")
            #logic for no imminent collision
            else:
                if not detects_target:
                    #logic for not banging againts the wall
                    if self.default_turn_left:
                        self.setPWM(0.75)
                    else:
                        self.setPWM(0.75)
                #if ball is found do PID
                else:
                    if (nearest_ball_position[1] > 100):
                        self.setPWM(nearest_ball_position[0])
                    else:
                        print("grab ball")
    #proccesses sensor_array to give a bool value if there will be a collision        
    def collisionDetected(self,sensor_array):
        for value in sensor_array:
            if value == '':
                continue
            if int(value) < self.collision_distance:
                return True
        return False
    #uses IMU to find rotational position
    def getPosition(self):
        try:
            imu=int(float(re.sub('[()]', '', str(self.sensor.euler)).split(',')[0]))
            return imu
        except:
            return None
    #PID control as well as motor code ( in the future)    
    def setPWM(self,error):
        error -= 0.5
        correction = 0
        correction+= 10*self.Proportional(0,error)
        if (correction > 49):
            correction + 49
        # correction+= self.integral(0,error)
        # correction+= self.derivative(0,error,)
        #let pwm be from 0 to 100
        pwm_L = 50 + correction
        pwm_R = 50 - correction
        # print("error: ", error, " | correction: ", correction)
        print("error: " , error, " L: ", pwm_L, " R: ", pwm_R)

    # proprecesses the image based on a hsv range (LowerBounds, upperBounds)
    def create_mask(self, image, ranges):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        blurred = cv2.GaussianBlur(hsv_image, self.gaussian_blur_kernel, 0)
        image = None
        for range in ranges:
            mask = cv2.inRange(blurred, range[0], range[1])
            mask = mask
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=5)
            if (image is not None):
                image = cv2.add(image, mask)
            else:
                image = mask
        return image

    # dumb contour detection, need to add template matching or something.
    def findCountours(self, mask):
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        l = list()
        for c in cnts:
            area = cv2.contourArea(c)
            if area > 300:
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                if math.pi * pow(int(radius), 2) - area < area * 0.90:
                    l.append(c)
        return l

    

    # displays contours on the screen
    def displayDetection(self, frame, cnts):
        if len(cnts) > 0:
            for c in cnts:
                if (cv2.contourArea(c) > 300):
                    ((x, y), radius) = cv2.minEnclosingCircle(c)
                    M = cv2.moments(c)
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                    # cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
                    cv2.circle(frame, center, 5, (0, 0, 255), -1)
        return frame
    #adds detections to a list in the class, used for finding the nearest ball
    def addDetections(self, cnts):
        l = []
        if len(cnts) > 0:
            for c in cnts:
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                l.append(center)
        self.detections = l

    def Proportional(self, SP, PV):
        error = SP - PV
        return self.Kp * error

    def integral(self, SP, PV):
        error = SP - PV
        # self.integral_v+=error*0.1
        if (self.I < 10):
            self.I += 0.1*error
        # correction = 0
        # for e in self.pastInputs:
        #     if (abs(self.servo_angle) < abs(self.servo_limit)):
        #         correction += e*0.1
        return -self.Ki * self.I

    def derivative(self, SP, PV, dt):
        error1 = SP - PV
        error0 = self.pastInputs[0]
        return self.Kd*(error1 - error0) / dt
    
class Sensor:
    
    def __init__(self, portnum):
        self.portnum = portnum
        
        self.expectedSetCount = 0
        self.timeout = 5
       
        
    def serialSensor(self):
        ser = serial.Serial(self.portnum, 9600, timeout=1)
        ser.reset_input_buffer()
        skips = 0
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').rstrip()
                self.sensor_array=line.split('@')
                if len(self.sensor_array) == self.expectedSetCount:
                    return self.sensor_array
                else:
                    if skips == 5:
                        return self.sensor_array
                    else:
                        continue
                        
                

def IMU():
    
    i2c = board.I2C()
    sensor = adafruit_bno055.BNO055_I2C(i2c)
    
    while True:
        try:
            imu=int(float(re.sub('[()]', '', str(sensor.euler)).split(',')[0]))
            return imu
        except:
            print("Error Retrieving IMU Data")

