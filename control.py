import math
import time
import matplotlib.pyplot as plt
import numpy as np
import random

class Control:

    def __init__(self, Kp = 0, Ki = 0, Kd = 0, Ki_memory = 5, integral_clamp = 50):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Ki_memory = Ki_memory
        self.pastInputs = list()
        self.end = -1
        self.start = 0
        self.RMSE = 0
        self.integral_clamp = integral_clamp
        self.servo_limit = 20 # degrees
        self.speed = 1
        self.servo_angle = 0
        self.wheel_base = .2
        self.predicted_physical_error = 0
        self.predicted_physical_correction = 0
        self.errors = []
        self.corrections = []
        self.position = []
        self.reset_time = 1
        self.integral_v = 0

    def loop(self):
        error = 0
        for i in range(40):
            self.start = time.time()

            error +=sense(error, 0.9,1,1.5)
            self.errors.append(error)

            self.predicted_physical_error = error*10*math.exp(-1*abs(error)) - self.predicted_physical_correction

            self.position.append(self.predicted_physical_error)
            correction = self._correction(self.predicted_physical_error)

            self.corrections.append(correction)

            error+= correction
            proposed_servo_angle = correction*100

            if (abs(proposed_servo_angle) < 20):
                self.servo_angle = proposed_servo_angle
            else:
                self.speed = 20/proposed_servo_angle
                if ( proposed_servo_angle > 0):
                    self.servo_angle = 20
                else:
                    self.servo_angle = -20

            if (len(self.pastInputs) < self.Ki_memory):
                self.pastInputs.insert(0,error)
            else:
                self.pastInputs.pop()
                self.pastInputs.insert(0, error)

            self.end = time.time()
            self.RMSE += math.sqrt(math.pow(error,2))

            print("error:", error,
                  # "| change in error:",delta_error,
                  "| correction:", correction,
                  "| servo angle: ", self.servo_angle,
                  "| speed:", self.speed,
                  "ppe", self.predicted_physical_error,
                  "ppc", self.getPhysicalCorrection())

            time.sleep(0.1)
        self.RMSE /= 500
        plt.plot(self.errors,'--r',self.corrections, '--b',self.position, '--g')
        plt.show()


    def _correction(self,error, P = True, I = True, D = True):
        correction = 0
        correction += self.Proportional(0, error)
        correction += self.integral(0, error)
        if (self.end != -1):
            correction += self.derivative(0, error, self.end - self.start)
        return correction

    def pulseResponse(self,magnitude):
        position = 0
        for i in range(100):
            if (i==1):
                self.predicted_physical_error = magnitude
            else:
                self.predicted_physical_error += self.predicted_physical_correction

            error = (self.predicted_physical_error*1024)
            if (len(self.pastInputs) < self.Ki_memory):
                self.pastInputs.insert(0,error)
            else:
                self.pastInputs.pop()
                self.pastInputs.insert(0, error)

            correction = 0
            correction+= self.Proportional(0,error)
            correction+=self.integral(0,error)
            if (i!=0):
                correction+=self.derivative(0,error,0.1)

            proposed_servo_angle = correction/1024

            if (proposed_servo_angle > 20):
                self.servo_angle = 20
                self.speed = 0.5
            elif (proposed_servo_angle < -20):
                self.servo_angle = -20
                self.speed = 0.5
            else:
                self.servo_angle = proposed_servo_angle
                self.speed = 1

            PWM_input = math.floor(self.servo_angle*4.5) + 90

            self.predicted_physical_correction = self.getPhysicalCorrection()


            print("error: ", self.predicted_physical_error,
                  "| servo_angle: ", self.servo_angle,
                  "| correction: ", self.predicted_physical_correction,
                  "| PWM input:", PWM_input)
            position = self.predicted_physical_correction + self.predicted_physical_error
            self.position.append(position)
            self.errors.append(self.predicted_physical_error)
            self.corrections.append(self.predicted_physical_correction)

        title = 'Kp = {Kp} | Ki = {Ki} | Kd = {Kd}'.format(Kp=self.Kp, Ki=self.Ki, Kd=self.Kd)
        plt.title(title)
        plt.plot(self.errors, '--r',label= "Process variable")
        plt.plot( self.corrections, '--g', label ="correction")
        plt.plot(self.position,'--b', label = "position")
        plt.xlabel("time")
        plt.legend()
        plt.show()

    def SineResponse(self, magnitude):
        position = 0
        noise = 0
        for i in range(150):
            noise = sense(noise,0.8,1,1)
            print(noise)
            low_f_sine = magnitude*math.sin(i/10)/(i/3+1)
            high_f_sine = (magnitude/30)*math.sin(i) + noise*2
            self.predicted_physical_error += self.predicted_physical_correction + low_f_sine + high_f_sine
            error = (self.predicted_physical_error*1024)
            if (len(self.pastInputs) < self.Ki_memory):
                self.pastInputs.insert(0,error)
            else:
                self.pastInputs.pop()
                self.pastInputs.insert(0, error)

            correction = 0
            correction+= self.Proportional(0,error)
            correction+=self.integral(0,error)
            if (i!=0):
                self.end = time.time()
                dt = self.end - self.start
                correction+=self.derivative(0,error,0.11)

            proposed_servo_angle = correction/1024

            if (proposed_servo_angle > 20):
                self.servo_angle = 20
                self.speed = 0.5
            elif (proposed_servo_angle < -20):
                self.servo_angle = -20
                self.speed = 0.5
            else:
                self.servo_angle = proposed_servo_angle
                self.speed = 1

            PWM_input = math.floor(self.servo_angle*4.5) + 90

            self.predicted_physical_correction = self.getPhysicalCorrection()
            self.start = time.time()

            print("error: ", self.predicted_physical_error,
                  "| servo_angle: ", self.servo_angle,
                  "| correction: ", self.predicted_physical_correction,
                  "| PWM input:", PWM_input)
            position = self.predicted_physical_error + self.predicted_physical_correction
            self.position.append(position)
            self.errors.append(self.predicted_physical_error)
            self.corrections.append(self.predicted_physical_correction)

        title = 'Kp = {Kp} | Ki = {Ki} | Kd = {Kd}'.format(Kp=self.Kp, Ki=self.Ki, Kd=self.Kd)
        plt.title(title)
        plt.plot(self.errors, '--r',label= "Process variable")
        plt.plot( self.corrections, '--g', label ="correction")
        plt.plot(self.position, '--b', label="relative position")
        plt.xlabel("time")
        plt.legend()
        plt.show()

    def stepResponse(self, magnitude):
        position = 0
        for i in range(50):
            self.end = time.time()
            self.predicted_physical_error +=  magnitude + self.predicted_physical_correction
            error = (self.predicted_physical_error * 1024)

            if (len(self.pastInputs) < self.Ki_memory):
                self.pastInputs.insert(0, error)
            else:
                self.pastInputs.pop()
                self.pastInputs.insert(0, error)

            correction = 0
            correction += self.Proportional(0, error)
            correction += self.integral(0, error)
            if (i != 0):
                self.end = time.time()
                correction += self.derivative(0, error, 0.11)

            proposed_servo_angle = correction / 1024

            if (proposed_servo_angle > 20):
                self.servo_angle = 20
                self.speed = 0.5
            elif (proposed_servo_angle < -20):
                self.servo_angle = -20
                self.speed = 0.5
            else:
                self.servo_angle = proposed_servo_angle
                self.speed = 1

            PWM_input = math.floor(self.servo_angle * 4.5) + 90

            self.predicted_physical_correction = self.getPhysicalCorrection()
            self.start = time.time()
            print("error: ", self.predicted_physical_error,
                  "| servo_angle: ", self.servo_angle,
                  "| correction: ", self.predicted_physical_correction,
                  "| PWM input:", PWM_input)
            position = self.predicted_physical_error + self.predicted_physical_correction
            self.position.append(position)
            self.errors.append(self.predicted_physical_error)
            self.corrections.append(self.predicted_physical_correction)

        title = 'Kp = {Kp} | Ki = {Ki} | Kd = {Kd}'.format(Kp=self.Kp, Ki=self.Ki, Kd=self.Kd)
        plt.title(title)
        plt.plot(self.errors, '--r', label="Process variable")
        plt.plot(self.corrections, '--g', label="correction")
        plt.plot(self.position, '--b', label="relative position")
        plt.xlabel("time")
        plt.legend()
        plt.show()

    def Proportional(self, SP, PV):
        error = SP - PV
        return self.Kp * error

    def integral(self, SP, PV):
        error = SP - PV
        # self.integral_v+=error*0.1
        correction = 0
        for e in self.pastInputs:
            if (abs(self.servo_angle) < abs(self.servo_limit)):
                correction += e*0.1
        return -self.Ki * correction

    def derivative(self, SP, PV, dt):
        error1 = SP - PV
        error0 = self.pastInputs[0]
        return self.Kd*(error1 - error0) / dt

    def getPhysicalCorrection(self):
        angle = (self.servo_angle*math.pi)/180
        if (angle == 0):
            d = 0
        else:
            d = self.wheel_base*math.sin(angle)
        # print("angle: ", angle, "displacement: ", d)
        return 30*d