

class Controller():

    def __init__(self,en_a,en_b,L_pin,R_pin):
        self.en_a = en_a
        self.en_b = en_b
        self.L_pin = L_pin
        self.R_pin = R_pin

    def L_speed(self, value):
        # motor control to set left motor speed
        return None

    def R_speed(self,value):
        # motor control to set right motor speed
        return None

