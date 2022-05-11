import numpy as np
from lib.Controller import Controller

class odometer:

    def __init__(self):
        self.velocity = 0
        self.distance = 0
        self.controller = Controller()


    '''
    Forward speed and travel distance calculator

    Functions:
        self.forward_velocity_calculator:   Calculate current forward velocity
        self.travel_distance:  Read the current travel distance
        self.reset_odometer:   Reset the travel distance to zero  
    
    Args:
        Vin:  Motor input value in 2*1 np array; that is, np.array([[vot_left], [vot_right]])
        dt:   Time difference between two frames
    '''

    def forward_velocity_calculator(self, Vin):
        M = self.controller.M
        C = self.controller.C
        B = self.controller.B
        X = M.dot(C.dot(Vin)+B)
        self.velocity = X[0][0]
        return self.velocity

    def odometer(self, Vin, dt):
        self.forward_velocity_calculator(Vin)
        self.distance += self.velocity*dt
        return self.distance

    def reset_odometer(self):
        self.distance = 0