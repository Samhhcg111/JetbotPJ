
class odometer:

    def __init__(self,controller):
        '''
        Forward speed and travel distance calculator
        Functions:
            self.forward_velocity_calculator:   Calculate current forward velocity
            self.travel_distance:  Read the current travel distance
            self.reset_odometer:   Reset the travel distance to zero  
        '''
        self.velocity = 0
        self.distance = 0
        self.controller = controller
        self.angular_velocity = 0
        self.orientation = 0
   
    def forward_velocity_calculator(self, Vin):
        '''
        Forward speed calculator
        Returns:
           velocity
        '''
        M = self.controller.M
        C = self.controller.C
        B = self.controller.B
        X = M.dot(C.dot(Vin)+B)
        self.velocity = X[0][0]
        return self.velocity

    def odometer(self, Vin, dt):
        '''
        Forward travel distance calculator
        Args:
            Vin:  Motor input value in 2*1 np array; that is, np.array([[vot_left], [vot_right]])
            dt:   Time difference between two frames
        ReturnS
            distance: accumulation distance
        '''
        self.forward_velocity_calculator(Vin)
        self.distance += self.velocity*dt
        return self.distance
    def angular_velocity_calculator(self, Vin):
        '''
        Angular speed calculator
        Returns:
           angular_velocity
        '''
        M = self.controller.M
        C = self.controller.C
        B = self.controller.B
        X = M.dot(C.dot(Vin)+B)
        self.angular_velocity = X[1][0]
        return self.angular_velocity

    def angular_odometer(self, Vin, dt):
        '''
        An Angular travel calculator
        Args:
            Vin:  Motor input value in 2*1 np array; that is, np.array([[vot_left], [vot_right]])
            dt:   Time difference between two frames
        ReturnS
            orientation: accumulation angle
        '''
        self.angular_velocity(Vin)
        self.orientation += self.angular_velocity*dt
        return self.orientation

    def reset_odometer(self):
        self.distance = 0
        self.orientation = 0