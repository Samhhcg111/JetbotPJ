import time
import numpy as np
from threading import Thread

class odometerThread(Thread):
    '''
    Odometer threading class.Use this class as a thread to accumulate distance or angle automatically
    
    Common use:
        start() : start accumulation
        stop()  : stop accumulation
        reset() : reset odometer
    '''
    def __init__(self,odometer,odometerTarget,timestep=0.1):
        '''
        Attributes:
            odometer: the odometer class object
            odometerTarget: odometer.odometer or odometer.angular_odometer
            timestep:seconds. Time step to accumulate
        '''
        super(odometerThread,self).__init__()
        self.__stop = False
        self.odometer = odometer
        self.Target = odometerTarget
        self.timestep = timestep

    def run(self) :
        dt = 0
        while not self.__stop:
            t0 = time.time()
            robot = self.odometer.controller.robot
            vot_left=robot.left_motor.value
            vot_right=robot.right_motor.value
            Vin = np.array([[vot_left], [vot_right]])
            dt = self.timestep+time.time()-t0
            self.Target(Vin,dt)
            time.sleep(self.timestep)

    def reset(self):
        '''
        Reset odometer
        '''
        self.odometer.reset_odometer()

    def stop(self):
        '''
        Stop thread
        '''
        self.__stop = True

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
        t0 = time.time()
        self.forward_velocity_calculator(Vin)
        dt = dt + time.time()-t0
        self.distance += self.velocity*dt
        return self.distance
    def odometerCalibrate(self, Vin, dt,c):
        '''
        Forward travel distance calculator
        Args:
            Vin:  Motor input value in 2*1 np array; that is, np.array([[vot_left], [vot_right]])
            dt:   Time difference between two frames
            c:   dt = dt * c
        ReturnS
            distance: accumulation distance
        '''
        t0 = time.time()
        self.forward_velocity_calculator(Vin)
        dt = dt + time.time()-t0
        self.distance += self.velocity*dt*c
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
        t0 = time.time()
        self.angular_velocity_calculator(Vin)
        dt = dt + time.time()-t0
        self.orientation += abs(self.angular_velocity)*dt
        return self.orientation

    def reset_odometer(self):
        self.distance = 0
        self.orientation = 0
