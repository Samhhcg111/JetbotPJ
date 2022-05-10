import math
import numpy as np
import time
class PID:
    def __init__(self, P, I, D):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.Pterm = 0
        self.Iterm = 0
        self.Dterm = 0
    def output(self, error, error_prev, dt):
        self.Pterm =  self.Kp*error
        self.Iterm += self.Ki*error*dt
        self.Dterm = self.Kd*(error - error_prev)/dt
        return (self.Pterm + self.Iterm + self.Dterm)
class Controller:
    '''
    This is the basic control class for jetbot.
    '''
    def __init__(self):
         ## Create velocity and angular velocity object of PID control ##
        self.velocity = PID(0.6,0,0) #1.7*8/15 #0.6
        self.omega = PID(0.10,0,0.00) # suggest 0.08~0.10
        ## The parameter of the differential drive car ##
        r = 3.3   # wheel radius
        l = 12.3  # distance between wheels
        C_L = 54.52  # Left wheel coef.   (C_L = left wheel angular velocity/input value)
        C_R = 55.254  # Right wheel coef.
        b_L = -2.2687 # Left wheel intercept
        b_R = -2.1796 # right wheel intercept
        ## Car Matrixes ##
        self.M = np.array([[r/2, r/2], [-r/l, r/l]])
        self.C = np.array([[C_L, 0], [0, C_R]])
        self.B = np.array([[b_L], [b_R]])
        MC = (self.M.dot(self.C))
        self.MC_inv = np.linalg.inv(MC)
        self.__TurnParam = [0,2,0]
        self.__StrightParam = [0,1,1]
    def get_feedback(self, ex, ex_prev, ey, ey_prev, dt):
        # Calculate the feedback
        v = self.velocity.output(ex, ex_prev, dt)
        w = self.omega.output(ey, ey_prev, dt)
        X = np.array([[v], [w]])
        Vot_in = self.MC_inv.dot(X-self.M.dot(self.B))

        # Get the feedback
        vot_left = Vot_in[0][0]
        vot_right = Vot_in[1][0]
        return (vot_left, vot_right)

    def calculate_dy_in_cm(center_line_pts, dx_in_cm=10, pixel_per_cm=8.6, ref_x_coor_in_PTimg=362):
        dx_in_pixel = dx_in_cm*pixel_per_cm
        dy_in_pixel = ref_x_coor_in_PTimg - center_line_pts[int(-dx_in_pixel),0] 
        dy_in_cm = dy_in_pixel/pixel_per_cm
        return(dy_in_cm)

    def setTurnParam(self,A=0,B=2,C=0):
        '''
        Set the turning curve parameters of function:
        waiting seconds =( A*radians^2 + B*radian + c )/ AngularVelocity
        
        Args:
            A: quadratic coefficent
            B: linear coefficent
            C: constant coefficent
        '''
        self.__TurnParam = [A,B,C]

    def setStrightParam(self,A=0,B=1,C=1):
        '''
        Set the 'go stright' curve parameters of function:
        waiting seconds = ( A*distance^2 + B*distance + c ) / Velocity
        
        Args:
            A: quadratic coefficent
            B: linear coefficent
            C: constant coefficent
        '''
        self.__StrightParam = [A,B,C]

    def turn(self,Robot,radian,radianbias:float=0):
        '''
        Open-loop control.Turning at constant angular speed in 'least' specific time toward target orientation.

        Args:
            Robot: robot object to control.
            radian: Target clockwise orientation in radians unit, range from -2pi~2pi.
            radianbias: prolong or shorten the turning time.
        '''
        A,B,C = self.__TurnParam
        deg = radian/np.pi*180
        print('Turn '+str(deg)+' deg')
        if not (radian == 0):
            turn_radian = radian
            w = -3
            if abs(radian)>np.pi:
                turn_radian = -(2*np.pi-abs(radian))
            if turn_radian < 0 :
                w = -w
            r = abs(turn_radian)
            seconds = (r*r*A+r*B+C+radianbias)/abs(w)
            if seconds <0:
                seconds = 0
            X = np.array([[0],[w]])
            Vot = self.MC_inv.dot(X-self.M.dot(self.B))
            # print('command: turning')
            Robot.left_motor.value = Vot[0][0]
            Robot.right_motor.value = Vot[1][0]
            time.sleep(seconds)
            Robot.left_motor.value = 0
            Robot.right_motor.value = 0
            # print('command: turning is done')
    def go_stright(self,Robot,distance):
        '''
        Open-loop control.Move forward at constant speed in specific time.

        Args:
            Robot: robot object to control.
            distance: unit [cm]
        '''
        print('Go stright '+str(distance)+' cm')
        A,B,C = self.__StrightParam
        v = 20
        if distance < 0 :
            v = -v
        d = distance
        seconds = (d*d*A+d*B+C)/abs(v)
        X = np.array([[v],[0]])
        Vot = self.MC_inv.dot(X-self.M.dot(self.B))
        # print('command: going straight')
        Robot.left_motor.value = Vot[0][0]
        Robot.right_motor.value = Vot[1][0]
        time.sleep(seconds)
        Robot.left_motor.value = 0
        Robot.right_motor.value = 0
        # print('command: going straight is done')

    def Od_turn(self,Robot,odometer,radian):
        '''
        Using odometer to turn toward a specific orientation
        '''
        print('Turn(Odometer) '+str(math.degrees(radian))+' deg')
        if not (radian == 0):
            turn_radian = radian
            w = -3
            if abs(radian)>np.pi:
                turn_radian = -(2*np.pi-abs(radian))
            if turn_radian < 0 :
                w = -w
            X = np.array([[0],[w]])
            Vot = self.MC_inv.dot(X-self.M.dot(self.B))
            while odometer.radian<radian:
                Robot.left_motor.value = Vot[0][0]
                Robot.right_motor.value = Vot[1][0]
            Robot.left_motor.value = 0
            Robot.right_motor.value = 0
        
    def Od_go_stright(self,Robot,odometer,distance):
        '''
        Using odometer to go to a specific distance
        '''
        print('Go stright(Odometer) '+str(distance)+' cm')
        V=20
        if distance < 0 :
            v = -v
        X = np.array([[v],[0]])
        Vot = self.MC_inv.dot(X-self.M.dot(self.B))
        while odometer.distance <distance:
            Robot.left_motor.value = Vot[0][0]
            Robot.right_motor.value = Vot[1][0]
        Robot.left_motor.value = 0
        Robot.right_motor.value = 0

    def robotStop(self,Robot):
        Robot.left_motor.value = 0
        Robot.right_motor.value = 0