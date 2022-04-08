import turtle
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
    
    def __init__(self):
         ## Create velocity and angular velocity object of PID control ##
        self.velocity = PID(0.6,0,0) #1.7*8/15 #0.6
        self.omega = PID(0.06,0,0.04) #0.06
        ## The parameter of the differential drive car ##
        r = 3.3   # wheel radius
        l = 12.3  # distance between wheels
        C_L = 54.52  # Left wheel coef.   (C_L = left wheel angular velocity/input value)
        C_R = 55.254  # Right wheel coef.
        b_L = -2.2687 # Left wheel intercept
        b_R = -2.1796 # right wheel intercept
        ## Car Matrixes ##
        self.M = np.array([[r/2, r/2], [-r/l, r/l]])
        C = np.array([[C_L, 0], [0, C_R]])
        self.B = np.array([[b_L], [b_R]])
        MC = (self.M.dot(C))
        self.MC_inv = np.linalg.inv(MC)

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
    
    def turn(self,Robot,radian):
        deg = radian/np.pi*180
        print('Turn '+str(deg)+' deg')
        if not (radian == 0):
            if radian>np.pi:
                radian = -(2*np.pi-radian)
            w = -3
            if radian < 0 :
                w = -w
            seconds = (abs(radian)*2)/abs(w)
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
        print('Go stright '+str(distance)+' cm')
        v = 20
        if distance < 0 :
            v = -v
        seconds = (distance+1)/abs(v)
        X = np.array([[v],[0]])
        Vot = self.MC_inv.dot(X-self.M.dot(self.B))
        # print('command: going straight')
        Robot.left_motor.value = Vot[0][0]
        Robot.right_motor.value = Vot[1][0]
        time.sleep(seconds)
        Robot.left_motor.value = 0
        Robot.right_motor.value = 0
        # print('command: going straight is done')
    def robotStop(self,Robot):
        Robot.left_motor.value = 0
        Robot.right_motor.value = 0