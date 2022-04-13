from tkinter import Frame
from matplotlib.pyplot import axis
import numpy as np
import cv2

class LandFollower:

    ############ Lane Detection Alogrithm #################

    def do_segment(frame):
        # Since an image is a multi-directional array containing the relative intensities of each pixel in the image, we can use frame.shape to return a tuple: [number of rows, number of columns, number of channels] of the dimensions of the frame
        # frame.shape[0] give us the number of rows of pixels the frame has. Since height begins from 0 at the top, the y-coordinate of the bottom of the frame is its height
        height = frame.shape[0]
        width = frame.shape[1]
        # Creates a triangular polygon for the mask defined by three (x, y) coordinates
        polygons = np.array([
                                [(40, 148), (534, 84), (534, 168), (430, 399), (263,399)]
                            ])
        # Creates an image filled with zero intensities with the same dimensions as the frame
        mask = np.zeros_like(frame)
        # Allows the mask to be filled with values of 1 and the other areas to be filled with values of 0
        cv2.fillPoly(mask, polygons, (255, 255, 255))
        # A bitwise and operation between the mask and frame keeps only the triangular area of the frame
        segment = cv2.bitwise_and(frame, mask)
        return segment

    def abs_sobel_thresh(img, thresh_min=0, thresh_max=255):
    
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        
        # Sobel x and y component
        sobel_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0)
        sobel_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1)
        
        # Absolute sobel
        abs_sobel = np.sqrt(sobel_x**2 + sobel_y**2)
        scaled_sobel = np.uint8(255*abs_sobel/np.max(abs_sobel))
        binary_output = np.zeros_like(scaled_sobel)
        binary_output[(scaled_sobel >= thresh_min) & (scaled_sobel <= thresh_max)] = 1

        return binary_output

    def find_line(binary_warped):
        # Take a histogram of the bottom half of the image
        histogram = np.sum(binary_warped[binary_warped.shape[0]//2:,:], axis=0)
        # Find the peak of the left and right halves of the histogram
        # These will be the starting point for the left and right lines
        #midpoint = np.int(histogram.shape[0]/2)
        midpoint = 362
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        # Choose the number of sliding windows
        nwindows = 9
        # Set height of windows
        window_height = np.int((binary_warped.shape[0]-202)/(nwindows))   #only use the bottom half
        # Identify the x and y positions of all nonzero pixels in the image
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        # Current positions to be updated for each window
        leftx_current = leftx_base
        rightx_current = rightx_base
        # Set the width of the windows +/- margin
        margin = 50
        # Set minimum number of pixels found to recenter window
        minpix = 30
        # Create empty lists to receive left and right lane pixel indices
        left_lane_inds = []
        right_lane_inds = []

        # Step through the windows one by one
        for window in range(nwindows):
            # Identify window boundaries in x and y (and right and left)
            win_y_low = binary_warped.shape[0] - (window+1)*window_height
            win_y_high = binary_warped.shape[0] - window*window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
            # Identify the nonzero pixels in x and y within the window
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
            (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
            (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]
            # Append these indices to the lists
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_left_inds) > minpix:
                leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:        
                rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

        # Concatenate the arrays of indices
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        # Extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds] 
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds] 

        # Determine if num of points is enougth for fitting
        len_left = len(leftx)
        len_right = len(rightx)
        if len_left>=3 and len_right>=3:
            lane = True
        else:
            lane = False

        if lane:
            # Fit a second order polynomial to each
            left_fit = np.polyfit(lefty, leftx, 2)
            right_fit = np.polyfit(righty, rightx, 2)

            print('left: ', left_fit)
            #print('\nright: ', right_fit)

            # Create points in lines for plot
            left_line_pts = np.array([[left_fit[0]*y**2 + left_fit[1]*y + left_fit[2], y] for y in range(200,401)], np.int32)
            right_line_pts = np.array([[right_fit[0]*y**2 + right_fit[1]*y + right_fit[2], y] for y in range(200,401)], np.int32)
            center_line_pts = np.array((left_line_pts + right_line_pts)/2, np.int32)

            return left_fit, right_fit, left_line_pts, right_line_pts, center_line_pts

        else:
            return None, None, None, None, None

    def calculate_dx_dy_in_cm(center_line_pts, dl_in_cm=5, pixel_per_cm=8.6, ref_x_coor_in_PTimg=362):
        dl_in_pixel = dl_in_cm*pixel_per_cm
        length = 0
        for index in range(1,200):
            dl = np.sqrt((center_line_pts[-index,0]-center_line_pts[-(index+1),0])**2 + (center_line_pts[-index,1]-center_line_pts[-(index+1),1])**2)
            length += dl
            if (length > dl_in_pixel):
                dx_in_pixel = 400 - center_line_pts[-index,1] 
                dx_in_cm = dx_in_pixel/pixel_per_cm
                dy_in_pixel = ref_x_coor_in_PTimg - center_line_pts[-index,0] 
                dy_in_cm = dy_in_pixel/pixel_per_cm
                return (dx_in_cm, dy_in_cm)


    def __init__(self,Robot,Controller):
        self.controller = Controller
        self.pid_count = 0
        self.robot = Robot
        self.dl_and_right_turn_condition = np.zeros((50,2)) # First column is dt, second column is condition (0 or 1)
        self.right_turn_mode = False

    def Stop(self):
        self.pid_count = 0
        self.robot.left_motor.value  = 0
        self.robot.right_motor.value = 0


    # Forward velocity calulator
    def forward_velocity_calculator(self, Vin):
        M = self.controller.M
        C = self.controller.C
        B = self.controller.B
        X = M.dot(C.dot(Vin)+B)
        return X[0][0]

    # Right turn checker
    def RightTrakingChecker(self, left_fit, dt, velocity, distance_required=20):
        # Remove old data
        self.dl_and_right_turn_condition = np.delete(self.dl_and_right_turn_condition, 0, axis=0)
        # Add new data
        dl = dt*velocity
        if left_fit[0] >= 3e-3:
            self.dl_and_right_turn_condition = np.append(self.dl_and_right_turn_condition, [[dl, 1]], axis=0)
        else:
            self.dl_and_right_turn_condition = np.append(self.dl_and_right_turn_condition, [[dl, 0]], axis=0)
        # Set right turning mode as false as defarlt
        self.right_turn_mode = False
        # Check if right turn is needed
        sum_of_distance = 0
        sum_of_condition_meet = 0
        sum_of_frame = 0
        for i in range(1,50):
            [sum_of_distance, sum_of_condition_meet] = [sum_of_distance, sum_of_condition_meet] + self.dl_and_right_turn_condition[-i]
            if sum_of_distance > distance_required:
                sum_of_frame = i
                break
        if sum_of_frame>8:
            self.right_turn_mode = (sum_of_condition_meet > 0.8*sum_of_frame)
        print(sum_of_frame)
        
    
    #------------start--------------------
    def Run(self, perspectiveTransform_img, dt, right_turning_mode_distance_threshold=20):
        dl_in_cm=15
        binary_img = LandFollower.abs_sobel_thresh(perspectiveTransform_img, thresh_min=35, thresh_max=120)
        binary_img_segment = LandFollower.do_segment(binary_img*255)
        left_fit, right_fit, left_line_pts, right_line_pts, center_line_pts = LandFollower.find_line(binary_img_segment)
        
        # Check if lane exist
        if left_fit is not None:
            binary_img_segment_lane_detection = cv2.polylines(binary_img_segment, [center_line_pts], False, (255,0,0), 1)
            binary_img_segment_lane_detection_left = cv2.polylines(binary_img_segment_lane_detection, [left_line_pts], False, (255,0,0), 1)
            binary_img_segment_lane_detection_left_right = cv2.polylines(binary_img_segment_lane_detection_left, [right_line_pts], False, (255,0,0), 1)
            
            dx_in_cm, dy_in_cm = LandFollower.calculate_dx_dy_in_cm(center_line_pts, dl_in_cm=dl_in_cm)

            outputIMG=binary_img_segment_lane_detection_left_right

            ex = dx_in_cm
            ey = dy_in_cm

            #-------------P I D------------------
            if self.pid_count == 0:
                self.ex_prev = ex
                self.ey_prev = ey
                self.pid_count = 1
            #filter out extreme value
            if np.abs(ey)>=12:  #7
                ey=np.sign(ey)*12
            # Get the feedback
            (vot_left, vot_right) = self.controller.get_feedback(ex, self.ex_prev, ey, self.ey_prev, dt)
 
            # Apply the min vot
            if vot_left <= 0.07: #0.07498
                vot_left = 0.07
            elif vot_left >= 0.12:
                vot_left = 0.12
            if vot_right <= 0.065: #0.07234
                vot_right = 0.065
            elif vot_right >0.12:
                vot_right = 0.12

            # Calculate forward velocity
            velocity = self.forward_velocity_calculator(np.array([[vot_left], [vot_right]]))
            #print(velocity)

            # Check if entering right turning mode is needed
            self.RightTrakingChecker(left_fit, dt, velocity, distance_required=right_turning_mode_distance_threshold)
            if self.right_turn_mode:
                self.dl_and_right_turn_condition = np.zeros((50,2)) # reset
                return binary_img_segment


            #(vot_left, vot_right) = (0.0,0.0)
            
            # update the error
            self.ex_prev = ex
            self.ey_prev = ey
            #------------------------------------
            output=binary_img_segment_lane_detection
            self.robot.left_motor.value=vot_left
            self.robot.right_motor.value=vot_right
            cv2.putText(output,'PID_info: ',(50,50),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,255),1,cv2.LINE_AA)
            cv2.putText(output,'ex: '+str(ex)+' ey: '+str(ey),(50,80),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,255),1,cv2.LINE_AA)
            cv2.putText(output,'vot_lef: '+str(vot_left)+' vot_right: '+str(vot_right),(50,100),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,255),1,cv2.LINE_AA)
            
            return outputIMG

        else:
            self.robot.left_motor.value=0
            self.robot.right_motor.value=0
            print ('lane is not found')
            return perspectiveTransform_img