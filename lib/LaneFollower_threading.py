import numpy as np
import cv2
from lib.odometer import odometer
import threading

class LaneFollower_threading:
    def find_line_left_threading(
        binary_warped, 
        midpoint = 300, 
        nwindows = 9, 
        margin = 50, 
        minpix = 30):
        '''
        Funciton: find_line
        Args:
            binary_warped: Sobel filtered and segmented image (should be binary image)
            midpoint:  The x axis of the jetbot
            nwindows:  Choose the number of sliding windows
            margin:    Set the width of the windows +/- margin
            minpix:    Create empty lists to receive left and right lane pixel indices
        '''

        '''
        Comment can be found in function find_line
        '''
        histogram = np.sum(binary_warped[binary_warped.shape[0]//2:,:], axis=0)
        leftx_base = np.argmax(histogram[:midpoint])

        window_height = np.int((binary_warped.shape[0]-202)/(nwindows))   
        
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        leftx_current = leftx_base

        left_lane_inds = []
        
        for window in range(nwindows):
            win_y_low = binary_warped.shape[0] - (window+1)*window_height
            win_y_high = binary_warped.shape[0] - window*window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
            (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
            left_lane_inds.append(good_left_inds)
            if len(good_left_inds) > minpix:
                leftx_current = np.int(np.mean(nonzerox[good_left_inds]))

        
        left_lane_inds = np.concatenate(left_lane_inds)

        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]

        len_left = len(leftx)

    def find_line_right_threading(
        binary_warped, 
        midpoint = 300, 
        nwindows = 9, 
        margin = 50, 
        minpix = 30):

        histogram = np.sum(binary_warped[binary_warped.shape[0]//2:,:], axis=0)
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        window_height = np.int((binary_warped.shape[0]-202)/(nwindows))

        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        rightx_current = rightx_base

        right_lane_inds = []

        for window in range(nwindows):
            win_y_low = binary_warped.shape[0] - (window+1)*window_height
            win_y_high = binary_warped.shape[0] - window*window_height
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
            (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]
            if len(good_right_inds) > minpix:        
                rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

        right_lane_inds = np.concatenate(right_lane_inds)

        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds] 

        len_right = len(rightx)