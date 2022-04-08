import numpy as np
import cv2

class ColorDetector:    

    def do_segment(frame):
            # Since an image is a multi-directional array containing the relative intensities of each pixel in the image, we can use frame.shape to return a tuple: [number of rows, number of columns, number of channels] of the dimensions of the frame
            # frame.shape[0] give us the number of rows of pixels the frame has. Since height begins from 0 at the top, the y-coordinate of the bottom of the frame is its height
            height = frame.shape[0]
            width = frame.shape[1]
            # Creates a triangular polygon for the mask defined by three (x, y) coordinates
            polygons = np.array([
                                    [(0, 0), (290, 0), (290, 434), (0, 434)]
                                ])
            # Creates an image filled with zero intensities with the same dimensions as the frame
            mask = np.zeros_like(frame)
            # Allows the mask to be filled with values of 1 and the other areas to be filled with values of 0
            cv2.fillPoly(mask, polygons, (255, 255, 255))
            # A bitwise and operation between the mask and frame keeps only the triangular area of the frame
            segment = cv2.bitwise_and(frame, mask)
            return segment


    def __init__(self):
        # Traffic light boolean
        self.RedLight = False
        self.GreenLingt = False
        self.YellowLight = False
    

    def TrafficLightDetector (
        self,
        imageFrame,
        red_lower = np.array([160, 60, 180], np.uint8),  # 136, 87, 111
        red_upper = np.array([180, 110, 255], np.uint8), # 180, 255, 255
        red_area_lower_limit = 300,
        green_lower = np.array([75, 52, 110], np.uint8),  # 25, 52, 72
        green_upper = np.array([85, 200, 200], np.uint8), # 102, 255, 255
        green_area_lower_limit = 300,
        yellow_lower = np.array([5, 30, 220], np.uint8), # 5, 30, 180
        yellow_upper = np.array([30, 150, 255], np.uint8),
        yellow_area_lower_limit = 300
    ):

        # Input image
        segmentedFrame = ColorDetector.do_segment(imageFrame) 
        hsvFrame = cv2.cvtColor(segmentedFrame, cv2.COLOR_BGR2HSV)

        # Define mask
        red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)
        green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)
        yellow_mask = cv2.inRange(hsvFrame, yellow_lower, yellow_upper)
        
        # Morphological Transform, Dilation for each color and bitwise and operator between image frame and mask determine to detect only that particular color.
        kernal = np.ones((5,5), "uint8")
        red_mask = cv2.dilate(red_mask, kernal)
        res_red = cv2.bitwise_and(segmentedFrame, segmentedFrame, mask=red_mask)
        green_mask = cv2.dilate(green_mask, kernal)
        res_green = cv2.bitwise_and(segmentedFrame, segmentedFrame, mask=green_mask)
        yellow_mask = cv2.dilate(yellow_mask, kernal)
        res_yellow = cv2.bitwise_and(segmentedFrame, segmentedFrame, mask=yellow_mask)
        
        #Creating contour to track red color
        red_contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for pic, red_contour in enumerate(red_contours):
            area = cv2.contourArea(red_contour)
            if area > red_area_lower_limit:
                x, y, w, h = cv2.boundingRect(red_contour)
                imageFrame = cv2.rectangle(imageFrame, (x,y), (x+w, y+h), (0,0,255), 2)
                cv2.putText(imageFrame, "Red", (x,y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,255))
                self.RedLight = True
        
        #Creating contour to track green color
        green_contours, hierarchy = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for pic, green_contour in enumerate(green_contours):
            area = cv2.contourArea(green_contour)
            if area > green_area_lower_limit:
                x, y, w, h = cv2.boundingRect(green_contour)
                imageFrame = cv2.rectangle(imageFrame, (x,y), (x+w, y+h), (0,255,0), 2)
                cv2.putText(imageFrame, "Green", (x,y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0))
                self.GreenLingt = True

        
        #Creating contour to track yellow color
        yellow_contours, hierarchy = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for pic, yellow_contour in enumerate(yellow_contours):
            area = cv2.contourArea(yellow_contour)
            if area > yellow_area_lower_limit:
                x, y, w, h = cv2.boundingRect(yellow_contour)
                imageFrame = cv2.rectangle(imageFrame, (x,y), (x+w, y+h), (0,255,255), 2)
                cv2.putText(imageFrame, "Yellow", (x,y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,255))
                self.YellowLight = True


        return imageFrame


