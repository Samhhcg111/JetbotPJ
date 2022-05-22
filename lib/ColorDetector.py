import numpy as np
import cv2

class ColorDetector:    

    def do_segment (
        frame,
        polygons = np.array([   # Creates a triangular polygon for the mask defined by three (x, y) coordinates
                                [(0, 0), (290, 0), (290, 434), (0, 434)]
                            ])
        ):

            # [number of rows, number of columns, number of channels]
            height = frame.shape[0]
            width = frame.shape[1]

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
        self.StopLineColor = False
        self.YellowLane = False
        self.WhiteLane = False
        self.stopLineIMG = None
        self.lane_color_image = None

    def TrafficLightDetector (
        self,
        imageFrame,
        ROI = np.array([   [(0, 0), (290, 0), (290, 434), (0, 434)]   ]),
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
        segmentedFrame = ColorDetector.do_segment(imageFrame, polygons=ROI) 
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

        # Set all lights is False as defualt
        self.RedLight = False
        self.GreenLingt = False
        self.YellowLight = False
        
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


    def StopLineColorDetector (
        self,
        image,
        ROI = np.array([    [(210, 250), (210, 342), (263, 399), (430, 399), (500,305), (500, 250)]    ]),
        stop_line_color_lower = np.array([170, 60, 111], np.uint8),  # 136, 87, 111
        stop_line_color_upper = np.array([180, 120, 180], np.uint8), # 180, 255, 255
        area_lower_limit = 1500,
        HSV_Data = [170,180,60,120,111,180,1500],
    ):
        '''
        Run Stop line detection

        Args:
            image: source image
            ROI: region of interest
            stop_line_color_lower: HSV lower bound
            stop_line_color_upper: HSV upper bound
            area_lower_limit: critical area
            HSV_Data:1-D list. The easy way to apply HSV bound ex:[H_min,H_max,S_min,S_max,V_min,V_max,Area]
        
        Returns:
            imageFrame: red line detection image
            mask: red line bitwise image
        '''
        if HSV_Data is not None:
            stop_line_color_lower  = np.array([HSV_Data[0], HSV_Data[2], HSV_Data[4]], np.uint8)
            stop_line_color_upper = np.array([HSV_Data[1], HSV_Data[3], HSV_Data[5]], np.uint8)
            area_lower_limit = HSV_Data[6]

        imageFrame = image.copy()
        # Input image
        segmentedFrame = ColorDetector.do_segment(imageFrame, polygons=ROI) 
        hsvFrame = cv2.cvtColor(segmentedFrame, cv2.COLOR_BGR2HSV)

        # Define mask
        mask = cv2.inRange(hsvFrame, stop_line_color_lower, stop_line_color_upper)

        # Morphological Transform, Dilation for each color and bitwise and operator between image frame and mask determine to detect only that particular color.
        kernal = np.ones((5,5), "uint8")
        mask = cv2.dilate(mask, kernal)
        res = cv2.bitwise_and(segmentedFrame, segmentedFrame, mask=mask)

        # Set stop line False as defualt
        self.StopLineColor = False

        #Creating contour to track red color
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if area > area_lower_limit:
                x, y, w, h = cv2.boundingRect(contour)
                imageFrame = cv2.rectangle(imageFrame, (x,y), (x+w, y+h), (0,0,255), 2)
                cv2.putText(imageFrame, "Stop line", (x,y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,255))
                self.StopLineColor = True
        self.stopLineIMG = imageFrame
        return imageFrame,mask


    def LaneColorDetector (
        self,
        image,  # Undistorted image only
        ROI,
        lane_yellow_lower = np.array([5, 30, 220], np.uint8), 
        lane_yellow_upper = np.array([30, 150, 255], np.uint8),
        lane_yellow_area_lower_limit = 100,
        lane_yellow_HSV_Data = [5,30,30,150,220,255,100],
        lane_white_lower = np.array([170, 60, 111], np.uint8),
        lane_white_upper = np.array([180, 120, 180], np.uint8), 
        lane_white_area_lower_limit = 1500,
        lane_white_HSV_Data = [170,180,60,120,111,180,1500]
    ):
        '''
            Indentify yelow lane at first
        '''
        if lane_yellow_HSV_Data is not None:
            lane_yellow_lower  = np.array([lane_yellow_HSV_Data[0], lane_yellow_HSV_Data[2], lane_yellow_HSV_Data[4]], np.uint8)
            lane_yellow_upper  = np.array([lane_yellow_HSV_Data[1], lane_yellow_HSV_Data[3], lane_yellow_HSV_Data[5]], np.uint8)
            lane_yellow_area_lower_limit = lane_yellow_HSV_Data[6]

        imageFrame = image.copy()
        # Input image
        segmentedFrame = ColorDetector.do_segment(imageFrame, polygons=ROI) 
        hsvFrame = cv2.cvtColor(segmentedFrame, cv2.COLOR_BGR2HSV)

        # Define mask
        mask = cv2.inRange(hsvFrame,  lane_yellow_lower, lane_yellow_upper)

        # Morphological Transform, Dilation for each color and bitwise and operator between image frame and mask determine to detect only that particular color.
        kernal = np.ones((5,5), "uint8")
        mask = cv2.dilate(mask, kernal)
        res = cv2.bitwise_and(segmentedFrame, segmentedFrame, mask=mask)

        # Set lane colors False as defualt
        self.YellowLane = False
        self.WhiteLane = False

        #Creating contour to track red color
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if area > lane_yellow_area_lower_limit:
                x, y, w, h = cv2.boundingRect(contour)
                imageFrame = cv2.rectangle(imageFrame, (x,y), (x+w, y+h), (0,0,255), 2)
                cv2.putText(imageFrame, "Yellow lane", (x,y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,255))
                self.YellowLane = True
                self.lane_color_image = imageFrame
                return imageFrame, mask
        # self.lane_color_image = imageFrame
        # return imageFrame,mask

        '''
            If yellow lane is not found, identify white lane
        '''
        # if lane_white_HSV_Data is not None:
        #     lane_white_lower  = np.array([lane_white_HSV_Data[0], lane_white_HSV_Data[2], lane_white_HSV_Data[4]], np.uint8)
        #     lane_white_upper  = np.array([lane_white_HSV_Data[1], lane_white_HSV_Data[3], lane_white_HSV_Data[5]], np.uint8)
        #     lane_white_area_lower_limit = lane_white_HSV_Data[6]

        # # Define mask
        # mask = cv2.inRange(hsvFrame,  lane_white_lower, lane_white_upper)

        # # Morphological Transform, Dilation for each color and bitwise and operator between image frame and mask determine to detect only that particular color.
        # kernal = np.ones((5,5), "uint8")
        # mask = cv2.dilate(mask, kernal)
        # res = cv2.bitwise_and(segmentedFrame, segmentedFrame, mask=mask)

        # # Set lane colors False as defualt
        # self.WhiteLane = False

        # #Creating contour to track red color
        # contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # for pic, contour in enumerate(contours):
        #     area = cv2.contourArea(contour)
        #     if area > lane_white_area_lower_limit:
        #         x, y, w, h = cv2.boundingRect(contour)
        #         imageFrame = cv2.rectangle(imageFrame, (x,y), (x+w, y+h), (0,0,255), 2)
        #         cv2.putText(imageFrame, "White lane", (x,y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,255))
        #         self.WhiteLane = True
        #         self.lane_color_image = imageFrame
        #         return imageFrame, mask

        '''
            Return original image if find nothing
        '''
        self.lane_color_image = imageFrame
        return imageFrame






