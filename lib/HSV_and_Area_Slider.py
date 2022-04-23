import cv2

class HSV_and_Area_Setting:

    def __init__(self):
        self.H_min = 0
        self.H_max = 0
        self.S_min = 0
        self.S_max = 0
        self.V_min = 0
        self.V_max = 0
        self.Area = 0


    def slider(self, window_name):

        def update(x):
            self.H_min = cv2.getTrackbarPos('H_min', window_name)
            self.H_max = cv2.getTrackbarPos('H_max', window_name)
            self.S_min = cv2.getTrackbarPos('H_min', window_name)
            self.S_max = cv2.getTrackbarPos('H_max', window_name)
            self.V_min = cv2.getTrackbarPos('H_min', window_name)
            self.V_max = cv2.getTrackbarPos('H_max', window_name)
            self.Area  = cv2.getTrackbarPos('Area', window_name)

        cv2.createTrackbar('H_min',window_name,0,180,update)
        cv2.createTrackbar('H_max',window_name,0,180,update)
        cv2.createTrackbar('S_min',window_name,0,255,update)
        cv2.createTrackbar('S_max',window_name,0,255,update)
        cv2.createTrackbar('V_min',window_name,0,255,update)
        cv2.createTrackbar('V_max',window_name,0,255,update)
        cv2.createTrackbar('Area',window_name,0,3000,update)

        cv2.setTrackbarPos('H_min',window_name,0)
        cv2.setTrackbarPos('H_max',window_name,180)
        cv2.setTrackbarPos('S_min',window_name,0)
        cv2.setTrackbarPos('S_max',window_name,255)
        cv2.setTrackbarPos('V_min',window_name,0)
        cv2.setTrackbarPos('V_max',window_name,255)
        cv2.setTrackbarPos('Area',window_name,300)