# import the necessary packages
import numpy as np
import cv2
#from sqlalchemy import true
 
# initialize the HOG descriptor/person detector
# hog = cv2.HOGDescriptor()
# hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

# cv2.startWindowThread()

# # open webcam video stream
# cap = cv2.VideoCapture(0)

# # the output will be written to output.avi
# out = cv2.VideoWriter(
#     'output.avi',
#     cv2.VideoWriter_fourcc(*'MJPG'),
#     15.,
#     (640,480))


class HumanDetector:

    def __init__(self):
        self.Human = False
        self.human_detection_count=0
        self.isDetectHuman = False
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    def Stop(self):
        self.human_detection_count = 0

    def Run(self,img):

        frame = img.copy()
        # resizing for faster detection
        frame = cv2.resize(frame, (640, 480))
        # take middle of frame
        frame = frame[0:480, 200:440]
        # using a greyscale picture, also for faster detection
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        # detect people in the image
        # returns the bounding boxes for the detected objects
        boxes, weights = self.hog.detectMultiScale(frame, winStride=(8,8) )

        boxes = np.array([[x, y, x + w, y + h] for (x, y, w, h) in boxes])

        for (xA, yA, xB, yB) in boxes:
            # display the detected boxes in the colour picture
            cv2.rectangle(frame, (xA, yA), (xB, yB),
                            (0, 255, 0), 2)
            area = (xA-xB)*(yA-yB)  
            print ('area = ', area)
            if area > 22000:
                self.human_detection_count += 1
                print("HumanDetection: true")           
            else:        
                print("HumanDetection: no")  
                

        if self.human_detection_count > 30:
            self.isDetectHuman = True

        output = frame
        return output
       

# print(roll())

# # When everything done, release the capture
# cap.release()
# # and release the output
# out.release()
# # finally, close the window
# cv2.destroyAllWindows()
# cv2.waitKey(1)