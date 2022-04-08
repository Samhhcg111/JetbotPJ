import numpy as np
import cv2
import time
from lib.MyCam import MyCam

#############commands handler#########
def usage():
    print("\nPress on displayed window:")
    print("[ESC]    : save and quit")

############# start ##############
Mycam = MyCam('calibration.npz')
# To flip the image, modify the flip_method parameter (0 and 2 are the most common)
print(MyCam.gstreamer_pipeline(flip_method=0))
camera = cv2.VideoCapture(MyCam.gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
if camera.isOpened():
    window_handle = cv2.namedWindow("output", cv2.WINDOW_AUTOSIZE)
    cv2.waitKey(50)
    usage()
    # Window
    start_time=0.0
    end_time=0.0
    timedifferent=0.1
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp=np.zeros((5*7,3),np.float32)
    objp[:,:2]=np.mgrid[1:6,7:0:-1].T.reshape(-1,2)
    count=0
 ########################## main loop #############################  
    while cv2.getWindowProperty("output", 0) >= 0:
        start_time = time.time()
    
        ret,img = camera.read()
        img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        ret,corners = cv2.findChessboardCorners(img_gray,(5,7),None)
        if ret == True:
            corner2 = cv2.cornerSubPix(img_gray,corners, (11,11), (-1,-1), criteria)
            cv2.drawChessboardCorners(img,(5,7),corner2,ret)

        end_time  = time.time()
        if end_time-start_time ==0:
            fps = 'max'
        else:
            fps = int(1/(end_time-start_time))
        img = cv2.putText(img,str(fps),(50,50),cv2.FONT_HERSHEY_PLAIN,2,(255,255,255),1)
        img = cv2.putText(img,str(count),(300,50),cv2.FONT_HERSHEY_PLAIN,2,(255,255,255),1)
        
        if cv2.waitKey(50)&0xFF == 0x1B:#ord('esc'):
            save_path = 'Vectors.npz'
            np.savez(save_path,rvec=rvec,tvec=tvec)
            break
        retval, rvec, tvec = cv2.solvePnP(objp,corner2,Mycam.camera_matrix,Mycam.dist_coeff)
        cv2.drawFrameAxes(img,Mycam.camera_matrix,Mycam.dist_coeff,rvec,tvec,1)
        cv2.imshow('output',img)
    camera.release()
    cv2.destroyAllWindows()
else:
    print("Unable to open camera")