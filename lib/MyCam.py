import cv2
import numpy as np
class MyCam:

    calibrationFile_path='calibration.npz'
    camera_matrix=None
    dist_coeff=None
    Perspective_h=None
    Perspective_w=None
    Perspective_M=None
    img_number=0
    #------Load calibration configuration--------
    def __init__(self,calibrationFile_path):
        if calibrationFile_path is None:
            self.calibrationFile_path='calibration.npz'
        else:
            self.calibrationFile_path=calibrationFile_path
        calibration_data=np.load(calibrationFile_path)
        self.camera_matrix=calibration_data['camera_matrix']
        self.dist_coeff=calibration_data['dist_coeff']
        #----perspectiveTransfer param-----
        corner_dst=np.float32([[144,106],[174,46],[328,111],[299,50]])
        h=400
        w=600
        s=20
        offsety=np.array([[0,1],[0,1],[0,1],[0,1]])
        offsetx=np.array([[1,0],[1,0],[1,0],[1,0]])
        corner_tf=np.float32([[0,4],[0,0],[6,4],[6,0]])
        P=np.mat(corner_tf)*s+np.mat(offsety)*(h-7*s)+np.mat(offsetx)*(w/2)
        self.Perspective_M=cv2.getPerspectiveTransform(corner_dst,np.float32(P))
        self.Perspective_h=h
        self.Perspective_w=w
    #Camera setting
    def gstreamer_pipeline(
        #Available sensor modes:
            #3264x2464 21fps
            #3264x1848 28fps
            #1920x1080 29.9fps
            #1640x1232 29.9fps
            #1280x720 59.9fps
            #1280x720 120fps
        w=1280,    
        h=720,
        display_width=640,
        display_height=480,
        framerate=120,
        flip_method=0,
    ):
        return (
            "nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM), "
            "width=(int)%d, height=(int)%d, "
            "format=(string)NV12, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink drop=True "
            % (
                w,
                h,
                framerate,
                flip_method,
                display_width,
                display_height,
            )
        )

    
    #-------warpPerspective------------
    def warpPerspective(self,img):
        output = cv2.warpPerspective(img,self.Perspective_M,(self.Perspective_w,self.Perspective_h))
        return output
    #-------undistortion---------------
    def undistortion(self,img):
        newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coeff, (img.shape[1], img.shape[0]), 1)
        dst = cv2.undistort(img, self.camera_matrix, self.dist_coeff, None, newCameraMatrix)
        x, y, w, h = roi
        dst = dst[y:y + h, x:x + w]
        return dst
    #------visualize referance frame
    def visualize_Ref_frame(src):
        lines_visualize = src.copy()
        corner_list=[(144,106),(174,46),(328,111),(299,50)]
        frame_color=(255,0,255)
        cv2.line(lines_visualize,corner_list[0],corner_list[1],frame_color,1)
        cv2.line(lines_visualize,corner_list[0],corner_list[2],frame_color,1)
        cv2.line(lines_visualize,corner_list[1],corner_list[3],frame_color,1)
        cv2.line(lines_visualize,corner_list[2],corner_list[3],frame_color,1)
        return lines_visualize
    def takepicture(self,img):
        img_name='./undistortimg'+str(self.img_number)+'.jpg'
        self.img_number+=1
        cv2.imwrite(img_name,img)
    def drawline(frame,u0:int,v0:int,u1:int,v1:int,color,thickness:int):
        img  = frame.copy()
        rows,cols = img.shape[:2]
        if (u1-u0) == 0:
            # print('Debug')
            return img
        dvdu = (v1-v0)/(u1-u0)
        b = v0-dvdu*u0
        U0,V0,U1,V1,vL,vR = None,None,None,None,None,None
        for u in range(0,cols-1,1) :
            v = dvdu*u+b
            if int(v) == v0:
                U0 = u
                V0 = v
            if int(v) == v1:
                U1 = u
                V1 = v
            if v>0 and v<rows:
                vL  = v
                uL = u
                if v1 is not None:
                    vR = v
                    uR = u
        if vL is not None and vR is not None:
            if U0 is None:
                U0 = uL
                V0 = vL
            if U1 is None:
                U1 = uR
                V1 = vR
            cv2.line(img,(int(U0),int(V0)),(int(U1),int(V1)),color,thickness) 
        # cv2.putText(img,str('Line: ')+str(vmax)+str(' , ')+str(vmin),(100,190),cv2.FONT_HERSHEY_PLAIN,1,(255,255,255),1)
        return img
           
    def inRange(img,u:int,v:int):
        rows,cols = img.shape[:2]
        if u>=cols or v>=rows or u<0 or v<0 :
            return False
        return True
