import numpy as np
import cv2
import time
from lib.MyCam import MyCam
from lib.Controller import Controller
from lib.GlobalPosDET import GlobalPosDET 
from lib.StageSwitch import StageSwitch
from lib.LaneFollower import LandFollower
from lib.Navigator import Navigator as NV
from lib.ColorDetector import ColorDetector
from lib.human_detection import HumanDetector
from lib.HSV_and_Area_Slider import HSV_and_Area_Setting

import sys
sys.path.append('../')
from jetbot.robot import Robot

#############commands handler#########
def usage():
    print("\nPress on displayed window:")
    print("[R]      : visualize referance frame")
    print("[T]      : next stage")
    print("[space]  : capture picture")
    print("[ESC]    : quit")
Key_quit = False
Key_visualize_Ref = True


############# start ##############
Mycam = MyCam('calibration.npz')
map_path = 'testmap.csv'
# To flip the image, modify the flip_method parameter (0 and 2 are the most common)
print(MyCam.gstreamer_pipeline(flip_method=0))
camera = cv2.VideoCapture(MyCam.gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
if camera.isOpened():
    window_handle = cv2.namedWindow("output", cv2.WINDOW_AUTOSIZE)
    cv2.waitKey(50)
    usage()
    # Window
    ######## Initialize ##########
    Error_count = 0
    start_time=0.0
    end_time=0.0
    timedifferent=0.1
    robot = Robot()
    controller = Controller()
    LF = LandFollower(robot,controller)
    Stop = True
    Stage = StageSwitch()
    target_pos=np.array([[5],[5]])
    GP = GlobalPosDET(0,1)
    Navigator = NV(map_path,0,1,4,5)
    CD = ColorDetector()
    HD = HumanDetector()
    # Create Sliders for HSV and area setting
    HSV_and_Area = HSV_and_Area_Setting()
    HSV_and_Area.slider(window_name="output")

    Stage.setStage(2)
 ########################## main loop #############################  
    while cv2.getWindowProperty("output", 0) >= 0:
        start_time=time.time()
        # Read from camera and display on windows
        ret, img= camera.read()
        if not ret:
            print('img read error')
            Error_count+=1
        else : 
            undistort_img=Mycam.undistortion(img)
            perspectiveTransform_img=Mycam.warpPerspective(undistort_img)
            if not ret:
                print("Cannot read camera frame, exit from program!")
                camera.release()
                cv2.destroyAllWindows()
                break
            if np.array_equal(GP.getPos,target_pos):
                Stop = True
                print('Mission complete')
                
            if Stage.isStage1() and not Stop:
                # print('do stage 1 task')
                
                # Lane following
                outputIMG = LF.Run(perspectiveTransform_img, timedifferent, 
                                   right_turning_mode_distance_threshold = 15)
                if LF.right_turn_mode:
                    LF.Stop()
                    controller.go_stright(robot, 12)
                    controller.turn(robot, np.deg2rad(85))
                    controller.go_stright(robot, 7.5)
                    LF.right_turn_mode = False


                # if detected human:
                    #do human avoidance
                #else:

                # Stop Line detection
                stop_line_img = CD.StopLineColorDetector (
                    perspectiveTransform_img, 
                    ROI = np.array([    [(263, 250), (263, 399), (430, 399), (430,250)]    ])
                )
                if CD.StopLineColor and Navigator.atIntersection(img,Mycam.camera_matrix,Mycam.dist_coeff, Critical_distance=35):
                    LF.Stop()
                    Stage.nextStage()

                #outputIMG = img

            if Stage.isStage2() and not Stop:

                outputIMG = HD.Run(img)

                # print('do traffic light task')
                # Traffic Light detection
                Traffic_light_marked_img = CD.TrafficLightDetector(
                    img,
                    green_lower = np.array([HSV_and_Area.H_min, HSV_and_Area.S_min, HSV_and_Area.V_min], np.uint8),
                    green_upper = np.array([HSV_and_Area.H_max, HSV_and_Area.S_max, HSV_and_Area.V_max], np.uint8),
                    green_area_lower_limit = HSV_and_Area.Area
                    )
                Red    = CD.RedLight
                Green  = CD.GreenLingt
                Yellow = CD.YellowLight
                # if Green and not Red:
                #     Stage.nextStage()

            if Stage.isStage3() and not Stop:
                # Intersection turn
                outputIMG=Navigator.Run(GP,img,Mycam.camera_matrix,Mycam.dist_coeff)
                if Navigator.complete:
                    crosspath = Navigator.crossPath
                    controller.turn(robot,crosspath[0])
                    controller.go_stright(robot,crosspath[1]+4) # plus turning radius error
                    controller.turn(robot,crosspath[2])
                    controller.go_stright(robot,crosspath[3])
                    Navigator.Stop()
                    Stage.nextStage()

            if Stop :
                # LF.Stop()
                HD.Stop()
            fps=int(round(1/timedifferent))
            # R visualize_Ref_frame
            if Key_visualize_Ref:
                UI_undistort_img=undistort_img.copy()
                outputIMG=MyCam.visualize_Ref_frame(UI_undistort_img)
            cv2.putText(outputIMG,str(fps),(20,20),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,255),1,cv2.LINE_AA)
            cv2.imshow("output",outputIMG)
        keyCode = cv2.waitKey(1)
        if Error_count >10:
            print('Too many error ...')
            break
        ############################  Command handle ###############################
        # ESC pressed to quit
        if keyCode%256 == 27:
            print("Escape hit, closing...")
            break
        # R visualize_Ref_frame
        if keyCode%256 == 114:
            if Key_visualize_Ref:
                Key_visualize_Ref =False
                Stop = False
            else:
                Key_visualize_Ref=True
                controller.robotStop(robot)
                Stop = True
        # T next stage
        if keyCode%256 == 116:
            print('Force next stage')
            controller.robotStop(robot)
            Stage.nextStage()
        # space capture img
        if keyCode%256 == 32:
            Mycam.takepicture(undistort_img)
            print('Chese')
        #--------------------------------------------------------------------
        end_time=time.time()
        timedifferent=end_time-start_time
    camera.release()
    cv2.destroyAllWindows()
else:
    print("Unable to open camera")