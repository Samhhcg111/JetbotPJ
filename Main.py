import numpy as np
import cv2
import time
import threading

from lib.MyCam import MyCam
from lib.Controller import Controller
from lib.GlobalPosDET import GlobalPosDET 
from lib.StageSwitch import StageSwitch
from lib.LaneFollower import LandFollower
from lib.Navigator import Navigator as NV
from lib.ColorDetector import ColorDetector
from lib.human_detection_class import HumanDetector
from lib.HSV_and_Area_Data_Slider import HSV_and_Area_Setting as HSV_DATA
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
dataPath = './data'
# To flip the image, modify the flip_method parameter (0 and 2 are the most common)
print(MyCam.gstreamer_pipeline(flip_method=0))
camera = cv2.VideoCapture(MyCam.gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
if camera.isOpened():
    window_handle = cv2.namedWindow("output", cv2.WINDOW_AUTOSIZE)
    cv2.waitKey(50)
    usage()

    ######## Initialize ##########

    '''
    Fundamental parameters
    '''
    IMG_capture_Error_count = 0
    start_time=0.0
    end_time=0.0
    timedifferent=0.1
    Stop = True

    '''
    Initialize classes
    '''
    robot = Robot()
    controller = Controller()
    LF = LandFollower(robot,controller)
    Stage = StageSwitch(TotalStage=4)
    target_pos=np.array([[5],[5]])
    GP = GlobalPosDET(0,1)
    Navigator = NV(map_path,0,1,4,5)
    CD = ColorDetector()
    HD = HumanDetector()
    StopLineHSV = HSV_DATA(dataPath,'TestMapStopLineHSV')
    TrafficLightHSV = HSV_DATA(dataPath,'TrafficLightHSV')

    '''
    Common setting
    '''
    # Create Sliders for HSV and area setting
    # HSV_and_Area.slider(window_name="output")
    Stage.setStage(1)
    ## StopLine setting ##
    # StopLineHSV.slider(window_name="output")
    StopLineROI = np.array([    [(200, 400), (380, 400), (380, 180), (200,180)]    ])
    IntersectionStopLineROI = np.array([    [(160, 400), (440, 400), (440, 210), (160,210)]    ])
    TrafficLightHSV.setValue([59, 94, 45, 141, 95, 213, 300])

    '''
    Controller setting
    '''
    controller.setStrightParam(B=1,C=1)
    controller.setTurnParam(B=2,C=0)
    
 ########################## main loop #############################  
    while cv2.getWindowProperty("output", 0) >= 0:
        start_time=time.time()
        # Read from camera and display on windows
        ret, img= camera.read()
        if not ret:
            print('[Main] img read error')
            IMG_capture_Error_count+=1
        else : 
            undistort_img=Mycam.undistortion(img)
            perspectiveTransform_img=Mycam.warpPerspective(undistort_img)
            if not ret:
                print("[Main] Cannot read camera frame, exit from program!")
                camera.release()
                cv2.destroyAllWindows()
                break
            if np.array_equal(GP.getPos,target_pos):
                Stop = True
                print('[Main] Mission complete')

            '''
            Stage 1
            '''
            if Stage.isStage(1) and not Stop:
                # Start threads #
                HumandetectThread = threading.Thread(target=HD.Run,args=(undistort_img,))
                HumandetectThread.start()
                StopLineThread = threading.Thread(target=CD.StopLineColorDetector,args = (
                    perspectiveTransform_img, 
                    StopLineROI
                ,None,None,None,StopLineHSV.getValue(),))
                StopLineThread.start()

                '''
                Lane following
                '''
                outputIMG = LF.Run(perspectiveTransform_img, timedifferent, 
                                   right_turning_mode_distance_threshold = 15)
                outputIMG = img
                if LF.right_turn_mode:  # Open loop right turn motion
                    LF.Stop()
                    controller.go_stright(robot, 14)
                    controller.turn(robot, np.deg2rad(70))
                    controller.go_stright(robot, 7.5)

                '''
                Human detection
                '''
                HumandetectThread.join()    #wait for detection
                # outputIMG =HD.detectIMG
                # HD.Run(img)
                if HD.isDetectHuman:
                    HD.do_human_aviodance(Robot=robot)
                    HD.Stop()

                '''
                Stop line detection:
                '''
                StopLineThread.join()   # wait fot Stop Line detection
                # outputIMG = CD.stopLineIMG
                if CD.StopLineColor:
                    print('[Main]Intersection detect')
                    LF.Stop()
                    HD.Stop()
                    Stage.setPause()
                    # Stage.nextStage()

                
            '''
            Stage 2: Calulate paths and go to the position in front of the stop line
            '''
            if Stage.isStage(2) and not Stop:
                if not Navigator.complete:
                    outputIMG=Navigator.CalculatePaths(GP,img,Mycam.camera_matrix,Mycam.dist_coeff)
                else :
                    Navigator.GotoStopPoint(controller=controller,robot=robot)
                    # Stage.setPause()
                    Stage.nextStage()

            '''
            Stage 3: Recalculate paths and wait for the green light
            '''
            if Stage.isStage(3) and not Stop:

                '''
                Stop line navigation
                '''
                stop_line_img,stopLineMask = CD.StopLineColorDetector (
                    perspectiveTransform_img, 
                    ROI =IntersectionStopLineROI,
                    HSV_Data=StopLineHSV.getValue()
                )
                outputIMG = stop_line_img
                if CD.StopLineColor:
                    print('[Main]Stopline detect,recalculate paths')
                    outputIMG=Navigator.ReCalculatePathsWithStopLine(perspectiveTransform_img,stopLineMask)
                    Stage.setPause()

                '''
                Traffic Light detection
                '''
                # Traffic_light_marked_img = CD.TrafficLightDetector(
                #     img,
                #     green_lower = np.array([TrafficLightHSV.H_min, TrafficLightHSV.S_min, TrafficLightHSV.V_min], np.uint8),
                #     green_upper = np.array([TrafficLightHSV.H_max, TrafficLightHSV.S_max, TrafficLightHSV.V_max], np.uint8),
                #     green_area_lower_limit = TrafficLightHSV.Area
                #     )
                # outputIMG = Traffic_light_marked_img

                # if not CD.RedLight and CD.YellowLight:
                # Stage.nextStage()
                
            '''
            Stage 4: go to the next road
            '''
            if Stage.isStage(4) and not Stop:
                    Navigator.GotoEntry(controller=controller,robot=robot)
                    Navigator.Stop()
                    Stage.nextStage()

            '''
            Function testing stage: remember to set the total_stage to the number '5' and use
            the setStage(5) function to active this stage.
            '''
            if Stage.isStage(5) and not Stop:
                stop_line_img,stopLineMask = CD.StopLineColorDetector (
                    perspectiveTransform_img, 
                    ROI = StopLineROI,
                    HSV_Data=StopLineHSV.getValue()
                )
                outputIMG = stop_line_img

            '''
            Basic information showing
            '''
            # Calculate FPS
            fps=int(round(1/timedifferent))
            # R visualize_Ref_frame
            if Key_visualize_Ref:
                UI_undistort_img=undistort_img.copy()
                outputIMG=MyCam.visualize_Ref_frame(UI_undistort_img)
            cv2.putText(outputIMG,str(fps),(20,20),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,255),1,cv2.LINE_AA)
            cv2.imshow("output",outputIMG)
        keyCode = cv2.waitKey(1)

        ############################  Command handle ###############################

        # ESC pressed to quit
        if keyCode%256 == 27:
            StopLineHSV.saveData()
            TrafficLightHSV.saveData()
            print("[Main] Escape hit, closing...")
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
            print('[Main] Force next stage')
            controller.robotStop(robot)
            Stage.nextStage()

        # space capture img
        if keyCode%256 == 32:
            Mycam.takepicture(undistort_img)
            print('[Main] Chese')

        # Calculate time different
        end_time=time.time()
        timedifferent=end_time-start_time
        if LF.right_turn_mode: # Reset dt after the open-loop right turn motion
            timedifferent = 0.1
            LF.right_turn_mode = False

        # Error handle
        if IMG_capture_Error_count >10:
            print('[Main] Too many img_capture error ...')
            break

    camera.release()
    cv2.destroyAllWindows()
else:
    print("[Main] Unable to open camera")