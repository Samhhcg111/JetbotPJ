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
map_path = 'map.csv'
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
    controller = Controller(Robot=robot)
    CD = ColorDetector()
    LF = LandFollower(Controller=controller, ColorDetector=CD)
    Stage = StageSwitch(TotalStage = 4)
    target_pos=np.array([[5],[5]])
    GP = GlobalPosDET(2,0)
    Navigator = NV(map_path,0,0,6,6)
    HD = HumanDetector(controller=controller)
    StopLineHSV = HSV_DATA(dataPath,'TestMapStopLineHSV')
    TrafficLightHSV_green = HSV_DATA(dataPath,'TrafficLightHSV')
    TrafficLightHSV_Red = HSV_DATA(dataPath,'TrafficLightHSVRed')
    LaneYellowHSV = HSV_DATA(dataPath, 'LaneYellowHSV')
    frame = 0

    '''
    Common setting
    '''
    # Create sliders for lane yellow detection setting
    LaneYellowHSV.setValue([5,30,30,150,220,255,100])
    #LaneYellowHSV.slider(window_name='output')

    ## Create Sliders for HSV and area setting
    # HSV_and_Area.slider(window_name="output")
    ## StopLine setting ##
    # StopLineHSV.slider(window_name="output")
    Stage.setStage(2)
    StopLineROI = np.array([    [(200, 400), (380, 400), (380, 180), (200,180)]    ])
    IntersectionStopLineROI = np.array([    [(160, 400), (440, 400), (440, 210), (160,210)]    ])
    TrafficLightHSV_green.setValue([59, 94, 45, 141, 95, 213, 300])
    # TrafficLightHSV_Red.slider(window_name="output")
    '''
    Controller setting
    '''
    # controller.setStraightParam(B=1.1,C=1)
    controller.setTurnParam(1.7+0.2,0,1.2+0.2,0,2+0.2,0,2.3,0)

    '''
    frequency condition setting
    '''
    frame_divisor_Lane_following = 1
    frame_divisor_Human_detection = 1
    frame_divisor_Stop_line_detection = 1
    frame_divisor_Aruco_detection = 1
    frame_reset = 24

    '''
    distance condition setting
    '''
    Main_odometer = LF.odometer
    # distance_Lane_following = 0
    # distance_Stop_line_detection = 0
    distance_Human_detection = [10,60,110,170] #boundaries [cm]
    distance_Aruco_detection = [60,100,120,300]
    '''
    condition swtich
    '''
    Do_Lane_following = True
    Do_Human_detection = False
    Do_Stop_line_detection = False
    Do_Aruco_detection = False
    allow_human_detection = True
    N0_trafficLight_count = 0
 ########################## main loop #############################  
    while cv2.getWindowProperty("output", 0) >= 0:
        start_time=time.time()
        # Read from camera and display on windows
        ret, img= camera.read()
        #reset frame if frame is greater than 60
        if frame > frame_reset:
            frame = 0
        #frame calculate
        frame = frame + 1
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
                
                '''
                condition determination
                '''
                if frame % frame_divisor_Lane_following == 0 :
                    Do_Lane_following = True
                else:
                    Do_Lane_following = False
                if allow_human_detection:
                    if frame % frame_divisor_Human_detection == 0 :
                        if Main_odometer.distance >= distance_Human_detection[2] and  Main_odometer.distance <= distance_Human_detection[3]:
                            Do_Human_detection = True
                        elif  Main_odometer.distance >= distance_Human_detection[0] and  Main_odometer.distance <= distance_Human_detection[1]:                        
                            Do_Human_detection = True
                        else :
                            Do_Human_detection = False
                    else: 
                        Do_Human_detection = False
                        HD.Stop()
                else:
                    Do_Human_detection = False
                # Test
                # Do_Human_detection = False
                # Do_Lane_following =False
                # if frame % frame_divisor_Stop_line_detection == 0 :
                #     if Main_odometer.distance == distance_Stop_line_detection: 
                #         Do_Stop_line_detection = True
                #     else:
                #         Do_Stop_line_detection = False
                # else:
                #     Do_Stop_line_detection = False

                if frame % frame_divisor_Aruco_detection == 0 :
                    if Main_odometer.distance >= distance_Aruco_detection[2] and  Main_odometer.distance <= distance_Aruco_detection[3]:
                        Do_Aruco_detection = True
                    elif  Main_odometer.distance >= distance_Aruco_detection[0] and  Main_odometer.distance <= distance_Aruco_detection[1]:                        
                        Do_Aruco_detection = True
                    else :
                        Do_Aruco_detection = False
                else: 
                    Do_Aruco_detection = False
                # print('[Main debug] '+str(Do_Human_detection))
                # Start threads #
                if Do_Human_detection:
                    HumandetectThread = threading.Thread(target=HD.Run,args=(undistort_img,))
                    HumandetectThread.start()
                # if Do_Stop_line_detection:
                #     StopLineThread = threading.Thread(target=CD.StopLineColorDetector,args = (
                #         perspectiveTransform_img, 
                #         StopLineROI
                #         ,None,None,None,StopLineHSV.getValue(),))
                #     StopLineThread.start()
                if Do_Aruco_detection:
                    ArucoThread = threading.Thread(target=Navigator.RunAtIntersection,args=(img,Mycam.camera_matrix,Mycam.dist_coeff,45,))
                    ArucoThread.start()
                
                '''
                Human detection
                '''
                if Do_Human_detection: 
                    HumandetectThread.join()    #wait for detection
                    outputIMG =HD.detectIMG
                    HD.Run(img)
                    if HD.isDetectHuman:
                        HD.do_human_aviodance()
                        HD.Stop()
                        Main_odometer.distance+=10
                        allow_human_detection = False
                '''
                Lane following
                '''
                if Do_Lane_following: 
                    Lane_Following_img = LF.Run(
                        perspectiveTransform_img = perspectiveTransform_img, 
                        dt = timedifferent,
                        HSV_Data = LaneYellowHSV.getValue(),
                        right_turning_mode_distance_threshold = 15, 
                        Stop=False
                    )
                    # outputIMG = Lane_Following_img
                    if not Do_Human_detection:
                        outputIMG = Lane_Following_img
                    if LF.right_turn_mode:  # Open loop right turn motion
                        LF.Stop()
                        controller.go_straight(14)
                        controller.turn(np.deg2rad(70))
                        controller.go_straight(7.5)

                '''
                Aruco detection
                '''
                if Do_Aruco_detection:
                    ArucoThread.join()
                    if Navigator.atIntersection:
                        print('[Main]Intersection detect')
                        LF.Stop()
                        HD.Stop()
                        # Stage.setPause()
                        Stage.nextStage()

                '''
                Stop line detection:
                '''
                # if Do_Stop_line_detection:
                #     StopLineThread.join()   # wait fot Stop Line detection
                #     # outputIMG = CD.stopLineIMG
                #     if CD.StopLineColor:
                #         print('[Main]Intersection detect')
                #         LF.Stop()
                #         HD.Stop()
                #         Stage.setPause()
                #         # Stage.nextStage()
                
            

            '''
            Stage 2: Calulate paths and go to the position in front of the stop line
            '''
            if Stage.isStage(2) and not Stop:
                # time.sleep(1)
                if not Navigator.complete:
                    outputIMG=Navigator.CalculatePaths(GP,img,Mycam.camera_matrix,Mycam.dist_coeff)
                else :
                    Navigator.GotoStopPoint(controller=controller)
                    Main_odometer.distance+=7
                    # Stage.setPause()
                    Stage.nextStage()
                    trafficLight_count = 0
            '''
            Stage 3: Recalculate paths and wait for the green light
            '''
            if Stage.isStage(3) and not Stop:

                '''
                Stop line navigation
                '''
                # stop_line_img,stopLineMask = CD.StopLineColorDetector (
                #     perspectiveTransform_img, 
                #     ROI =IntersectionStopLineROI,
                #     HSV_Data=StopLineHSV.getValue()
                # )
                # outputIMG = stop_line_img
                # if CD.StopLineColor:
                #     print('[Main]Stopline detect,recalculate paths')
                #     outputIMG=Navigator.ReCalculatePathsWithStopLine(perspectiveTransform_img,stopLineMask)
                #     Stage.setPause()

                '''
                Traffic Light detection
                '''
                Traffic_light_marked_img = CD.TrafficLightDetector(
                    img,
                    green_lower = np.array([TrafficLightHSV_green.H_min, TrafficLightHSV_green.S_min, TrafficLightHSV_green.V_min], np.uint8),
                    green_upper = np.array([TrafficLightHSV_green.H_max, TrafficLightHSV_green.S_max, TrafficLightHSV_green.V_max], np.uint8),
                    green_area_lower_limit = TrafficLightHSV_green.Area,
                    red_lower=  np.array([TrafficLightHSV_Red.H_min, TrafficLightHSV_Red.S_min, TrafficLightHSV_Red.V_min], np.uint8),
                    red_upper= np.array([TrafficLightHSV_Red.H_max, TrafficLightHSV_Red.S_max, TrafficLightHSV_Red.V_max], np.uint8),
                    red_area_lower_limit = TrafficLightHSV_Red.Area
                    )
                outputIMG = Traffic_light_marked_img

                if CD.RedLight :
                    N0_trafficLight_count = 0
                if CD.GreenLingt or CD.YellowLight:
                    print('[Main] Green or Yellow light, go')
                    Stage.nextStage()
                elif not CD.RedLight and not CD.YellowLight and not CD.GreenLingt:
                    N0_trafficLight_count += 1
                    if N0_trafficLight_count > 10:
                        print('[Main] No traffic light, go')
                        Stage.nextStage()
                
            '''
            Stage 4: go to the next road
            '''
            if Stage.isStage(4) and not Stop:
                    Navigator.GotoEntry(controller=controller)
                    Navigator.Stop()
                    Stage.nextStage()
                    start_time = time.time()
                    allow_human_detection = True
            '''
            Function testing stage: remember to set the total_stage to the number '5' and use
            the setStage(5) function to active this stage.
            '''
            if Stage.isStage(5) and not Stop:
                '''
                Traffic Light detection
                '''
                Traffic_light_marked_img = CD.TrafficLightDetector(
                    img,
                    green_lower = np.array([TrafficLightHSV_green.H_min, TrafficLightHSV_green.S_min, TrafficLightHSV_green.V_min], np.uint8),
                    green_upper = np.array([TrafficLightHSV_green.H_max, TrafficLightHSV_green.S_max, TrafficLightHSV_green.V_max], np.uint8),
                    green_area_lower_limit = TrafficLightHSV_green.Area,
                    red_lower=  np.array([TrafficLightHSV_Red.H_min, TrafficLightHSV_Red.S_min, TrafficLightHSV_Red.V_min], np.uint8),
                    red_upper= np.array([TrafficLightHSV_Red.H_max, TrafficLightHSV_Red.S_max, TrafficLightHSV_Red.V_max], np.uint8),
                    red_area_lower_limit = TrafficLightHSV_Red.Area
                    )
                outputIMG = Traffic_light_marked_img

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
            cv2.putText(outputIMG,str('Odometer : ')+str(Main_odometer.distance),(60,20),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),1,cv2.LINE_AA)
            cv2.putText(outputIMG,str('Aruco detect : ')+str(Do_Aruco_detection),(60,40),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),1,cv2.LINE_AA)
            cv2.putText(outputIMG,str('Human detect : ')+str(Do_Human_detection),(60,60),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),1,cv2.LINE_AA)
            cv2.imshow("output",outputIMG)
        keyCode = cv2.waitKey(1)

        ############################  Command handle ###############################

        # ESC pressed to quit
        if keyCode%256 == 27:
            StopLineHSV.saveData()
            TrafficLightHSV_Red.saveData()
            # LaneYellowHSV.saveData() # Save data after using slider
            print("[Main] Escape hit, closing...")
            break

        # R visualize_Ref_frame
        if keyCode%256 == 114:
            if Key_visualize_Ref:
                Key_visualize_Ref =False
                Stop = False
            else:
                Key_visualize_Ref=True
                controller.robotStop()
                Stop = True
                Main_odometer.distance = 0
                allow_human_detection = True
                N0_trafficLight_count = 0
        # T next stage
        if keyCode%256 == 116:
            print('[Main] Force next stage')
            controller.robotStop()
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