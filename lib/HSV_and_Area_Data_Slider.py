import cv2
import os
import numpy as np
class HSV_and_Area_Setting:
    '''
    HSV and area setting with data IO class
    
    Common functions:
        saveData(): save the value to npz data file
        slider(): create a slider to tune values 
        setValue(): set HSV and area values directly 
        readValue():get HSV and area values

    Args:
        saveDirPath:data directory path
        dataname: hsvdata name ex:'stoplineHsvdata'
    '''

    def __init__(self,saveDirPath,dataname):
        self.__saveDirPath = saveDirPath
        self.__dataname = dataname
        self.H_min = 0
        self.H_max = 180
        self.S_min = 0
        self.S_max = 255
        self.V_min = 0
        self.V_max = 255
        self.Area = 3000
        self.readData()

    def setValue(self,HSV:list):
        '''
        Set HSV and area values directly 

        Args:
            HSV:1-D list, ex:[H_min,H_max,S_min,S_max,V_min,V_max,Area]
        '''
        self.H_min = HSV[0]
        self.H_max = HSV[1]
        self.S_min = HSV[2]
        self.S_max = HSV[3]
        self.V_min = HSV[4]
        self.V_max = HSV[5]
        self.Area = HSV[6]
    
   
    def getValue(self):
        '''
        Get the data

        Return:
            list[H_min,H_max,S_min,S_max,V_min,V_max,Area]
        '''
        return [self.H_min,self.H_max,self.S_min,self.S_max,self.V_min,self.V_max,self.Area]
    
   
    def readData(self):
        '''
        Read existing data
        '''
        savePath = self.__saveDirPath+'/'+self.__dataname+'.npz'
        if os.path.exists(savePath):
            data = np.load(savePath)
            HSVdata = data['HSVdata']
            # print('load:')
            # print(HSVdata)
            self.H_min = HSVdata[0]
            self.H_max = HSVdata[1]
            self.S_min = HSVdata[2]
            self.S_max = HSVdata[3]
            self.V_min = HSVdata[4]
            self.V_max = HSVdata[5]
            self.Area  = HSVdata[6]
        else:
            print('initHSVdata: '+str(self.__dataname))
            self.saveData()

    def saveData(self):
        '''
        save the value to npz data file
        '''
        savePath = self.__saveDirPath+'/'+self.__dataname
        if not os.path.isdir(self.__saveDirPath):
            print('no HSV directory is found, create directory')
            os.mkdir(self.__saveDirPath)
        HSVdata = np.array([self.H_min,self.H_max,self.S_min,self.S_max,self.V_min,self.V_max,self.Area])
        # print('save:')
        # print(HSVdata)
        np.savez(savePath,HSVdata = HSVdata)

    
    def slider(self, window_name):
        '''
        create a slider to tune values 
        '''
        def updateH_min(x):
            self.H_min = cv2.getTrackbarPos('H_min', window_name)
        def updateH_max(x):
            self.H_max = cv2.getTrackbarPos('H_max', window_name)
        def updateS_min(x):
            self.S_min = cv2.getTrackbarPos('S_min', window_name)
        def updateS_max(x):
            self.S_max = cv2.getTrackbarPos('S_max', window_name)
        def updateV_min(x): 
            self.V_min = cv2.getTrackbarPos('V_min', window_name)
        def updateV_max(x):
            self.V_max = cv2.getTrackbarPos('V_max', window_name)
        def updateArea(x):
            self.Area  = cv2.getTrackbarPos('Area', window_name)

        cv2.createTrackbar('H_min',window_name,self.H_min,180,updateH_min)
        cv2.createTrackbar('H_max',window_name,self.H_max,180,updateH_max)
        cv2.createTrackbar('S_min',window_name,self.S_min,255,updateS_min)
        cv2.createTrackbar('S_max',window_name,self.S_max,255,updateS_max)
        cv2.createTrackbar('V_min',window_name,self.V_min,255,updateV_min)
        cv2.createTrackbar('V_max',window_name,self.V_max,255,updateV_max)
        cv2.createTrackbar('Area',window_name,self.Area,3000,updateArea)
