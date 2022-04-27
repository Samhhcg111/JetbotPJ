import csv
import numpy as np
import math
import os
import cv2
from lib.GlobalPosDET import GlobalPosDET
from lib.Dijkstra import Dijkstra
from lib.Intersection import Intersesction_Navigator

class Navigator:
    digit_map=np.empty((7,7)).astype(int)
    map_path = os.path.abspath(__file__)+'\\..\\map.csv'
    PathfindingMC=None
    path=None
    IN = Intersesction_Navigator()
    aruco_dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
    def __init__(self,map_path,i0:int,j0:int,i_dest:int,j_dest:int):
        if map_path is not None:
            self.map_path=map_path
        with open(self.map_path,newline='') as mapfile:
            rows=csv.reader(mapfile,delimiter=',')
            i=0
            for row in rows:
                j=0
                for col in row:
                    self.digit_map[i,j]=int(col)
                    j+=1
                i+=1
        self.PathfindingMC = Dijkstra(self.digit_map)
        self.goalGlobalPos = [i_dest,j_dest] 
        self.genPath(i0,j0,i_dest,j_dest)
        self.count = 0
        self.complete = False
        self.crossPath = None
        # self.ArucoParam = cv2.aruco.DetectorParameters_create()
    def genPath(self,i0:int,j0:int,i_dest:int,j_dest:int):
        self.path = self.PathfindingMC.getpath(i0,j0,i_dest,j_dest)
    def presentMap(self):
        MAP_img = np.zeros((480,480,3))
        MAP_img[:] = (255,255,255)
        row=MAP_img.shape[0]*0.9
        col=MAP_img.shape[1]*0.9
        s_row=row/self.digit_map.shape[0]
        s_col=col/self.digit_map.shape[1]
        offset_row=MAP_img.shape[0]*0.1
        offset_col=MAP_img.shape[1]*0.1
        if self.path is None:
            return MAP_img
        #draw road
        for i in range(0,len(self.path)-1,1):
            MAP_img = cv2.line(MAP_img,(int(self.path[i].j*s_col+offset_col),int(self.path[i].i*s_row+offset_row)),(int(self.path[i+1].j*s_col+offset_col),int(self.path[i+1].i*s_row+offset_row)),(204,0,0),2)
        for i in range(0,self.digit_map.shape[0],1):
            for j in range(0,self.digit_map.shape[1],1):
                if(self.digit_map[i][j]==2 or self.digit_map[i][j]==1):
                    MAP_img=cv2.circle(MAP_img,(int(j*s_col+offset_col),int(i*s_row+offset_row)),10,(0,0,0))
        MAP_img = cv2.circle(MAP_img,(int(self.path[0].j*s_col+offset_col),int(self.path[0].i*s_row+offset_row)),5,(0,0,204))
        if(len(self.path)>2):
            for i in range(1,len(self.path)-1,1):
                MAP_img=cv2.circle(MAP_img,(int(self.path[i].j*s_col+offset_col),int(self.path[i].i*s_row+offset_row)),5,(255,128,0))
        MAP_img=cv2.circle(MAP_img,(int(self.path[len(self.path)-1].j*s_col+offset_col),int(self.path[len(self.path)-1].i*s_row+offset_row)),5,(127,0,255))
        return MAP_img

    def __distance(self,vector):
        distance = math.sqrt(vector[0]*vector[0]+vector[1]*vector[1])
        return distance
        
    def __Identify_entry(self,nowPos:np.array,nextPos:np.array):
        direction = nextPos-nowPos
        if direction[0]<0 and int(direction[1]) == 0:
            return 0
        if direction[0]>0  and int(direction[1]) == 0:
            return 2
        if direction[1]>0 and int(direction[0]) == 0:
            return 1
        if direction[1]<0 and int(direction[0]) == 0:
            return 3 
    def Run(self,Global_DET:GlobalPosDET,src_img,mtx,dist):
        corners,ids,rejectImgPoints = cv2.aruco.detectMarkers(src_img,self.aruco_dictionary)
        # corners,ids,rejectImgPoints = cv2.aruco.detectMarkers(src_img,self.aruco_dictionary,parameters=self.ArucoParam)
        output_img = src_img.copy()
        vectors = []
        if ids is not None:
            if self.count ==0:
                if len(ids)==1:
                    closeId = ids[0][0]                 
                else:
                    min_distance = 100
                    for i in range(0,len(ids),1):
                        rvec,tvec,_objpoints = cv2.aruco.estimatePoseSingleMarkers(corners[i],3.8,mtx,dist)
                        distance = self.__distance(tvec[0][0][0:2])
                        if distance<min_distance:
                            min_distance = distance
                            closeId = ids[i][0]
                self.intersection_id,pos,self.now_entry_point = self.IN.Indentify_intersection(closeId)
                Global_DET.setPos(pos[0],pos[1])
                self.genPath(pos[0],pos[1],self.goalGlobalPos[0],self.goalGlobalPos[1])
                nextPos = self.path[1]
                self.entry = self.__Identify_entry(np.array([pos[0],pos[1]]),np.array([nextPos.i,nextPos.j]))
                # print('go entry point : '+str(self.entry))
                self.count+=1
                # print('Debug1')
                
            if self.count>0:
                # self.now_entry_point = 2 #NoteBook Test
                output_img,JS_distance,JS_radians,SN_distance,SN_radians,NE_radians,NE_distance = self.IN.navigate(src_img,mtx,dist,corners,ids,self.intersection_id,self.entry,self.now_entry_point)
                # output_img,JS_distance,JS_radians,SN_distance,SN_radians,NE_radians,NE_distance = self.IN.navigate(src_img,mtx,dist,corners,ids,2,3,2) #NoteBook Test
                self.count+=1
                vectors.append([JS_distance,JS_radians,SN_distance,SN_radians,NE_radians,NE_distance])
                # print('count '+str(self.count))
            if self.count >5:
                self.crossPath = np.average(vectors,0)
                self.complete = True
        return output_img
    def atIntersection(self,src_img,mtx,dist,Critical_distance = 30):
        corners,ids,rejectImgPoints = cv2.aruco.detectMarkers(src_img,self.aruco_dictionary)
        if ids is None:
            return False
        else:
            if len(ids)==1:
                closeId = ids[0][0]                 
            else:
                min_distance = 100
                for i,id in enumerate(ids):
                    rvec,tvec,_objpoints = cv2.aruco.estimatePoseSingleMarkers(corners[i],3.8,mtx,dist)
                    distance = self.__distance(tvec[0][0][0:2])
                    if distance<min_distance:
                        min_distance = distance
                        closeId = id[0]
            self.intersection_id,pos,now_entry_point = self.IN.Indentify_intersection(closeId)
            Pos = self.IN.getCameraPosition(src_img,mtx,dist,corners,ids, self.intersection_id)
            distance = self.__distance(Pos[0:2])
            if distance < Critical_distance :
                return True
            else:
                return False
    def Stop(self):
        self.count = 0
        self.complete = False
        self.crossPath = None
    
