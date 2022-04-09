# import math
import numpy as np
import cv2
import math
from lib.MyCam import MyCam
class Intersesction_Navigator:
    __intersections={}
    __intersection_size=np.array([15,15])
    def __init__(self):
        self.__intersections={}
        intersection_size = 35
        for i in range(1,4,1): #test
        # for i in range(1,7,1): # NoteBook Test
            section = None
            ### NoteBook Test ####
            # if(i==2):
            #     section=Intersection(i,np.array([None,3,4,5,2]),(2,2))
            ############

            ####test####
            if (i == 1):
                section = Intersection(i,np.array([1,2,3,None,None]),(2,1))
            if (i == 2):
                section = Intersection(i,np.array([5,None,7,8,None]),(2,3))
            if (i == 3):
                section = Intersection(i,np.array([9,10,None,12,None]),(4,3))
            if section is not None:
            ############
                self.__intersections[i]=section
    def Indentify_intersection(self,arucoid):
        pos_id = None
        for Intersection_id in self.__intersections:
            for id in self.__intersections[Intersection_id].ids:
                if id[0] == arucoid:
                    pos_id = self.__intersections[Intersection_id].id
                    pos = self.__intersections[Intersection_id].pos
        if pos_id is None:
            print('Not such aruco id : '+str(arucoid))
        return pos_id,pos
    def navigate(self,frame,mtx,dist,corners,aruco_ids,intersection_id,entry_point:int):
        img = frame.copy()
        rows,cols = frame.shape[:2]
        retval,rvec,tvec = cv2.aruco.estimatePoseBoard(corners,aruco_ids,self.__intersections[intersection_id].aruco_board,mtx,dist,None,None)
        if rvec is not None and tvec is not None:
            for entry in self.__intersections[intersection_id].entries:
                if entry is not None:
                    en_x,en_y=entry
                    objpt = np.array([[en_x,en_y,0]])
                    imgpt,jacobian = cv2.projectPoints(objpt,rvec,tvec,mtx,dist)
                    u,v=imgpt[0][0]
                    cv2.circle(img,(int(u),int(v)),5,(255,0,255),2)
            entry =self.__intersections[intersection_id].entries[entry_point]
            node = self.__intersections[intersection_id].entryNodes[entry_point]
            if  entry is not None:
                camera_pos = np.array([[0.],[0.],[0.]])
                # XYZ  = Intersesction_Navigator.jetbot2world(rvec,tvec,camera_pos)
                XYZ = Intersesction_Navigator.camera2world(rvec,tvec,camera_pos)
                camera_front_pos = np.array([[0.],[0.],[1]])
                # XYZ_front = Intersesction_Navigator.jetbot2world(rvec,tvec,camera_front_pos)
                XYZ_front = Intersesction_Navigator.camera2world(rvec,tvec,camera_front_pos)
                direction_vector = np.array([XYZ_front[0][0]-XYZ[0][0],XYZ_front[1][0]-XYZ[1][0]])
                JN_vector = np.array([node[0]-XYZ[0][0],node[1]-XYZ[1][0]])
                NE_vector = np.array([node[0]-entry[0],node[1]-entry[1]])
                JN_distance = Intersesction_Navigator.__distance(JN_vector)+2 # offset to jetbot pos if usiing camera pos
                JN_radians = Intersesction_Navigator.__clockwiseAng(direction_vector,JN_vector)
                NE_distance = Intersesction_Navigator.__distance(NE_vector)
                NE_radians = Intersesction_Navigator.__clockwiseAng(direction_vector,NE_vector)
                # JO_ang = math.degrees(JO_radians)
                # OE_ang = math.degrees(OE_radians)
                # cv2.putText(img,str('Pos: ')+str(XYZ[0:2]),(100,100),cv2.FONT_HERSHEY_PLAIN,1,(255,255,255),1)
                # print('Turn '+str(JO_ang)+' go '+str(JO_distance))
                # print('and Turn '+str(OE_ang)+' go '+str(OE_distance))
                self_objpt = np.array([[XYZ[0][0],XYZ[1][0],0]])
                imgpt,jacobian = cv2.projectPoints(self_objpt,rvec,tvec,mtx,dist)
                u0,v0=imgpt[0][0]
                if u0>0 and v0 >0 and u0 <img.shape[1] and v0<img.shape[0]:
                    cv2.circle(img,(int(u0),int(v0)),5,(0,0,255),2)
                Node_objpt = np.array([[node[0],node[1],0.]])
                imgpt,jacobian = cv2.projectPoints(Node_objpt,rvec,tvec,mtx,dist)
                u1,v1=imgpt[0][0]
                img = MyCam.drawline(img,int(u0),int(v0),int(u1),int(v1),(0,255,0),2)
                E_objpt = np.array([[entry[0],entry[1],0]])
                imgpt,jacobian = cv2.projectPoints(E_objpt,rvec,tvec,mtx,dist)
                u3,v3=imgpt[0][0]
                img = MyCam.drawline(img,int(u1),int(v1),int(u3),int(v3),(0,255,0),2)
            cv2.aruco.drawAxis(img,mtx,dist,rvec,tvec,2)
        return img,JN_radians,JN_distance,NE_radians,NE_distance
    def camera2world(rvec:np.array,tvec:np.array,objpt:np.array):
        objpt = np.r_[objpt,[[1]]]
        R,jacobian = cv2.Rodrigues(rvec)
        RT = np.c_[R,tvec]
        RT = np.r_[RT,[[0,0,0,1]]]
        pinv = np.linalg.pinv(RT)
        XYZ = np.asmatrix(pinv)*np.mat(objpt)
        XYZ = np.asarray(XYZ)
        return XYZ
    def jetbot2world(rvec:np.array,tvec:np.array,objpt:np.array):
        M_CJ = np.mat([[1.13904494e-02,-9.99929001e-01,-3.50012164e-03,-8.56096844e+00],[-6.00409085e-01,-4.04022905e-03,-7.99682816e-01,1.13824197e+01],[ 7.99611898e-01,1.12102515e-02,-6.00412477e-01,-8.25225470e+00],[-7.09838495e-18,6.56462602e-17,0.00000000e+00,1.00000000e+00]])
        objpt = np.r_[objpt,[[1]]]
        R,jacobian = cv2.Rodrigues(rvec)
        RT = np.c_[R,tvec]
        RT = np.r_[RT,[[0,0,0,1]]]
        pinv = np.linalg.pinv(RT)
        XYZ = np.asmatrix(pinv)*M_CJ*np.mat(objpt)
        XYZ = np.asarray(XYZ)
        return XYZ
    def __clockwiseAng(vector1,vector2):
        ref_v = np.array([0,1])
        def Angle_2D(v1,v2):
                l1 = math.sqrt(v1[0]*v1[0]+v1[1]*v1[1])
                l2 = math.sqrt(v2[0]*v2[0]+v2[1]*v2[1])
                radians =math.acos((v1[0]*v2[0]+v1[1]*v2[1])/l1/l2)
                return radians
        rad1 = Angle_2D(ref_v,vector1)
        if vector1[0]<0:
            rad1 = 2*math.pi-rad1
        rad2 = Angle_2D(ref_v,vector2)
        if vector2[0]<0:
            rad2 = 2*math.pi-rad2
        radians = rad2 - rad1
        if radians<0:
            radians+=2*math.pi
        return radians
    def __distance(vector):
        distance = math.sqrt(vector[0]*vector[0]+vector[1]*vector[1])
        return distance
class Intersection:
    aruco_dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
    aruco_size = 3.8
    aruco_high = 1.2
    # aruco_distance = 7.5+5 # NoteBook Test
    aruco_distance = 18 #test
    # intersection_size = 15 # NoteBook Test
    intersection_size = 15.5*2 #test
    def __init__(self,id:int,arucoIds:np.array,pos):
        self.id=id
        self.pos = [pos[0],pos[1]]
        self.entries=[None,None,None,None]
        self.entryNodes=[None,None,None,None]
        corners = []
        self.ids=[]
        a = self.aruco_distance
        b = self.intersection_size/2
        b2 = self.intersection_size/4
        c = self.aruco_high
        s = self.aruco_size
        if arucoIds[0] is not None:
            corners.append([[-b+s,a,0],[-b,a,0],[-b,a+s,0],[-b+s,a+s,0]]) #test
            # corners.append([[-b,a,c+s],[-b-s,a,c+s],[-b-s,a,c],[-b,a,c]]) # NoteBook Test
            self.ids.append([arucoIds[0]])
            self.entries[0] = (b2,b)
            self.entryNodes[0] = (b2,b2)
        if arucoIds[1] is not None:
            corners.append([[a,b-s,0],[a,b,0],[a+s,b,0],[a+s,b-s,0]]) #test
            # corners.append([[a,b,c+s],[a,b+s,c+s],[a,b+s,c],[a,b,c]]) # NoteBook Test
            self.ids.append([arucoIds[1]])
            self.entries[1] = (b,-b2)
            self.entryNodes[1] = (b2,-b2)
        if arucoIds[2] is not None:
            corners.append([[b-s,-a,0],[b,-a,0],[b,-a-s,0],[b-s,-a-s,0]]) #test
            # corners.append([[b,-a,c+s],[b+s,-a,c+s],[b+s,-a,c],[b,-a,c]]) # NoteBook Test
            self.ids.append([arucoIds[2]])
            self.entries[2] = (-b2,-b)
            self.entryNodes[2] = (-b2,-b2)
        if arucoIds[3] is not None:
            corners.append([[-a,-b+s,0],[-a,-b,0],[-a-s,-b,0],[-a-s,-b+s,0]]) #test
            # corners.append([[-a,-b,c+s],[-a,-b-s,c+s],[-a,-b-s,c],[-a,-b,c]]) # NoteBook Test
            self.ids.append([arucoIds[3]])
            self.entries[3] = (-b,b2)
            self.entryNodes[3] = (-b2,b2)
        if arucoIds[4] is not None:
            s2=s/2
            corners.append([[-s2,s2,0],[s2,s2,0],[s2,-s2,0],[-s2,-s2,0]]) 
            self.ids.append([arucoIds[4]])
        self.aruco_board = cv2.aruco.Board_create(np.array(corners,np.float32),self.aruco_dictionary,np.array(self.ids))
        
