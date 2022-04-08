from ast import Global
import numpy as np

class GlobalPosDET:
    def __init__(self,global_pos_i:int,global_pos_j:int):
        self.__Pos=np.array([[global_pos_i],[global_pos_j]])
    def getPos(self):
        return self.__Pos
    def setPos(self,global_pos_i:int,global_pos_j:int):
        self.__Pos = np.array([[global_pos_i],[global_pos_j]])