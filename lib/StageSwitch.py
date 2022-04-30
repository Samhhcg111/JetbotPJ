
class StageSwitch:
    def __init__(self,TotalStage:int = 4):
        self.__stage = 1
        self.__TotalStage = TotalStage
        self.__pause = False
    def nextStage(self):
        self.__pause = False
        self.__stage+=1
        if self.__stage >self.__TotalStage:
            self.__stage=1
        self.printStage()

    def setStage(self,stage:int):
        self.__stage=stage

    def setPause(self):
        self.__pause =True

    def isStage(self,number:int):
        if self.__pause:
            return False
        if self.__stage == number:
            return True
        return False

    def printStage(self):
        print('Stage : '+str(self.__stage))
