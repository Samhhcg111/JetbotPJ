
class StageSwitch:
    def __init__(self):
        self.__stage = 1
    def nextStage(self):
        self.__stage+=1
        if self.__stage >3:
            self.__stage=1
        self.printStage()
    def setStage(self,stage:int):
        self.__stage=stage

    def isStage1(self):
        if self.__stage == 1:
            return True
        return False
    def isStage2(self):
        if self.__stage == 2:
            return True
        return False
    def isStage3(self):
        if self.__stage == 3:
            return True
        return False
    def printStage(self):
        print('Stage : '+str(self.__stage))
