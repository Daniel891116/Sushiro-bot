# rice manager

from copy import deepcopy
from .robotWorkspace import *

class riceManager():
    def __init__(self):
        # riceStandardPose
        self.riceMap = dict()

        for m in [-1, 0, 1]:
            for n in [-1, 0, 1]:
                riceInfo = deepcopy(riceBowlPose("down"))
                riceInfo.x += m * riceSize
                riceInfo.y += n * riceSize
                riceInfo.z -= (abs(m) * riceDecay + abs(n) * riceDecay)
                self.riceMap[(riceInfo.x, riceInfo.y)] = riceInfo

        self.justConsulted = None

    def consult(self, target):
        suggestedList = deepcopy(sorted(self.riceMap.items(), key = lambda riceInfo: riceInfo[1].z, reverse = True))
        for suggested in suggestedList:
            suggested = suggested[1]
            if suggested.z < riceLowest + riceHeight(target):
                print("[Error] Rice may be not enough?")
                return riceBowlPose("down")
            suggestedKey = (suggested.x, suggested.y)
            if suggestedKey != self.justConsulted:
                self.justConsulted = suggestedKey
                break

        self.riceMap[suggestedKey].z -= riceHeight(target)
        suggested.z -= riceHeight(target)
        # print("Suggested: ", end = "")
        # print(suggested)
        return suggested
