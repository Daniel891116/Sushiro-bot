# seaweed manager

from copy import deepcopy
from .robotWorkspace import *

seaweedReadyHeight = 250
seaweedDownHeight = 225
seaweedHighHeight = 300

class seaweedManager():
    def __init__(self):
        # riceStandardPose
        Pose0 = Pose(260, 260, 225, -180, 0, 45)
        Pose1 = Pose(300, 300, 225, -180, 0, 45)
        Pose2 = Pose(340, 340, 225, -180, 0, 45)
        self.seaweed = [Pose0, Pose1, Pose2]

    def consult(self):
        if len(self.seaweed) == 0:
            print("[Error] No more seaweed !!!")
            return [readyPose, readyPose, readyPose]

        self.seaweed[-1].z = seaweedReadyHeight
        PoseReady = deepcopy(self.seaweed[-1])
        self.seaweed[-1].z = seaweedDownHeight
        PoseDown = deepcopy(self.seaweed[-1])
        self.seaweed[-1].z = seaweedHighHeight
        PoseHigh = deepcopy(self.seaweed[-1])
        Poses = [PoseReady, PoseDown, PoseHigh]
        self.seaweed.pop()

        return Poses
