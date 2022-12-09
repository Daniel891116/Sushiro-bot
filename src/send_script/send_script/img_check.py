import numpy as np
import cv2 as cv
import glob
import os
import math
import matplotlib.pyplot as plt
from .imgutils import DetectRice, DetectCucumber, DetectSalmon

sashimi_gripper_pos  = np.array([ 72, 103])
tekamaki_gripper_pos = np.array([199, 147])

sashimi_calib_scale = 5.27
tekamaki_calib_scale = 4.30

def findSalmon(img: np.ndarray) -> tuple:
    _, Salmons = DetectSalmon(img)
    displacement = Salmons[0]['edge_mid']/sashimi_calib_scale - sashimi_gripper_pos
    print(f"gripper should move {displacement}")
    return displacement

def isRiceballOK(img) -> tuple:
    _, Rice = DetectRice(img)
    displacement = Rice['center']/tekamaki_calib_scale - tekamaki_gripper_pos
    print(f"gripper should move {displacement}")
    if Rice['valid']:
        return True, (0, 0)
    else:
        return False, displacement

def findCucumber(img) -> tuple:
    _, Cucumber = DetectCucumber(img)
    displacement = Cucumber['center']/sashimi_calib_scale - sashimi_gripper_pos
    print(f"gripper should move {displacement}")
    if Cucumber.keys != []:
        return True, displacement
    else:
        return False, (0, 0)

def isMakkiRiceEnough(img) -> bool:
    return False

def isMakkiTuffingOK(img) -> bool:
    return False