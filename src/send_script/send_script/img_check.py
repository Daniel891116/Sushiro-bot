import numpy as np
import cv2 as cv
import glob
import os
import math
import matplotlib.pyplot as plt
from .imgutils import DetectRice, DetectCucumber, DetectSalmon

sashimi_gripper_pos  = np.array([169, 205])
tekamaki_gripper_pos = np.array([153, 193])

sashimi_calib_scale = 3.89
tekamaki_calib_scale = 4.30

def findSalmon(img: np.ndarray) -> tuple:
    _, Salmons = DetectSalmon(img)
    if Salmons == []:
        return False, (0, 0)
    displacement = Salmons[0]['edge_mid']/sashimi_calib_scale - sashimi_gripper_pos
    print(f"found {len(Salmons)}")
    print(f"gripper should move {displacement}")
    return True, displacement

def isRiceballOK(img) -> tuple:
    _, Rice = DetectRice(img)
    displacement = Rice['center']/tekamaki_calib_scale - tekamaki_gripper_pos
    print(f"gripper should move {displacement}")
    if Rice['valid']:
        return True, displacement
    else:
        return False, displacement

def findCucumber(img) -> tuple:
    img, Cucumber = DetectCucumber(img)
    displacement = Cucumber['center']/sashimi_calib_scale - sashimi_gripper_pos
    print(f"gripper should move {displacement}")
    if Cucumber.keys() != []:
        return True, displacement
    else:
        return False, (0, 0)

def isMakkiRiceEnough(img) -> bool:
    return False

def isMakkiTuffingOK(img) -> bool:
    return False