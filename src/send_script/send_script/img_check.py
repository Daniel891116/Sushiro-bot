import numpy as np
import cv2 as cv
import glob
import os
import math
import matplotlib.pyplot as plt
from .imgutils import DetectRice, DetectCucumber, DetectSalmon

def isSashimiOK(img: np.ndarray) -> bool:
    _, Salmon = DetectSalmon(img)
    print(Salmon)
    return False

def isRiceballOK(img) -> bool:
    _, Rice = DetectRice(img)
    print(Rice)
    return False

def findCucumber(img) -> bool:
    _, Cucumber = DetectCucumber(img)
    return False

def isMakkiRiceEnough(img) -> bool:
    return False

def isMakkiTuffingOK(img) -> bool:
    return False