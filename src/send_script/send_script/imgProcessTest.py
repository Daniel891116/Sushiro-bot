# image process test for HW4

import cv2
import numpy as np
import scipy.ndimage as ndi
from skimage import measure as skiMeasure
import matplotlib.pyplot as plt

# assuming getting image "img"
filename = None
orginalImg = cv2.imread(filename)
img = cv2.cvtColor(orginalImg, cv2.COLOR_RGB2GRAY) # BGR2GRAY

# filter
filtSize = 5
filt = np.ones((filtSize, filtSize), np.float32) / (filtSize**2)
img = cv2.filter2D(img.astype('float32'), -1, filt, borderType = cv2.BORDER_CONSTANT)
# img = ndi.correlate(img, kernel, mode = 'nearest').transpose()

# to binary
threshold = 128 # ?
# img = np.where(img > threshold, 1, 0)
img = (img > 128).astype(np.int_)

# clear border, imclearborder

# image open and close
kernelSize = 5
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)) # MORPH_ELLIPSE, MORPH_CROSS
img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)

# regionprops
labels = skiMeasure.label(img, connectivity = 2)
print("regions number: ", labels.max() + 1)
print(skiMeasure.regionprops(labels).centroid)
