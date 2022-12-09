import cv2 as cv
import glob
import os
import math
import matplotlib.pyplot as plt
import numpy as np

def GetCenterFromCnt(contour) -> tuple:
    M = cv.moments(contour)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    return (cx, cy)

def Denoise(img: np.ndarray) -> np.ndarray:
    kernel = np.ones((3,3), np.uint8)
    img = cv.erode(img, kernel, iterations = 1)
    img = cv.dilate(img, kernel, iterations = 1)
    return img

rollCalib = cv.imread('rice_42.png')
sashimiCalib = cv.imread('cucumber_cali.png')

crop1 = (500, 700)
crop2 = (500, 700)
rollCalib = rollCalib[700:, 500:800, :]
sashimiCalib = sashimiCalib[700:, 500:800, :]
temp1 = rollCalib
temp2 = sashimiCalib
rollCalib = cv.cvtColor(rollCalib, cv.COLOR_BGR2GRAY)
sashimiCalib = cv.cvtColor(sashimiCalib, cv.COLOR_BGR2GRAY)
rollCalib = cv.adaptiveThreshold(rollCalib, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY_INV, 11, 2)
sashimiCalib = cv.adaptiveThreshold(sashimiCalib, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY_INV, 11, 2)

# rollCalib = Denoise(rollCalib)
# sashimiCalib = Denoise(sashimiCalib)

contours_1, _ = cv.findContours(rollCalib, mode=cv.RETR_LIST, method=cv.CHAIN_APPROX_NONE)
contours_2, _ = cv.findContours(sashimiCalib, mode=cv.RETR_LIST, method=cv.CHAIN_APPROX_NONE)

max_cnt_1 = sorted(contours_1, key = lambda cnt : -cv.contourArea(cnt))[0]
max_cnt_2 = sorted(contours_2, key = lambda cnt : -cv.contourArea(cnt))[0]

center1 = GetCenterFromCnt(max_cnt_1)
center2 = GetCenterFromCnt(max_cnt_2)

center1 = np.array([x+y for x, y in zip(center1, crop1)])
center2 = np.array([x+y for x, y in zip(center2, crop2)])

empty_1 = np.zeros_like(rollCalib)
empty_2 = np.zeros_like(sashimiCalib)

area1 = cv.contourArea(max_cnt_1)
area2 = cv.contourArea(max_cnt_2)

scale1 = math.sqrt(area1) / 25    # pixel per mm 
scale2 = math.sqrt(area2) / 25   # pixel per mm

cv.drawContours(empty_1, [max_cnt_1], -1, color = (255, 0, 0), thickness = cv.FILLED)
cv.drawContours(empty_2, [max_cnt_2], -1, color = (255, 0, 0), thickness = cv.FILLED)

fig = plt.figure(figsize=(10, 8))
ax1 = fig.add_subplot(1, 2, 1)
ax1.set_title(f"scale of tekamaki: {scale1:.2f}/mm^2, center at {center1//scale1}")
# ax1.imshow(empty_1, cmap = 'gray')
ax1.imshow(temp1, cmap = 'gray')
ax2 = fig.add_subplot(1, 2, 2)
ax2.set_title(f"scale of sashimi: {scale2:.2f}/mm^2, center at {center2//scale2}")
# ax2.imshow(empty_2, cmap = 'gray')
ax2.imshow(temp2, cmap = 'gray')
plt.show()