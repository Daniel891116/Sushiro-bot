import numpy as np
import cv2 as cv
import glob
import os
import math
import matplotlib.pyplot as plt


def Denoise(img: np.ndarray) -> np.ndarray:
    kernel = np.ones((3,3), np.uint8)
    img = cv.erode(img, kernel, iterations = 1)
    img = cv.dilate(img, kernel, iterations = 1)
    return img

rollCalib = cv.imread('rice_42.png')
sashimiCalib = cv.imread('rice_40.png')
rollCalib = rollCalib[700:, 500:800, :]
sashimiCalib = sashimiCalib[100:500, 300:700, :]
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

empty_1 = np.zeros_like(rollCalib)
empty_2 = np.zeros_like(sashimiCalib)

area1 = cv.contourArea(max_cnt_1)
area2 = cv.contourArea(max_cnt_2)

scale1 = area1 / (25*25)    # pixel per mm square
scale2 = area2 / (25*25)    # pixel per mm square

cv.drawContours(empty_1, [max_cnt_1], -1, color = (255, 0, 0), thickness = cv.FILLED)
cv.drawContours(empty_2, [max_cnt_2], -1, color = (255, 0, 0), thickness = cv.FILLED)

fig = plt.figure(figsize=(10, 8))
ax1 = fig.add_subplot(1, 2, 1)
ax1.set_title(f"scale of tekamaki: {scale1:.2f}/mm^2")
ax1.imshow(empty_1, cmap = 'gray')
ax2 = fig.add_subplot(1, 2, 2)
ax2.set_title(f"scale of sashimi: {scale2:.2f}/mm^2")
ax2.imshow(empty_2, cmap = 'gray')
plt.show()