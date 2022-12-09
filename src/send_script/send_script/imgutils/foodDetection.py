import numpy as np
import cv2 as cv
import glob
import os
import math
import matplotlib.pyplot as plt

def Findedge(counter):
    # sobel_x = np.array\
    # (
    #     [
    #         [ 1,  0, -1],
    #         [ 2,  0, -2],
    #         [ 1,  0, -1]
    #     ]
    # )
    # x_edge = cv.filter2D(input, ddepth=-1, kernel = cv.flip(sobel_x, -1)).astype(np.uint8)
    rect = cv.minAreaRect(counter)
    box = cv.boxPoints(rect)
    box = np.int0(box)
    sortedbox = sorted([point for point in box], key = lambda point : point[0])
    print(sortedbox[0:2])
    left_edge_mid = np.mean(sortedbox[0:2], axis = 0)
    print(left_edge_mid)
    return left_edge_mid

def DetectRice(bgrimg: np.ndarray):
    thres_area = 45000
    lower_bound = np.array([35,  10,  100])
    upper_bound = np.array([86, 40, 255])

    hsvimg = cv.cvtColor(bgrimg, cv.COLOR_BGR2HSV)
    [h, w] = hsvimg.shape[:-1]
    rice_mask = np.zeros([h, w])
    check_range = hsvimg[:, 470:960]
    mask = cv.inRange(check_range, lower_bound, upper_bound)
    rice_mask[:, 470:960] = mask
    # kernel = np.ones((7,7), np.uint8)
    # rice_mask = cv.erode(rice_mask, kernel, iterations = 1)
    # rice_mask = cv.dilate(rice_mask, kernel, iterations = 2)

    contours, _ = cv.findContours(image=rice_mask.astype(np.uint8), mode=cv.RETR_LIST, method=cv.CHAIN_APPROX_NONE)
    max_cnt = sorted(contours, key = lambda cnt : -cv.contourArea(cnt))[0]
    # cv.drawContours(rice_mask, [max_cnt], -1, color = (255, 255, 255), thickness = cv.FILLED)
    rice_area = cv.contourArea(max_cnt)
    M = cv.moments(max_cnt)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])

    phi = 0.5 * math.atan2(2 * M['nu11'], M['nu20'] - M['nu02'])
    orientation = phi * 180 / math.pi

    rect = cv.minAreaRect(max_cnt)
    box = cv.boxPoints(rect)
    box = np.int0(box)
    box_area = cv.contourArea(box)
    cv.drawContours(rice_mask, [box], -1,color = (255,255,255),thickness = 2)

    rice = dict()
    rice['area'] = rice_area
    rice['area_rate'] = rice_area / box_area
    rice['center'] = np.array([cx, cy])
    rice['orientation'] = orientation
    rice['valid'] = rice_area >= thres_area and (rice_area / box_area) >= 0.7 and abs(abs(orientation) - 90) <= 15
    
    return rice_mask, rice 

def DetectCucumber(bgrimg: np.ndarray):
    hsvimg = cv.cvtColor(bgrimg, cv.COLOR_BGR2HSV)
    lower_bound = np.array([30, 30,  30])
    upper_bound = np.array([60, 190, 215])

    [h, w] = hsvimg.shape[:-1]
    cucumber_mask = np.zeros([h, w])
    check_range = hsvimg
    mask = cv.inRange(check_range, lower_bound, upper_bound)
    cucumber_mask = mask
    kernel = np.ones((5,5), np.uint8)
    cucumber_mask = cv.erode(cucumber_mask, kernel, iterations = 1)
    cucumber_mask = cv.dilate(cucumber_mask, kernel, iterations = 2)

    contours, _ = cv.findContours(image=cucumber_mask.astype(np.uint8), mode=cv.RETR_LIST, method=cv.CHAIN_APPROX_NONE)
    max_cnt = sorted(contours, key = lambda cnt : -cv.contourArea(cnt))[0]
    M = cv.moments(max_cnt)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])

    phi = 0.5 * math.atan2(2 * M['nu11'], M['nu20'] - M['nu02'])
    orientation = phi * 180 / math.pi

    rect = cv.minAreaRect(max_cnt)
    box = cv.boxPoints(rect)
    box = np.int0(box)
    cv.drawContours(cucumber_mask, [box], -1,color = (255,255,255),thickness = 2)

    cucumber = dict()
    cucumber['center'] = np.array([cx, cy])
    cucumber['orientation'] = orientation
    
    return cucumber_mask, cucumber 

def DetectSalmon(bgrimg: np.ndarray):
    crop = np.array([400, 0])
    bgrimg = bgrimg[:, 400:]

    hsvimg = cv.cvtColor(bgrimg, cv.COLOR_BGR2HSV)
    hsvimg = cv.GaussianBlur(hsvimg, (51, 51), 0)

    lower_bound = np.array([ 30, 10, 140])
    upper_bound = np.array([ 75, 30, 220])

    [h, w] = hsvimg.shape[:-1]
    salmon_mask = np.zeros([h, w])
    check_range = hsvimg
    mask = cv.inRange(check_range, lower_bound, upper_bound)
    kernel = np.ones((5,5), np.uint8)
    mask = cv.erode(mask, kernel, iterations = 1)
    mask = cv.dilate(mask, kernel, iterations = 2)

    contours, _ = cv.findContours(image=mask.astype(np.uint8), mode=cv.RETR_LIST, method=cv.CHAIN_APPROX_NONE)
    valid_cnt = []
    for cnt in contours:
        if cv.contourArea(cnt) >= 15000:
            valid_cnt.append(cnt)
    cv.drawContours(image = salmon_mask, contours = valid_cnt, contourIdx = -1, color = (255, 255, 255), thickness = cv.FILLED)
    salmons = []
    for cnt in valid_cnt:
        mid_point = Findedge(cnt)
        M = cv.moments(cnt)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        center = np.array([cx, cy])

        phi = 0.5 * math.atan2(2 * M['nu11'], M['nu20'] - M['nu02'])
        orientation = phi * 180 / math.pi

        salmon = dict()
        salmon['center'] = center + crop
        salmon['edge_mid'] = mid_point + crop
        salmon['orientation'] = orientation
        salmons.append(salmon)
    
    return salmon_mask, salmons