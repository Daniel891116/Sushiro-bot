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

def GetOuterBox(img: np.ndarray):
    x_min = -1
    x_max = -1
    y_min = 0
    y_max = img.shape[0]

    def edge(scan1: np.ndarray, scan2: np.ndarray, threshold: float):
        max1 = np.max(scan1)
        max2 = np.max(scan2)
        if (max1 * threshold >= np.mean(scan1)) ^ (max2 * threshold >= np.mean(scan2)):
            return True
        else:
            return False

    for x in range(img.shape[1] - 1):
        if edge(img[:, x], img[:, x+1], 0.5) and (x_min == -1):
            x_min = x

    for x in range(img.shape[1] - 1, 1, -1):
        if edge(img[:, x], img[:, x - 1], 0.5) and (x_max == -1):
            x_max = x

    # for y in range(img.shape[0] - 1):
    #     if edge(img[y, :], img[y+1, :], 0.5) and (y_min == -1):
    #         y_min = y

    # for y in range(img.shape[0] - 1, 1, -1):
    #     if edge(img[y, :], img[y - 1, :], 0.5) and (y_max == -1):
    #         y_max = y

    return [[x_min, x_max], [y_min, y_max]]

def GradeRiceRoll(seaweed_mask: np.ndarray, box: list):
    x_range = box[0]
    y_range = box[1]
    w = box[0][1] - box[0][0]
    h = box[1][1] - box[1][0]
    seaweed_mask = seaweed_mask[y_range[0]:y_range[1], x_range[0]:x_range[1]]
    rice_mask = 255 - seaweed_mask

    def GetPropotion(img: np.ndarray) -> float:
        return np.mean(img)/np.max(img)

    def GetDistribution(img: np.ndarray) -> list:
        y_detect = 0
        h_detect = 100
        distribution = []

        while y_detect < h - h_detect:
            section = img[y_detect:y_detect + h_detect,:]
            if GetPropotion(section) >= 0.15: 
                if not(distribution == [] or distribution[-1] == -1):
                    distribution.append(-1)
                y_detect += 1
            else:
                distribution.append([y_detect + h_detect / 2])
                y_detect += 1

        if len(distribution) != 0:
            if distribution[-1] != -1:
                distribution.append(-1)

        return distribution

    def ProcessDistribution(distribution: list) -> list:
        current = []
        output = []

        for y in distribution:
            if y == -1:
                output.append(np.mean(current))
                current.clear()
            else:
                current.append(y)
        
        return output

    distribution = GetDistribution(rice_mask)
    distribution = ProcessDistribution(distribution)

    return rice_mask, np.array(distribution)

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
    empty = np.zeros((bgrimg.shape[0], bgrimg.shape[1]))
    crop = np.array([350, 0])
    bgrimg = bgrimg[:, 350:720]

    hsvimg = cv.cvtColor(bgrimg, cv.COLOR_BGR2HSV)
    # hsvimg = cv.GaussianBlur(hsvimg, (51, 51), 0)

    lower_bound = np.array([  3,  50, 107])
    upper_bound = np.array([ 15, 155, 215])

    [h, w] = hsvimg.shape[:-1]
    salmon_mask = np.zeros([h, w])
    check_range = hsvimg
    mask = cv.inRange(check_range, lower_bound, upper_bound)
    # kernel = np.ones((5,5), np.uint8)
    # mask = cv.erode(mask, kernel, iterations = 1)
    # mask = cv.dilate(mask, kernel, iterations = 2)

    contours, _ = cv.findContours(image=mask.astype(np.uint8), mode=cv.RETR_LIST, method=cv.CHAIN_APPROX_NONE)
    valid_cnt = []
    for cnt in contours:
        if cv.contourArea(cnt) >= 10000:
            valid_cnt.append(cnt)
    cv.drawContours(image = salmon_mask, contours = valid_cnt, contourIdx = -1, color = (255, 255, 255), thickness = cv.FILLED)
    salmons = []
    for cnt in valid_cnt:
        mid_point, _ = Findedge(cnt)
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
    empty[:, 350:720] = salmon_mask
    return empty, salmons

def DetectRiceRoll(bgrimg: np.ndarray):
    
    crop = np.array([300, 150])
    bgrimg = bgrimg[150:900, 300:1000]
    hsvimg = cv.cvtColor(bgrimg, cv.COLOR_BGR2HSV)
    hsvimg = cv.GaussianBlur(hsvimg, (21, 21), 0)

    lower_bound = np.array([   0, 18,  28])
    upper_bound = np.array([ 173, 64, 100])

    [h, w] = hsvimg.shape[:-1]
    seaweed_mask = np.zeros([h, w])
    check_range = hsvimg
    seaweed_mask = cv.inRange(check_range, lower_bound, upper_bound)
    contours, _ = cv.findContours(image=seaweed_mask.astype(np.uint8), mode=cv.RETR_LIST, method=cv.CHAIN_APPROX_NONE)
    # cv.drawContours(seaweed_mask, contours, -1, color = (255, 255, 255), thickness = cv.FILLED)
    # all_area = 0
    # for cnt in contours:
    #     all_area += cv.contourArea(cnt)

    kernel = np.ones((21,21), np.uint8)
    seaweed_mask = cv.dilate(seaweed_mask, kernel, iterations = 1)
    seaweed_mask = cv.erode(seaweed_mask, kernel, iterations = 1)
    outcontours, _ = cv.findContours(image=seaweed_mask.astype(np.uint8), mode=cv.RETR_LIST, method=cv.CHAIN_APPROX_NONE)
    
    cv.drawContours(seaweed_mask, outcontours, -1, color = (255, 255, 255), thickness = cv.FILLED)
    box = GetOuterBox(seaweed_mask)
    rice_mask, distribution = GradeRiceRoll(seaweed_mask, box)
    riceroll = dict()
    riceroll['box'] = box
    riceroll['empty_pos'] = distribution

    return rice_mask, riceroll

