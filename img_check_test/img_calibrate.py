import cv2 as cv
import glob
import os
import math
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image as PImage
import plotly.graph_objects as go

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

def create_rgb_surface(rgb_img, depth_img, **kwargs):
    rgb_img = rgb_img.swapaxes(0, 1)[:, ::-1]
    depth_img = depth_img.swapaxes(0, 1)[:, ::-1]
    eight_bit_img = PImage.fromarray(rgb_img).convert('P', palette='WEB', dither=None)
    idx_to_color = np.array(eight_bit_img.getpalette()).reshape((-1, 3))
    colorscale=[[i/255.0, "rgb({}, {}, {})".format(*rgb)] for i, rgb in enumerate(idx_to_color)]
    depth_map = depth_img.copy().astype('float')
    # depth_map[depth_map<depth_cutoff] = np.nan
    return go.Surface(
        z=depth_map,
        surfacecolor=np.array(eight_bit_img),
        cmin=0, 
        cmax=255,
        colorscale=colorscale,
        showscale=False,
        **kwargs
    )

def genDepth_map(img: np.ndarray, depth: int) -> np.ndarray:
    depth_map = np.empty((img.shape[0], img.shape[1]), dtype=np.int16)
    depth_map[:] = depth
    return depth_map

rollCalib = cv.imread('rice_42.png')
sashimiCalib = cv.imread('cucumber_cali.png')

origin_rollCalib = rollCalib
origin_sashimiCalib = sashimiCalib

# origin_rollCalib = cv.cvtColor(origin_rollCalib, cv.COLOR_BGR2GRAY)
# origin_sashimiCalib = cv.cvtColor(origin_sashimiCalib, cv.COLOR_BGR2GRAY)

crop1 = (500, 700)
crop2 = (500, 700)
plot1 = np.zeros((rollCalib.shape[0], rollCalib.shape[1]))
plot2 = np.zeros((sashimiCalib.shape[0], sashimiCalib.shape[1]))
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

plot1[700:, 500:800] = empty_1
plot2[700:, 500:800] = empty_2

# fig = plt.figure(figsize=(10, 8))
# X = np.array(range(plot1.shape[1]))
# Y = np.array(range(plot1.shape[0]))
# ax1 = fig.add_subplot(1, 2, 1, projection = '3d')
# # ax1.set_title(f"scale of tekamaki: {scale1:.2f}/mm^2, center at {center1//scale1}")
# ax1.contourf(X, Y, plot1, 100, zdir='z', offset=350, cmap="gray", alpha = 0.5)
# ax1.contourf(X, Y, origin_rollCalib, 100, zdir='z', offset=0, cmap="gray", alpha = 0.5)
# ax1.set_zlim(0, 500)
# d = np.zeros((origin_rollCalib.shape[0], origin_rollCalib.shape[1]))
d1 = genDepth_map(origin_rollCalib, 0)
d2 = genDepth_map(origin_rollCalib, 500)
fig = go.Figure(
    data=[
        create_rgb_surface(np.flipud(cv.cvtColor(origin_rollCalib, cv.COLOR_BGR2RGB)), 
                             d1, 
                             contours_z=dict(show=True, project_z=True, highlightcolor="limegreen"),
                             opacity=0.8
                            ),
        create_rgb_surface(np.flipud(plot1), 
                             d2,
                             contours_z=dict(show=True, project_z=True, highlightcolor="limegreen"),
                             opacity=0.5
                            )],
    # layout_title_text="3D Surface"
)

fig.update_layout(
    scene = dict(
        xaxis = dict(nticks=4, range=[0,origin_rollCalib.shape[0]],),
        yaxis = dict(nticks=4, range=[0,origin_rollCalib.shape[1]],),
        zaxis = dict(nticks=4, range=[0,500],),
        ),
    width=1000,
    margin=dict(r=20, l=10, b=10, t=10))

fig.write_image("roll_3d_calib.png")
fig.write_html("roll_3d_calib.html")


# ax1.imshow(plot1, cmap = 'gray')
# ax1.imshow(temp1, cmap = 'gray')

# X = np.array(range(plot2.shape[1]))
# Y = np.array(range(plot2.shape[0]))
# ax2 = fig.add_subplot(1, 2, 2, projection = '3d')
# # ax2.set_title(f"scale of sashimi: {scale2:.2f}/mm^2, center at {center2//scale2}")
# ax2.contourf(X, Y, plot2, 100, zdir='z', offset=350, cmap="gray", alpha = 0.5)
# ax2.contourf(X, Y, origin_sashimiCalib, 100, zdir='z', offset=0, cmap="gray", alpha = 0.5)

d1 = genDepth_map(origin_sashimiCalib, 0)
d2 = genDepth_map(origin_rollCalib, 500)
fig = go.Figure(
    data=[
        create_rgb_surface(np.flipud(cv.cvtColor(origin_sashimiCalib, cv.COLOR_BGR2RGB)), 
                             d1,
                             contours_z=dict(show=True, project_z=True, highlightcolor="limegreen"),
                             opacity=0.8
                            ),
        create_rgb_surface(np.flipud(plot2),
                             d2,
                             contours_z=dict(show=True, project_z=True, highlightcolor="limegreen"),
                             opacity=0.5
                            )
        ],
    # layout_title_text="3D Surface"
)

fig.update_layout(
    scene = dict(
        xaxis = dict(nticks=4, range=[0,origin_sashimiCalib.shape[0]],),
        yaxis = dict(nticks=4, range=[0,origin_sashimiCalib.shape[1]],),
        zaxis = dict(nticks=4, range=[0,500],),
        ),
    width=1000,
    margin=dict(r=20, l=10, b=10, t=10))

fig.write_image("sashimi_3d_calib.png")
fig.write_html("sashimi_3d_calib.html")


# ax2.set_zlim(0, 500) 
# ax2.imshow(plot2, cmap = 'gray')
# ax2.imshow(temp2, cmap = 'gray')
# plt.show()
