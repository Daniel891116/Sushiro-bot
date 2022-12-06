#!/usr/bin/env python

# THE MAIN PACKAGE

import rclpy
from rclpy.node import Node

import sys
sys.path.append('/home/robot/colcon_ws/install/tm_msgs/lib/python3.6/site-packages')
from tm_msgs.msg import *
from tm_msgs.srv import *

from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge
from time import sleep
import math
import numpy as np
from skimage import measure as skiMeasure
from skimage import color
import matplotlib.pyplot as plt

# from .robotComm import send_script, set_io, moveTo, gripper, takePic
from .robotWorkspace import Pose
from .myRobot import myRobot

def main(args=None):
	rclpy.init(args=args)
	node = myRobot('sushiro')
	try:
		rclpy.spin(node)
	except Exception as e:
		node.destroy_node()
		rclpy.shutdown()
		raise e

if __name__ == '__main__':
	main()
