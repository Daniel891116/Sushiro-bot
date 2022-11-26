#!/usr/bin/env python

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

from .robotComm import send_script, set_io, moveTo, gripper, takePic

class ImageSub(Node):
    def __init__(self, nodeName):
        super().__init__(nodeName)
        self.subscription = self.create_subscription(Image, 'techman_image', self.image_callback, 10)
    
    def image_callback(self, data):
        self.get_logger().info('Received image')
        
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(img_msg = data)

def main(args=None):
    rclpy.init(args=args)
    node = ImageSub('sushiro')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
