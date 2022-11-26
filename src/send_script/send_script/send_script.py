#!/usr/bin/env python

import rclpy

import sys
sys.path.append('/home/robot/colcon_ws/install/tm_msgs/lib/python3.6/site-packages')
from tm_msgs.msg import *
from tm_msgs.srv import *
import cv2
from time import sleep

from .robotComm import send_script, set_io, moveTo, gripper, takePic

# import related stuff
# https://answers.ros.org/question/367793/including-a-python-module-in-a-ros2-package/

def main(args=None):

    rclpy.init(args=args)

    #--- move command by joint angle ---#
    # script = 'PTP(\"JPP\",45,0,90,0,90,0,35,200,0,false)'

    #--- move command by end effector's pose (x,y,z,a,b,c) ---#
    # targetP1 = "398.97, -122.27, 748.26, -179.62, 0.25, 90.12"s

    # Initial camera position for taking image (Please do not change the values)
    # For right arm: targetP1 = "230.00, 230, 730, -180.00, 0.0, 135.00"
    # For left  arm:rclpy targetP1 = "350.00, 350, 730, -180.00, 0.0, 135.00"
    #targetP1 = "230.00, 230, 730, -180.00, 0.0, 135.00"
    #targetP2 = "300.00, 100, 500, -180.00, 0.0, 135.00"
    #script1 = f"Line(\"CPP\",{targetP1},100,200,0,false)"
    #script2 = f"Line(\"CPP\",{targetP2},100,200,0,false)"
    # script1 = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
    # script2 = "PTP(\"CPP\","+targetP2+",100,200,0,false)"
    #send_script(script1)
    #send_script(script2)

# What does Vision_DoJob do? Try to use it...
# -------------------------------------------------
    #send_script("Vision_DoJob(job1)")
    #cv2.waitKey(1)
    #send_script("Vision_DoJob(job1)")
    #cv2.waitKey(1)
#--------------------------------------------------

    #sleep(15)
    #set_io(1.0) # 1.0: close gripper, 0.0: open gripper
    #sleep(1)
    #set_io(0.0)
    #rclpy.shutdown()

    import_testing()

    try:
        while(True):
            target = input("Input target: ")
            if(target[0] == 'g'):
                target = target[1:]
                print("Gripper: " + target)
                set_io(float(target))
                sleep(1)
            elif(target[0] == 'v'):
                send_script("Vision_DoJob(job1)")
                print("Vision")
                sleep(1)
            else:
                print("Target: " + target)
                script = f"Line(\"CPP\",{target},100,200,0,false)"
                send_script(script)
                sleep(1)
    except Exception as e:
        print(e)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
