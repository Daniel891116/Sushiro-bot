import rclpy
from rclpy.node import Node

import sys
sys.path.append('/home/robot/colcon_ws/install/tm_msgs/lib/python3.6/site-packages')
from tm_msgs.msg import *
from tm_msgs.srv import *

from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge
from math import sqrt
from threading import Thread, Event
from time import sleep
from copy import deepcopy

from .robotWorkspace import * # ...Pose
from .MCU import MCU
from .img_check import findSalmon, isRiceballOK, findCucumber

class myRobot(Node):
	"""class myRobot, deal with the comms of the robot"""
	def __init__(self, nodeName):
		super().__init__(nodeName)
		self.subscription = self.create_subscription(Image, 'techman_image', self.imageCallback, 10)

		self.arm_node = rclpy.create_node('arm')
		self.arm_cli = self.arm_node.create_client(SendScript, 'send_script')

		self.gripper_node = rclpy.create_node('gripper')
		self.gripper_cli = self.gripper_node.create_client(SetIO, 'set_io')

		self.gripper("open")
		self.moveToPose(readyPose)

		self.bridge = CvBridge()
		self.img = None

		# start sushi service
		self.callbackEvent = Event()
		inputThread = Thread(target = self.startCommandService)
		inputThread.start()

		# temp
		self.count = 0

	def __del__(self):
		self.arm_node.destroy_node()
		self.gripper_node.destroy_node()

	# image
	def imageCallback(self, data):
		self.img = self.bridge.imgmsg_to_cv2(img_msg = data)
		if self.img_filename:
			cv2.imwrite(self.img_filename, self.img)
		self.img_filename = None
		self.callbackEvent.set()

	def imageCapture(self, filename = None):
		self.img = None
		self.img_filename = filename
		self.send_script("Vision_DoJob(job1)")
		# wait for callback
		self.callbackEvent.wait()
		self.callbackEvent.clear()
		return self.img

	# gripper
	def gripper(self, cmd):
		if cmd.casefold() == "close".casefold():
			self.set_io(1.0)
		elif cmd.casefold() == "open".casefold():
			self.set_io(0.0)
		else:
			self.gripper_node.get_logger().info(f"[Error] Unknown gripper command: \'{cmd}\'")
			raise ValueError
		sleep(1)

	def set_io(self, state, pin = 0):
		# self.gripper_node = rclpy.create_node('gripper')
		# self.gripper_cli = self.gripper_node.create_client(SetIO, 'set_io')

		while not self.gripper_cli.wait_for_service(timeout_sec = 1.0):
			self.gripper_node.get_logger().info('Gripper service not availabe, waiting again...')

		io_cmd = SetIO.Request()
		io_cmd.module = 1
		io_cmd.type = 1
		io_cmd.pin = pin
		io_cmd.state = state
		self.gripper_cli.call_async(io_cmd)
		# self.gripper_node.destroy_node()

	def try_pwm(self, pin = 0):
		# self.gripper_node = rclpy.create_node('gripper')
		# self.gripper_cli = self.gripper_node.create_client(SetIO, 'set_io')

		while not self.gripper_cli.wait_for_service(timeout_sec = 1.0):
			self.gripper_node.get_logger().info('Gripper service not availabe, waiting again...')

		io_cmd = SetIO.Request()
		io_cmd.module = 1
		io_cmd.type = 1
		io_cmd.pin = pin
		io_cmd.state = 1.0
		for _ in range(100):
			io_cmd.state = 1.0 - io_cmd.state
			self.gripper_cli.call_async(io_cmd)
			sleep(0.1)
		# self.gripper_node.destroy_node()

	# move
	def send_script(self, script):
		# self.arm_node = rclpy.create_node('arm')
		# self.arm_cli = self.arm_node.create_client(SendScript, 'send_script')

		while not self.arm_cli.wait_for_service(timeout_sec=1.0):
			self.arm_node.get_logger().info('Arm service not availabe, waiting again...')

		move_cmd = SendScript.Request()
		move_cmd.script = script
		self.arm_cli.call_async(move_cmd)
		# self.arm_node.destroy_node()

	def moveToXYZ(self, x = None, y = None, z = None, rx = None, ry = None, rz = None, speed = 100):
		self.currentPose.x = x if x is not None else self.currentPose.x
		self.currentPose.y = y if y is not None else self.currentPose.y
		self.currentPose.z = z if z is not None else self.currentPose.z
		self.currentPose.rx = rx if rx is not None else self.currentPose.rx
		self.currentPose.ry = ry if ry is not None else self.currentPose.ry
		self.currentPose.rz = rz if rz is not None else self.currentPose.rz
		self.send_script(f"Line(\"CPP\",{x}, {y}, {z}, {rx}, {ry}, {rz}, {speed},200,0,false)")

	def moveToPose(self, pose, speed = 100):
		self.currentPose = deepcopy(pose)
		self.send_script(f"Line(\"CPP\",{pose.x}, {pose.y}, {pose.z}, {pose.rx}, {pose.ry}, {pose.rz}, {speed},200,0,false)")

	def moveToX(self, x, speed = 100):
		self.currentPose.x = x
		self.moveToPose(self.currentPose, speed)

	def moveToY(self, y, speed = 100):
		self.currentPose.y = y
		self.moveToPose(self.currentPose, speed)

	def moveToZ(self, z, speed = 100):
		self.currentPose.z = z
		self.moveToPose(self.currentPose, speed)

	def moveToRX(self, rx, speed = 100):
		self.currentPose.rx = rx
		self.moveToPose(self.currentPose, speed)

	def moveToRY(self, ry, speed = 100):
		self.currentPose.ry = ry
		self.moveToPose(self.currentPose, speed)

	def moveToRZ(self, rz, speed = 100):
		self.currentPose.rz = rz
		self.moveToPose(self.currentPose, speed)

	def moveAddX(self, addx, speed = 100):
		self.currentPose.x += addx
		self.moveToPose(self.currentPose, speed)

	def moveAddY(self, addy, speed = 100):
		self.currentPose.y += addy
		self.moveToPose(self.currentPose, speed)

	def moveAddZ(self, addz, speed = 100):
		self.currentPose.z += addz
		self.moveToPose(self.currentPose, speed)

	def moveAddRX(self, addrx, speed = 100):
		self.currentPose.rx += addrx
		self.moveToPose(self.currentPose, speed)

	def moveAddRY(self, addry, speed = 100):
		self.currentPose.ry += addry
		self.moveToPose(self.currentPose, speed)

	def moveAddRZ(self, addrz, speed = 100):
		self.currentPose.rz += addrz
		self.moveToPose(self.currentPose, speed)

	# command service
	def startCommandService(self):
		count = 0
		while True:
			command = input("Command: ")
			if command[0] == 'v':
				print("Vision")
				self.imageCapture(filename = f"visionTest_{count}.jpg")
				count += 1
			elif command[0] == 'g':
				self.set_io(float(command[1]))
			elif command[0] == 'x':
				x = float(command[2:])
				print(f"x: {x}")
				self.moveToX(x)
			elif command[0] == 'y':
				y = float(command[2:])
				print(f"y: {y}")
				self.moveToY(y)
			elif command[0] == 'z':
				z = float(command[2:])
				print(f"z: {z}")
				self.moveToZ(z)
			elif command[0] == 'r':
				if command[1] == 'x':
					rx = float(command[3:])
					print(f"rx: {rx}")
					self.moveToRX(rx)
				elif command[1] == 'y':
					ry = float(command[3:])
					print(f"ry: {ry}")
					self.moveToRY(ry)
				elif command[1] == 'z':
					rz = float(command[3:])
					print(f"rz: {rz}")
					self.moveToRZ(rz)
			elif command[0] == 'p':
				if command[1] == 'r':
					self.moveToPose(ricePhotoPose)
					print(isRiceballOK(self.imageCapture()))
				elif command[1] == 'l':
					self.moveToPose(rollPhotoPose)
					self.imageCapture(f"roll1_{self.count}.png")
					self.count += 1
					# print
				elif command[1] == 'c':
					self.moveToPose(cucumberPhotoPose)
					result = findCucumber(self.imageCapture())
					if result[0]:
						self.gripper("open")
						dx, dy = result[1]
						self.currentPose.x += (-dx/sqrt(2) -dy/sqrt(2))
						self.currentPose.y += (-dx/sqrt(2) +dy/sqrt(2))
						self.currentPose.z = cucumberHeight
						self.moveToPose(self.currentPose)
						self.gripper("close")
						self.moveToZ(250)
					else:
						print("[Error] Cannot find cucumbers !!! Continue...")
				elif command[1] == 's':
					self.moveToPose(salmonPhotoPose)
					result = findSalmon(self.imageCapture(f"salmonTest_{self.count}.png"))
					if result[0]:
						self.gripper("open")
						dx, dy = result[1]
						self.currentPose.x += (-dx/sqrt(2) -dy/sqrt(2) -salmonBias/sqrt(2))
						self.currentPose.y += (-dx/sqrt(2) +dy/sqrt(2) -salmonBias/sqrt(2))
						self.currentPose.z = salmonHeight
						self.moveToPose(self.currentPose)
						self.gripper("close")
						self.moveToZ(250)
						self.count += 1
					else:
						print("[Error] Cannot find salmon !!! Continue...")
			elif command[0] == '?':
				print(self.currentPose)
			elif command[0] == 't':
				self.try_pwm()
			elif command[0] == 'q':
				# quit
				break
			else:
				continue

	# sushi service