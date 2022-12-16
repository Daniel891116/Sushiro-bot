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
from .img_check import findSalmon, isRiceballOK, findCucumber, isMakkiRiceEnough
from .riceManager import riceManager
from .seaweedManager import seaweedManager

class myRobot(Node):
	"""class myRobot, deal with the comms of the robot"""
	def __init__(self, nodeName):
		super().__init__(nodeName)
		self.subscription = self.create_subscription(Image, 'techman_image', self.imageCallback, 10)

		self.arm_node = rclpy.create_node('arm')
		self.arm_cli = self.arm_node.create_client(SendScript, 'send_script')

		self.gripper_node = rclpy.create_node('gripper')
		self.gripper_cli = self.gripper_node.create_client(SetIO, 'set_io')

		self.bridge = CvBridge()
		self.img = None
		self.riceManager = riceManager()
		self.seaweedManager = seaweedManager()
		self.MCU = MCU("/dev/ttyUSB0")

		self.moveToPose(readyPose)
		self.platformState = "out" # assumed outside
		# 0: out, 1: in
		self.movePlatform("in")
		self.moveToPose(readyPose)
		self.gripper("open")

		# start sushi service
		self.callbackEvent = Event()
		inputThread = Thread(target = self.startSushiService)
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

	def block(self):
		self.imageCapture()

	# gripper
	def gripper(self, cmd, shake_freq = 0.1, shake_time = 5):
		if cmd.casefold() == "close".casefold():
			self.set_io(1.0)
		elif cmd.casefold() == "open".casefold():
			self.set_io(0.0)
		elif cmd.casefold() == "shake".casefold():
			# do it itself
			while not self.gripper_cli.wait_for_service(timeout_sec = 1.0):
				self.gripper_node.get_logger().info('Gripper service not availabe, waiting again...')
			io_cmd = SetIO.Request()
			io_cmd.module = 1
			io_cmd.type = 2
			io_cmd.pin = 0
			io_cmd.state = 0.0

			self.gripper_cli.call_async(io_cmd)
			sleep(0.5)
			for _ in range(int(shake_time/shake_freq)):
				io_cmd.state = 1.0 - io_cmd.state
				self.gripper_cli.call_async(io_cmd)
				sleep(shake_freq)
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
		self.moveToPose(self.currentPose, speed)

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

	def movePlatform(self, cmd):
		# in, out
		if self.platformState.casefold() != cmd.casefold():
			if self.platformState.casefold() == "in":
				# now inside, move outside
				self.gripper("open")
				self.moveToPose(platformInPose("up"))
				self.moveToPose(platformInPose("down"))
				self.moveToPose(platformIn2OutPose)
				self.moveToPose(platformBackInPose)
				self.moveToZ(250)
			elif self.platformState.casefold() == "out":
				# now outside, move inside
				self.gripper("close")
				self.moveToPose(platformOutPose("up"))
				self.moveToPose(platformOutPose("down"))
				self.moveToPose(platformOut2InPose)
				self.moveToZ(250)
			else:
				print("[Error] Unknown platform command !!!")
				return
		self.platformState = cmd
		self.MCU.setPlatformState(cmd)

	def nigiri(self):
		self.moveToPose(readyPose)
		self.movePlatform("out")

		# water
		self.moveToPose(waterBowlPose("up"))
		self.moveToPose(waterBowlPose("down"))
		self.block()
		self.gripper("open")
		self.gripper("shake", shake_freq = 0.2, shake_time = 4)
		self.moveToPose(waterBowlPose("shake"))
		self.gripper("shake", shake_freq = 0.1, shake_time = 7)

		# target: rice
		self.moveToPose(riceBowlPose("up"))
		self.gripper("open")
		self.moveToPose(self.riceManager.consult("rice"))
		self.gripper("close")
		self.block()
		self.moveToZ(250)
		self.moveToPose(riceStandardPose("up"))
		self.moveToPose(riceStandardPose("down"))
		self.block()
		self.gripper("open")
		self.block()
		self.gripper("shake", shake_freq = 0.15, shake_time = 6)
		self.moveToZ(250)
		self.movePlatform("out")
		self.block()
		self.MCU.nigiriRoll()

		# check rice
		while True:
			self.moveToPose(ricePhotoPose)
			result = isRiceballOK(self.imageCapture())
			if result[0]:
				print("Nice rice, next step")
				break
			else:
				# form by MCU
				self.movePlatform("out")
				self.MCU.nigiriRoll()
				# form by gripper
				dx, dy = result[1]
				currentRiceFormPose = deepcopy(riceFormPose(self.currentPose.x, self.currentPose.y, "up"))
				currentRiceFormPose.x += (-dx/sqrt(2) -dy/sqrt(2) -riceBias -gripperBias/sqrt(2))
				currentRiceFormPose.y += (-dx/sqrt(2) +dy/sqrt(2) -riceBias +gripperBias/sqrt(2))
				self.gripper("open")
				self.moveToPose(currentRiceFormPose)
				self.moveToPose(riceFormPose(currentRiceFormPose.x, currentRiceFormPose.y, "down"))
				self.gripper("close")
				self.block()
				self.gripper("open")
				self.moveToZ(250)

		# salmon
		self.moveToPose(salmonPhotoPose)
		result = findSalmon(self.imageCapture())
		if result[0]:
			self.gripper("open")
			dx, dy = result[1]
			self.currentPose.x += (-dx/sqrt(2) -dy/sqrt(2) -salmonBias/sqrt(2))
			self.currentPose.y += (-dx/sqrt(2) +dy/sqrt(2) -salmonBias/sqrt(2))
			self.currentPose.z = salmonHeight
			self.moveToPose(self.currentPose)
			self.gripper("close")
			self.block()
			self.moveToZ(250)

			# find rice
			self.moveToPose(ricePhotoPose)
			result = isRiceballOK(self.imageCapture(f"riceTest_{self.count}.png"))
			self.MCU.platformHalf()
			dx, dy = result[1]
			pose = deepcopy(self.currentPose)
			pose.x += (-dx/sqrt(2) -dy/sqrt(2) +20)
			pose.y += (-dx/sqrt(2) +dy/sqrt(2) -20)
			pose.z = 250
			pose.rz = 135
			self.moveToPose(pose)
			pose.x -= 30
			pose.y += 30
			pose.z -= 20
			self.moveToPose(pose)
			pose.x -= 35*2.9
			pose.y += 35*2.9
			pose.z -= 50
			pose.ry = 30
			self.moveToPose(pose)
			self.gripper("open")

			self.moveToPose(readyPose)

		else:
			print("[Error] Cannot find salmon !!! Continue...")

	def tekkamaki(self):
		# seaweed
		self.movePlatform("in")
		poses = self.seaweedManager.consult()
		for pose in poses:
			self.gripper(pose) if type(pose) == str else self.moveToPose(pose)
		self.moveToZ(250)

		# wash
		self.moveToPose(waterBowlPose("up"))
		self.moveToPose(waterBowlPose("down"))
		self.block()
		self.gripper("open")
		self.gripper("shake", shake_freq = 0.2, shake_time = 4)
		self.moveToPose(waterBowlPose("shake"))
		self.gripper("shake", shake_freq = 0.1, shake_time = 7)

		# pick rice - dip
		for m in [-2, -1, 0, 1, 2]:
			# pick rice
			# target: roll
			self.moveToPose(riceBowlPose("up"))
			self.gripper("open")
			self.moveToPose(self.riceManager.consult("roll"))
			self.gripper("close")
			self.block()
			self.moveToZ(250)
			self.moveToPose(rollRiceStandardPose(m, "up"))
			self.moveToPose(rollRiceStandardPose(m, "down"))
			self.block()
			self.gripper("open")
			self.block()
			self.gripper("shake", shake_freq = 0.15, shake_time = 6)
			self.moveToZ(250)

			# dip
			self.moveToPose(waterBowlPose("up"))
			self.moveToPose(waterBowlPose("down"))
			self.moveToPose(waterBowlPose("shake"))
			self.block()
			self.gripper("shake", shake_freq = 0.1, shake_time = 7)
			self.gripper("open")

		while True:
			self.moveToPose(rollPhotoPose)
			result = isMakkiRiceEnough(self.imageCapture(f"rollTest_{self.count}.png"))
			self.count += 1
			if result[0]:
				print("Nice roll, next step")
				break
			else:
				for destination in result[1]:
					# pick up rice
					# target: roll
					self.moveToPose(riceBowlPose("up"))
					self.gripper("open")
					self.moveToPose(self.riceManager.consult("roll"))
					self.gripper("close")
					self.block()
					self.moveToZ(250)

					# fill up the hole
					targetPost = deepcopy(ricePhotoPose)
					targetPost.rz = 135
					targetPost.x += (-destination/sqrt(2))
					targetPost.y += ( destination/sqrt(2))
					targetPost.z = 250
					self.moveToPose(targetPost)
					# self.moveToPose(rollRiceStandardPose(m, "down"))
					self.moveToZ(200)
					self.block()
					self.gripper("open")
					self.block()
					self.gripper("shake", shake_freq = 0.15, shake_time = 6)
					self.moveToZ(250)

					# dip
					self.moveToPose(waterBowlPose("up"))
					self.moveToPose(waterBowlPose("down"))
					self.moveToPose(waterBowlPose("shake"))
					self.block()
					self.gripper("shake", shake_freq = 0.1, shake_time = 7)
					self.gripper("open")

		# cucumber
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
			self.moveToPose(cucumberStandardPose("up"))
			self.moveToPose(cucumberStandardPose("down"))
			self.block()
			self.gripper("open")
			self.moveToZ(250)
			self.block()	
		else:
			print("[Error] Cannot find cucumbers !!! Continue...")

		# rolling
		self.MCU.tekkaRoll(self.platformState) # in
		self.moveToZ(180)
		self.moveToPose(rollRiceMovedPose("down"))
		self.moveToPose(rollRiceMovedPose("up"))
		self.movePlatform("out")
		self.block()
		self.MCU.tekkaRoll(self.platformState)
		self.moveToPose(readyPose)

	def startSushiService(self):
		self.block()
		print("Order mode")
		while True:
			order = self.MCU.getOrder()
			print("Order: " + order)
			if order.casefold() == "nigiri":
				self.nigiri()
			elif order.casefold() == "tekkamaki":
				self.tekkamaki()
