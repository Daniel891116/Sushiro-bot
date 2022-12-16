# MCU communications
import serial
from time import sleep

class MCU():
	def __init__(self, COM_PORT = None, BAUDRATE = 115200):
		if COM_PORT is None:
			from serial.tools.list_ports import comports
			for comport in comports():
				if comport.despcription.strip() == "USB Serial":
					COM_PORT = comport.device
					break

		if COM_PORT is None:
			print("[Error] USB-SERIAL CH340 port not found")
			exit(-1)

		try:
			self.ser = serial.Serial(COM_PORT, BAUDRATE)
			self.platformFlat();
		except Exception as e:
			print("=" * 32)
			print(f"[Error] Cannot open serial port: {COM_PORT} with baudrate {BAUDRATE}")
			print("=" * 32)
			raise e

		self.platformState = None

	def __del__(self):
		self.ser.close()

	def setPlatformState(self, cmd):
		self.platformState = cmd

	def platformAuto(self):
		self.ser.write(b'1')

	def getOrder(self):
		self.ser.write(b'2')
		while True:
			if self.ser.in_waiting:
				data = self.ser.readline().decode().strip()
				if data.casefold() == "nigiri".casefold():
					return "nigiri"
				elif data.casefold() == "tekka".casefold():
					return "tekkamaki"
				elif data.casefold() == "order mode".casefold():
					print("Enter order mode confirmed.")
				elif data.casefold() == "".casefold():
					continue
				else:
					print("[Warning] Unknown MCU message: \"" + data + "\", skipping")

	def nigiriRoll(self):
		# platform should be inside
		if self.platformState != "out":
			print("[Warning] platform should be outside while nigiriRoll !!!")
			return False
		self.ser.write(b'3')
		sleep(10)
		return True

	def tekkaRoll(self, mode):
		if self.platformState.casefold() != mode.casefold():
			print("[Error] platform state inconsistent with the tekka roll mode !!!")
			return False

		if mode.casefold() == "in".casefold():
			self.ser.write(b'4')
			sleep(15)
		else:
			self.ser.write(b'5')
			sleep(15)

	def platformHalf(self):
		self.ser.write(b'6')
		sleep(3)

	def platformFlat(self):
		self.ser.write(b'0')
		sleep(3)
