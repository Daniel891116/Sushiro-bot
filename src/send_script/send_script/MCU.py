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

	def __del__(self):
		self.ser.close()

	def getOrder(self):
		while True:
			if self.ser.in_waiting():
				data = self.ser.readline().decode().strip()
				if data.casefold() == "nigiri".casefold():
					return 0
				elif data.casefold() == "tekkamaki".casefold():
					return 1

	def platformAuto(self):
		self.ser.write(b'1')

	def platformManual(self):
		self.ser.write(b'2')

	def platformDegree(self, inn, mid, out):
		self.ser.write(f'{inn}_{mid}_{out}')

	def nigiriRoll(self):
		self.ser.write(b'3')
		sleep(10)

	def tekkaRoll(self, mode):
		if mode.casefold() == "in".casefold():
			self.ser.write(b'4')
		else:
			self.ser.write(b'5')

	def platformHalf(self):
		self.ser.write(b'6')
		sleep(3)

	def platformFlat(self):
		self.ser.write(b'0')

# auto
# roll
# half
# reset

# manual
# 10 10 10
# 170 170 170

# nigiri
# tekkamaki