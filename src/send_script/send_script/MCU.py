# MCU communications
import serial

def roll(state):
	if state.casefold() == "Rick Astely".casefold():
		print("Never gonna give you up")
	else:
		pass

class MCU():
	def __init__(self, COM_PORT = None, BAUDRATE = 115200):
		if COM_PORT is None:
			from serial.tools.list_ports import comports
			for comport in comports():
				if comport.despcription.split('(')[0].strip() == "USB-SERIAL CH340":
					COM_PORT = comport.device
					break

		if COM_PORT is None:
			print("[Error] USB-SERIAL CH340 port not found")
			exit(-1)

		try:
			self.ser = serial.Serial(COM_PORT, BAUDRATE)
		except Exception as e:
			print("=" * 32)
			print(f"[Error] Cannot open serial port: {COM_PORT} with baudrate {BAUDRATE}")
			print("=" * 32)
			raise e

		self.platformPlace = 1 # 1: out, 0: in
		self.platformState = 0

	def __del__(self):
		self.ser.close()

	def getOrder(self):
		while True:
			if ser.in_waiting():
				data = ser.readline().decode().strip()
				if data.casefold() == "nigiri".casefold():
					return 0
				elif data.casefold() == "tekkamaki".casefold():
					return 1

	def platformAuto(self):
		self.ser.write(b'auto\r\n')

	def platformManual(self):
		self.ser.write(b'manual\r\n')

	def 

# auto
# roll
# half
# reset

# manual
# 0 0 0
# 90 90 90