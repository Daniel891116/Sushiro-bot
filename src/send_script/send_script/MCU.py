# MCU communications
import serial

def getOrder():
    return -1

def roll(state):
    if state.casefold() == "Rick Astely".casefold():
        print("Never gonna give you up")
    else:
        pass

class MCU():
    def __init__(self, COM_PORT = '/dev/ttyACM0', BAUDRATE = 115200):
        self.ser = serial.Serial(COM_PORT, BAUDRATE)
        # try catch
