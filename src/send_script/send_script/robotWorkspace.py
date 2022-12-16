# pose
from math import sqrt

try:
	try:
		from dataclasses import dataclass
	except ImportError:
		import os
		os.system("sudo pip3 install dataclasses")
	finally:
		from dataclasses import dataclass

	@dataclass
	class Pose:
		"""pose class, dataclass"""
		_x: float = 400
		_y: float = 400
		_z: float = 300
		_rx: float = -180
		_ry: float = 0
		_rz: float = 135

		@property
		def x(self) -> float: return self._x
		@property
		def y(self) -> float: return self._y
		@property
		def z(self) -> float: return self._z
		@property
		def rx(self) -> float: return self._rx
		@property
		def ry(self) -> float: return self._ry
		@property
		def rz(self) -> float: return self._rz

		@x.setter
		def x(self, value: float) -> None: self._x = value
		@y.setter
		def y(self, value: float) -> None: self._y = value
		@z.setter
		def z(self, value: float) -> None: self._z = value
		@rx.setter
		def rx(self, value: float) -> None: self._rx = value
		@ry.setter
		def ry(self, value: float) -> None: self._ry = value
		@rz.setter
		def rz(self, value: float) -> None: self._rz = value

except ImportError:
	print("[Warning] dataclasses failed, using NamedTuple")
	from typing import NamedTuple

	class Pose(NamedTuple):
		"""pose class, NamedTuple"""
		_x: float
		_y: float
		_z: float
		_rx: float
		_ry: float
		_rz: float

		@property
		def x(self) -> float: return self._x
		@property
		def y(self) -> float: return self._y
		@property
		def z(self) -> float: return self._z
		@property
		def rx(self) -> float: return self._rx
		@property
		def ry(self) -> float: return self._ry
		@property
		def rz(self) -> float: return self._rz

		@x.setter
		def x(self, value: float) -> None: self._x = value
		@y.setter
		def y(self, value: float) -> None: self._y = value
		@z.setter
		def z(self, value: float) -> None: self._z = value
		@rx.setter
		def rx(self, value: float) -> None: self._rx = value
		@ry.setter
		def ry(self, value: float) -> None: self._ry = value
		@rz.setter
		def rz(self, value: float) -> None: self._rz = value

readyPose = Pose(300, 300, 400, -180, 0, 135)

ricePhotoPose = Pose(360, 460, 400, -180, 0, 45)
riceStandardPose = lambda place: Pose(410, 410, 180 if place.casefold() == "down".casefold() else 250, -180, 0, 45)
riceFormPose = lambda x, y, place: Pose(x, y, 158 if place.casefold() == "down".casefold() else 250, 135, 0, 135)
riceBias = 35
gripperBias = 25

rollPhotoPose = Pose(360, 460, 400, -180, 0, 45)
rollRiceGap = 20
rollRiceStandardPose = lambda m, place: Pose(410 - m * rollRiceGap, 410 + m * rollRiceGap, 200 if place.casefold() == "down".casefold() else 250, -180, 0, 135)
rollRiceMovedPose = lambda place: Pose(380, 380, 180 if place.casefold() == "down".casefold() else 250, -180, 0, 45)

cucumberStandardPose = lambda place: Pose(410, 410, 200 if place.casefold() == "down".casefold() else 250, -180, 0, 45)
cucumberPhotoPose = Pose(410, 160, 400, -180, 0, 45)
cucumberHeight = 140 + 1

salmonPhotoPose = Pose(350, 100, 400, -180, 0, 45)
salmonHeight = 140 + 7.5 # tune larger if collision
salmonBias = 5 # 10 # 15
# salmonLength = 

riceBowlPose = lambda place: Pose(285/sqrt(2) - 250/sqrt(2), 285/sqrt(2) + 250/sqrt(2), 160 if place.casefold() == "down".casefold() else 250, -180, 0, 45)
# riceBowlPose = lambda place: Pose(285/sqrt(2) - 250/sqrt(2), 285/sqrt(2) + 250/sqrt(2), 141 if place.casefold() == "down".casefold() else 250, -180, 0, 45) # temp
# plate, 180 for bowl
riceLowest = 135
riceSize = 50
riceDecay = 10
riceHeight = lambda target: 3 if target.casefold() == "roll".casefold() else 10
waterBowlPose = lambda place: Pose(475/sqrt(2) - 240/sqrt(2), 475/sqrt(2) + 240/sqrt(2), 140 if place.casefold() == "down".casefold() else 250, 
	-165 if place.casefold() == "shake".casefold() else -180, 15 if place.casefold() == "shake".casefold() else 0, 45)

platformOutPose = lambda place: Pose(485, 485, 150 if place.casefold() == "down" else 250, -180, 0, 45)
platformOut2InPose = Pose(459, 459, 150, -180, 0, 45)

platformInPose = lambda place: Pose(513, 493, 133 if place.casefold() == "down" else 250, -180, 30, 45)
platformIn2OutPose = Pose(530, 510, 133, -180, 30, 45)
platformBackInPose = Pose(528, 508, 133, -180, 30, 45)
