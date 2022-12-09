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

ricePhotoPose = Pose(350, 450, 300, -180, 0, 45)
riceStandardPose = Pose(400, 400, 180, -180, 0, 45)

rollPhotoPose = Pose(350, 450, 400, -180, 0, 45)
rollRiceStandardPose = Pose(400, 400, 200, -180, 0, 135)
cucumberStandardPose = Pose(400, 400, 200, -180, 0, 45)

cucumberPhotoPose = Pose(400, 150, 400, -180, 0, 45)
cucumberHeight = 140 + 2.5

salmonPhotoPose = Pose(325, 75, 400, -180, 0, 45)
salmonHeight = 140 + 2.5
salmonBias = 10 # 15

riceBowlPose = lambda place: Pose(295/sqrt(2) - 250/sqrt(2), 295/sqrt(2) + 250/sqrt(2), 160 if place.casefold() == "down".casefold() else 250, -180, 0, 45)
# plate, 180 for bowl
riceLowest = 135
riceSize = 50
riceDecay = 10
riceHeight = lambda target: 5 if target.casefold() == "roll".casefold() else 10
waterBowlPose = lambda place: Pose(475/sqrt(2) - 250/sqrt(2), 475/sqrt(2) + 250/sqrt(2), 140 if place.casefold() == "down".casefold() else 250, -180, 0, 45)
