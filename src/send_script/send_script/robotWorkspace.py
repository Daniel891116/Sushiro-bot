# pose

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
		x: float = 400
		y: float = 400
		z: float = 300
		rx: float = -180
		ry: float = 0
		rz: float = 135
except ImportError:
	print("[Warning] dataclasses failed, using NamedTuple")
	from typing import NamedTuple

	class Pose(NamedTuple):
		"""pose class, NamedTuple"""
		x: float
		y: float
		z: float
		rx: float
		ry: float
		rz: float

readyPose = Pose(300, 300, 400, -180, 0, 135)
ricePhotoPose = Pose(350, 450, 300, -180, 0, 45)
rollPhotoPose = Pose(350, 450, 400, -180, 0, 45)
cucumberPhotoPose = Pose(400, 150, 300, -180, 0, 45)
salmonPhotoPose = Pose(325, 75, 300, -180, 0, 45)
