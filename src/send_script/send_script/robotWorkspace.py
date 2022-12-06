# pose
try:
	from waiting import wait, TimeoutExpired
except ImportError:
	import os
	os.system("sudo pip install waiting")
finally:
	from waiting import wait, TimeoutExpired

try:
	try:
		from dataclasses import dataclass
	except ImportError:
		import os
		os.system("sudo pip install dataclasses")
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
