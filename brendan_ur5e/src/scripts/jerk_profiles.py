from jerk_profile import JerkProfile
import numpy as np

class UDDU(JerkProfile):
	def __init__(self, jerk_min, jerk_max):
		# type: (float, float) -> None
		super(UDDU, self).__init__(jerk_min, jerk_max)
	
	@staticmethod
	def generate_steps(min, max):
		# type: (float, float) -> list[float]
		return [max, 0, min, 0, min, 0, max]
	
	@staticmethod
	def time_to_jerk_slope(duration):
		# type: (float) -> float
		predict = np.poly1d(np.array(
			[0.023323615160349715, 8.953635534704624e-16, -1.7272062389448268e-15, 1.0542875637864707e-15]
		))
		return predict(duration)


class UDUD(JerkProfile):
	def __init__(self, jerk_min, jerk_max):
		# type: (float, float) -> None
		super(UDUD, self).__init__(jerk_min, jerk_max)
	
	@staticmethod
	def generate_steps(min, max):
		# type: (float, float) -> list[float]
		return [max, 0, min, 0, max, 0, min]
	
	@staticmethod
	def time_to_jerk_slope(duration):
		# type: (float) -> float
		predict = np.poly1d(np.array(
			[0.04081632653061282, -3.355246560737928e-15, 6.241555680159846e-15, -3.755174617562978e-15]
		))
		return predict(duration)


class DDUU(JerkProfile):
	def __init__(self, jerk_min, jerk_max):
		# type: (float, float) -> None
		super(DDUU, self).__init__(jerk_min, jerk_max)

	@staticmethod
	def generate_steps(min, max):
		# type: (float, float) -> list[float]
		return [min, min, min, 0, max, max, max]
	
	@staticmethod
	def time_to_jerk_slope(duration):
		# type: (float) -> float
		predict = np.poly1d(np.array(
			[-0.12244897959183706, 2.585478661687749e-15, -5.875997922520806e-15, 4.061830171427839e-15]
		))
		return predict(duration)

class DDDU(JerkProfile):
	def __init__(self, jerk_min, jerk_max):
		# type: (float, float) -> None
		super(DDDU, self).__init__(jerk_min, jerk_max)

	@staticmethod
	def generate_steps(min, max):
		# type: (float, float) -> list[float]
		return [min, min, min, min, min, 0, max]

	@staticmethod
	def time_to_jerk_slope(duration):
		# type: (float) -> float
		predict = np.poly1d(np.array(
			[-0.16229348882410405, 1.7365741886309628e-14, -3.087844513560346e-14, 1.7166131814193233e-14]
		))
		return predict(duration)

