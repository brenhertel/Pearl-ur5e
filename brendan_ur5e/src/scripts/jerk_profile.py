class JerkProfile(object):
	def __init__(self, jerk_min, jerk_max):
		# type: (float, float) -> None
		self.steps = self.generate_steps(jerk_min, jerk_max)
	
	@staticmethod
	def generate_steps(min, max):
		# type: (float, float) -> list[float]
		return [0, 0, 0, 0, 0, 0, 0]
	
	@staticmethod
	def time_to_jerk_slope(duration):
		# type: (float) -> float
		return 0

# Template for new JerkProfile (add to jerk_profiles.py)
# class JerkProfileName(JerkProfile):
# 	def __init__(self, jerk_min, jerk_max):
#		# type: (float, float) -> None
# 		super(JerkProfileName, self).__init__(jerk_min, jerk_max)
#
# 	@staticmethod
# 	def generate_steps(min, max):
#		# type: (float, float) -> list[float]
# 		pass  # Include generation for an array of 7 elements representing jerk values over 7 constant timestamps
#
# 	@staticmethod
# 	def time_to_jerk_slope(duration):
#		# type: (float) -> float
# 		pass  # Include conversion between a time duration and a jerk/position slope
# 		# model_generator.py can be used to generate a numpy array that represents a model, and can be implemented in this function with the following:
# 		# 
# 		# predict = np.poly1d(np.array([array here]))
# 		# return predict(duration)

