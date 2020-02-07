#!/usr/bin/env python

import numpy as np
from std_msgs.msg import String

if __name__ == '__main__':
	rospy.init_node('test_string', anonymous=True)
	sample_str = "jake, jack, and joseph and james"
	print(sample_str)
	split_sample = sample_str.split(",")
	print(split_sample)
	split_again_sample = split_sample[2].split("j")
	print(split_again_sample)
	arr_test = np.empty([3, 1])
	print(arr_test)
	arr_test = np.append(arr_test, [[1.0], [2.0], [3.0]], axis=1)
	print(arr_test)
	arr_test = np.delete(arr_test, 0, 1)
	print(arr_test)
