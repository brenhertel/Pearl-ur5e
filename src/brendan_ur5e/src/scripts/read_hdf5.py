#!/usr/bin/env python

import h5py
import rospy
import roslib
import numpy as np

if __name__ == '__main__':
	rospy.init_node('read_hdf5')
	hf = h5py.File('recorded_demo Tue Nov 26 18:52:31 2019.h5', 'r')
	keys = hf.keys()
	print(keys)
	demo = hf.get('demo1')
	demo1_items = demo.items()
	print(demo1_items)
	joint_state_info = demo.get('joint_state_info')
	joint_state_items = joint_state_info.items()
	print(joint_state_items)
	joint_pos_info = joint_state_info.get('joint_positions')
	joint_pos_info = np.array(joint_pos_info)
	for i in range(0, np.size(joint_pos_info, 1)):
		print(joint_pos_info[0][i])
		print(joint_pos_info[1][i])
		print(joint_pos_info[2][i])
