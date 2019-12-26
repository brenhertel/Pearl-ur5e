#!/usr/bin/env python

import h5py
import rospy
import roslib
import numpy as np

if __name__ == '__main__':
	rospy.init_node('read_hdf5')
	hf = h5py.File('recorded_demo Thu Dec 26 14:14:19 2019.h5', 'r')
	keys = hf.keys()
	print(keys)
	demo = hf.get('demo1')
	demo1_items = demo.items()
	print(demo1_items)
	joint_state_info = demo.get('joint_state_info')
	joint_state_items = joint_state_info.items()
	print(joint_state_items)
	tf_info = demo.get('tf_info')
	joint_pos_info = joint_state_info.get('joint_positions')
	joint_pos_info = np.array(joint_pos_info)
	pos_rot_data = tf_info.get('pos_rot_data')
	pos_rot_data = np.array(pos_rot_data)
	hf.close()
	for i in range(0, np.size(pos_rot_data, 1)):
		print("translation x: ", pos_rot_data[0][i])
		print("translation y: ", pos_rot_data[1][i])
		print("translation z: ", pos_rot_data[2][i])
		print("rotation x: ", pos_rot_data[3][i])
		print("rotation y: ", pos_rot_data[4][i])
		print("rotation z: ", pos_rot_data[5][i])
		print("rotation w: ", pos_rot_data[6][i])
		print
