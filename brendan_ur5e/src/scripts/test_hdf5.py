#!/usr/bin/env python

import h5py
import numpy as np
import rospy
from std_msgs.msg import String
#name: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]

num_joints = 6;

def get_joint_states(msg, pos_arr):
	cur_length_pos = 1	
	pos_msg = msg.split("position: [")
	ind_pos_msgs = pos_msg[1].split(",")
	for i in range(0, num_joints-1):
		rospy.loginfo('Reading in %f\n', float(ind_pos_msgs[i]))
		pos_arr[i, cur_length_pos] = float(ind_pos_msgs[i])
	rospy.spin()
		
	

if __name__ == '__main__':
	rospy.init_node('test_hdf5', anonymous=True)
	
	current_height = num_joints
	current_length = 1

	pos_arr = np.zeros( (current_height, current_length) )
	rospy.Subscriber('/joint_states', String, get_joint_states, pos_arr)

	rospy.loginfo('Finished reading in from /joint_states')

	fp = h5py.File ('file.h5', 'w')
	dset = fp.create_dataset("joint_positions", data=pos_arr)

	#grp = fp.create_group("subgroup_test")
	#dset2 = grp.create_dataset("subgroup_dataset", data=a)
	#dset2[0,0] = 1
	#rospy.loginfo('Dataset element [0,0]: %d', dset2[0,0])
	#rospy.loginfo('Dataset element [1,0]: %d', dset2[1,0])
	#dset2 = grp.create_dataset("subgroup_dataset", (current_height, current_length), dtype = 'f', data=a)
	#same as:
		#dset2 = f.create_dataset('subgroup_test/subgroup_dataset', (50,), dtype = 'f')
	fp.close()
