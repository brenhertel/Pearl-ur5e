#!/usr/bin/env python

import h5py
import numpy as np
import rospy
from std_msgs.msg import String
#name: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]

num_joints = 6;

def get_joint_states(msg, pos_arr):
	print('got here 3')	
	cur_length_pos = 1
	#rate = rospy.Rate(10) # 10hz	
	#while not rospy.is_shutdown():
	pos_msg = msg.split("position: [")
	print(pos_msg[1])
	ind_pos_msgs = pos_msg[1].split(",")
	for i in range(0, num_joints-1):
		rospy.loginfo('Reading in %f', float(ind_pos_msgs[i]))
		print(float(ind_pos_msgs[i]))
		pos_arr[i, cur_length_pos] = float(ind_pos_msgs[i])
	#rate.sleep()		
	
def understand_string(data):
	print('got here 3(2)')
	rospy.loginfo('Please help')	
	#rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)

#def nothing(msg):
	#print(len(msg.data))
	#for i  in range(0, len(msg.data)):
	#	print('nothing happens')
	#	rospy.loginfo('nothing happens here either')

if __name__ == '__main__':
	rospy.init_node('demo_record', anonymous=True)	
	rospy.loginfo('Start of demo recorder')	
	#nothing("")
	print('got here 1')
	current_height = num_joints
	current_length = 1
	pos_arr = np.zeros( (current_height, current_length) )
	print('got here 2')
	#rospy.Subscriber("/joint_states", String, get_joint_states, pos_arr)
	#rospy.Subscriber('/joint_states', String, understand_string)	
	rospy.Subscriber("joint_states", String, understand_string)
	#rospy.spin()
	print('got here 4')
	rospy.loginfo('Finished reading in from /joint_states')

	fp = h5py.File ('file.h5', 'w')
	dset = fp.create_dataset("joint_positions", data=pos_arr)

	fp.close()
