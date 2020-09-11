#!/usr/bin/env python

import rospy
import roslib
import h5py
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import JointState

num_joints = 6
starting_length = 1

def get_joint_states(js, pos_arr):
	global starting_length
	global num_joints
	cur_pos_arr = np.zeros( (num_joints, starting_length) )
	for i in range(0, len(js.position)):
		cur_pos_arr[i, 0] = js.position[i]
		print(cur_pos_arr[i, 0])
	pos_arr = np.append(pos_arr, cur_pos_arr, axis = 1)	

#def callback(data):
#    rospy.loginfo("Joints: %s", data.name)
#    rospy.loginfo("Position0: %f", data.position[0])
#    rospy.loginfo("Position1: %f", data.position[1])
#    rospy.loginfo("Position2: %f", data.position[2])
#    rospy.loginfo("Position3: %f", data.position[3])
#    rospy.loginfo("Position4: %f", data.position[4])
#    rospy.loginfo("Position5: %f", data.position[5])
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
	global num_joints
	global starting_length
	rospy.init_node('joint_state_listener', anonymous=True)
	current_height = num_joints
	#current_length = starting_length
	pos_arr = np.zeros( (current_height, starting_length) )
	rospy.Subscriber("joint_states", JointState, get_joint_states, pos_arr)

    # spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
    listener()
