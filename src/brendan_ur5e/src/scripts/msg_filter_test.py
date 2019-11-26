#!/usr/bin/env python
import rospy
import roslib
import h5py
import numpy as np
import message_filters
import math
import tf
import geometry_msgs.msg
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from tf.msg import tfMessage


num_joints = 6
starting_length = 1
cur_time_s = 0
cur_time_ns = 0

#def get_joint_states(js, pos_arr):
#	global starting_length
#	global num_joints
#	cur_pos_arr = np.zeros( (num_joints, starting_length) )
#	for i in range(0, len(js.position)):
#		cur_pos_arr[i, 0] = js.position[i]
#		print(cur_pos_arr[i, 0])
#	pos_arr = np.append(pos_arr, cur_pos_arr, axis = 1)	

def js_callback(jsmsg):
	global cur_time_s
	cur_time_s = jsmsg.header.stamp.secs
	global cur_time_ns
	cur_time_ns = jsmsg.header.stamp.nsecs
	rospy.Subscriber("tf", tfMessage, tf_callback)
#	for i in range(0, len(jsmsg.position)):
#		rospy.loginfo("Joint %d position: %f", i, jsmsg.position[i])

def tf_callback(tfmsg):
	#child_id_reference = "tool0_controller";	
	#if (tfmsg.transforms[0].child_frame_id == child_id_reference):	
	#global cur_seq
	#rospy.loginfo("js seq: %d, tf seq: %d", cur_seq, tfmsg.transforms[0].header.seq)	
	global cur_time_s
	global cur_time_ns
	if cur_time_s == tfmsg.transforms[0].header.stamp.secs and cur_time_ns == tfmsg.transforms[0].header.stamp.nsecs:
		rospy.loginfo("Match!")
#	if (len(tfmsg.transforms) == 1):	
#		rospy.loginfo("Tool X: %f", tfmsg.transforms[0].transform.translation.x)
#		rospy.loginfo("Tool Y: %f", tfmsg.transforms[0].transform.translation.y)
#		rospy.loginfo("Tool Z: %f", tfmsg.transforms[0].transform.translation.z)
#		rospy.loginfo("Tool rot. X: %f", tfmsg.transforms[0].transform.rotation.x)
#		rospy.loginfo("Tool rot. Y: %f", tfmsg.transforms[0].transform.rotation.y)
#		rospy.loginfo("Tool rot. Z: %f", tfmsg.transforms[0].transform.rotation.z)
#		rospy.loginfo("Tool rot. W: %f", tfmsg.transforms[0].transform.rotation.w)
#	if (len(tfmsg.transforms) == 1):	
#		rospy.loginfo("Tool header: %s", tfmsg.transforms[0].child_frame_id)
   
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
	global num_joints
	global starting_length
	rospy.init_node('demo_recorder', anonymous=True)
	current_height = num_joints
	#current_length = starting_length
	pos_arr = np.zeros( (current_height, starting_length) )
	#rospy.Subscriber("joint_states", JointState, get_joint_states, pos_arr)

	#tft_msg = tf.Transformer(True, rospy.Duration(10.0))

	#js_sub = message_filters.Subscriber('joint_states', JointState)
	#tf_sub = message_filters.Subscriber('tf', tfMessage.transforms[0])
	#ts = message_filters.TimeSynchronizer([js_sub, tf_sub], 10)
	#ts.registerCallback(callback)
	rospy.Subscriber("joint_states", JointState, js_callback)
	
        # spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
    listener()
