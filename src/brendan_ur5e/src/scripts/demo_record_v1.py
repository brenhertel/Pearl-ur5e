#!/usr/bin/env python
import rospy
import roslib
import time
import h5py
import numpy as np
import message_filters
import math
import tf
import geometry_msgs.msg
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from tf.msg import tfMessage
from geometry_msgs.msg import WrenchStamped

class Arr_Struct():
	def __init__(self, time_arr, joint_arr, effort_arr, pos_arr, force_arr, torq_arr):
		self.time_arr = time_arr
		self.joint_arr = joint_arr
		self.effort_arr = effort_arr
		self.pos_arr = pos_arr
		self.force_arr = force_arr
		self.torq_arr = torq_arr
#		self.grip_status_arr = grip_status_arr	

def myhook():
	print("Starting shutdown")
	time.sleep(30)
	print("Finishing shutdown")

def wr_callback(wrmsg, (tfmsg, jsmsg, arr_struct)):	
	if jsmsg.header.stamp.secs == wrmsg.header.stamp.secs and jsmsg.header.stamp.nsecs == wrmsg.header.stamp.nsecs:
		arr_struct.time_arr = np.append(arr_struct.time_arr, [[jsmsg.header.stamp.secs], [jsmsg.header.stamp.nsecs]], axis=1)
		arr_struct.joint_arr = np.append(arr_struct.joint_arr, [[jsmsg.position[0]], [jsmsg.position[1]], [jsmsg.position[2]], [jsmsg.position[3]], [jsmsg.position[4]], [jsmsg.position[5]]], axis=1)
		arr_struct.effort_arr = np.append(arr_struct.effort_arr, [[jsmsg.effort[0]], [jsmsg.effort[1]], [jsmsg.effort[2]], [jsmsg.effort[3]], [jsmsg.effort[4]], [jsmsg.effort[5]]], axis=1)
		arr_struct.pos_arr = np.append(arr_struct.pos_arr, [[tfmsg.transforms[0].transform.translation.x], [tfmsg.transforms[0].transform.translation.y], [tfmsg.transforms[0].transform.translation.z], [tfmsg.transforms[0].transform.rotation.x], [tfmsg.transforms[0].transform.rotation.y], [tfmsg.transforms[0].transform.rotation.z], [tfmsg.transforms[0].transform.rotation.w]], axis=1)
		arr_struct.force_arr = np.append(arr_struct.force_arr, [[wrmsg.wrench.force.x], [wrmsg.wrench.force.y], [wrmsg.wrench.force.z]], axis=1)
		arr_struct.torq_arr = np.append(arr_struct.torq_arr, [[wrmsg.wrench.torque.x], [wrmsg.wrench.torque.y], [wrmsg.wrench.torque.z]], axis=1)
		rospy.loginfo("wr_callback full at %d", wrmsg.header.stamp.secs)

def tf_callback(tfmsg, (jsmsg, arr_struct)):	
	if jsmsg.header.stamp.secs == tfmsg.transforms[0].header.stamp.secs and jsmsg.header.stamp.nsecs == tfmsg.transforms[0].header.stamp.nsecs and tfmsg.transforms[0].child_frame_id == 'tool0_controller':
		rospy.Subscriber("wrench", WrenchStamped, wr_callback, (tfmsg, jsmsg, arr_struct))

def js_callback(jsmsg, arr_struct):
	#rospy.loginfo("js_callback at %d", jsmsg.header.stamp.secs)
	rospy.Subscriber("tf", tfMessage, tf_callback, (jsmsg, arr_struct))
   
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
	time_arr = np.empty([2, 1])	
	joint_arr = np.empty([6, 1])
	effort_arr = np.empty([6, 1])
	pos_arr = np.empty([7, 1])
	force_arr = np.empty([3, 1])
	torq_arr = np.empty([3, 1])
	arr_struct = Arr_Struct(time_arr, joint_arr, effort_arr, pos_arr, force_arr, torq_arr)
#	grip_status_arr = np.empty([1])
	rospy.init_node('demo_recorder', anonymous=True)
	rospy.Subscriber("joint_states", JointState, js_callback, arr_struct)
	
        # spin() simply keeps python from exiting until this node is stopped
	rospy.spin()
	rospy.on_shutdown(myhook)
	arr_struct.joint_arr = np.delete(arr_struct.joint_arr, 0, 1)
	arr_struct.effort_arr = np.delete(arr_struct.effort_arr, 0, 1)
	arr_struct.time_arr = np.delete(arr_struct.time_arr, 0, 1)
	arr_struct.pos_arr = np.delete(arr_struct.pos_arr, 0, 1)
	arr_struct.force_arr = np.delete(arr_struct.force_arr, 0, 1)
	arr_struct.torq_arr = np.delete(arr_struct.torq_arr, 0, 1)
	print(arr_struct.time_arr.shape)
	print(arr_struct.time_arr)
	print(arr_struct.joint_arr.shape)
	print(arr_struct.joint_arr)
	print(arr_struct.effort_arr.shape)
	print(arr_struct.effort_arr)
	print(arr_struct.pos_arr.shape)
	print(arr_struct.pos_arr)
	print(arr_struct.force_arr.shape)
	print(arr_struct.force_arr)
	print(arr_struct.torq_arr.shape)
	print(arr_struct.torq_arr)
#	grip_status_arr = np.delete(grip_status_arr, 0)

if __name__ == '__main__':
    listener()
