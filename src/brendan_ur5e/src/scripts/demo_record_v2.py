#!/usr/bin/env python

#all necessarry imports
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
from brendan_ur5e.srv import shutdown_request

recording = 1

#global variable to keep track of time of the recorded data
record_time = 0

name = 'recorded_demo ' + time.ctime() + '.h5'

def save_demo(arr_struct, demo_number):
	global name
	fp = h5py.File (name, 'w')
	demo_name = 'demo' + str(demo_number)
	dset_time = fp.create_dataset(demo_name + '/time_info/time_data', data=arr_struct.time_arr)
	dset_joint = fp.create_dataset(demo_name + '/joint_state_info/joint_positions', data=arr_struct.joint_arr)
	dset_eff = fp.create_dataset(demo_name + '/joint_state_info/joint_effort', data=arr_struct.effort_arr)
	dset_pos = fp.create_dataset(demo_name + '/tf_info/pos_rot_data', data=arr_struct.pos_arr)
	dset_force = fp.create_dataset(demo_name + '/gripper_info/force_data', data=arr_struct.force_arr)
	dset_torq = fp.create_dataset(demo_name + '/gripper_info/torque_data', data=arr_struct.torq_arr)
	fp.close()

#service that shuts down the node after a catch-up period
def shutdown_self(req):
	if req.a == 1:
		rospy.loginfo("recieved request to shutdown")
		time_at_req = int(round(time.time()))		
		while (time_at_req >= record_time):
			rospy.loginfo("time_at_req: %d, record_time: %d", time_at_req, record_time)
			rospy.sleep(0.05)
		rospy.loginfo("final shutdown")
		global recording
		recording = 0
		#shutdown_sub_node()


#instead of passing all the indidvidual arrays around, I use a structure of arrays to keep them all in the same place
class Arr_Struct():
	def __init__(self, time_arr, joint_arr, effort_arr, pos_arr, force_arr, torq_arr):
		self.time_arr = time_arr
		self.joint_arr = joint_arr
		self.effort_arr = effort_arr
		self.pos_arr = pos_arr
		self.force_arr = force_arr
		self.torq_arr = torq_arr
#I'll add in the code for understanding the gripper later
#		self.grip_status_arr = grip_status_arr	

#Once the wrench subscriber is called, and all the time stamps line up (the first two time stamps must have lined up for the third to be called), it writes all the data the messages have been carrying to the arrays. This ensures time synchronous data, without going back and processing later if data needs to be cleaned up.
def wr_callback(wrmsg, (tfmsg, jsmsg, arr_struct)):
	global recording	
	if jsmsg.header.stamp.secs == wrmsg.header.stamp.secs and jsmsg.header.stamp.nsecs == wrmsg.header.stamp.nsecs and recording == 1:
		arr_struct.time_arr = np.append(arr_struct.time_arr, [[jsmsg.header.stamp.secs], [jsmsg.header.stamp.nsecs]], axis=1)
		arr_struct.joint_arr = np.append(arr_struct.joint_arr, [[jsmsg.position[0]], [jsmsg.position[1]], [jsmsg.position[2]], [jsmsg.position[3]], [jsmsg.position[4]], [jsmsg.position[5]]], axis=1)
		arr_struct.effort_arr = np.append(arr_struct.effort_arr, [[jsmsg.effort[0]], [jsmsg.effort[1]], [jsmsg.effort[2]], [jsmsg.effort[3]], [jsmsg.effort[4]], [jsmsg.effort[5]]], axis=1)
		arr_struct.pos_arr = np.append(arr_struct.pos_arr, [[tfmsg.transforms[0].transform.translation.x], [tfmsg.transforms[0].transform.translation.y], [tfmsg.transforms[0].transform.translation.z], [tfmsg.transforms[0].transform.rotation.x], [tfmsg.transforms[0].transform.rotation.y], [tfmsg.transforms[0].transform.rotation.z], [tfmsg.transforms[0].transform.rotation.w]], axis=1)
		arr_struct.force_arr = np.append(arr_struct.force_arr, [[wrmsg.wrench.force.x], [wrmsg.wrench.force.y], [wrmsg.wrench.force.z]], axis=1)
		arr_struct.torq_arr = np.append(arr_struct.torq_arr, [[wrmsg.wrench.torque.x], [wrmsg.wrench.torque.y], [wrmsg.wrench.torque.z]], axis=1)
		global record_time
		record_time = wrmsg.header.stamp.secs
		rospy.loginfo("wr_callback full at %d", record_time)

#if tf and joint_states are at the same time, and tf has tool0 data, call on wrench
def tf_callback(tfmsg, (jsmsg, arr_struct)):	
	if jsmsg.header.stamp.secs == tfmsg.transforms[0].header.stamp.secs and jsmsg.header.stamp.nsecs == tfmsg.transforms[0].header.stamp.nsecs and tfmsg.transforms[0].child_frame_id == 'tool0_controller':
		#global sub_wr
		sub_wr = rospy.Subscriber("wrench", WrenchStamped, wr_callback, (tfmsg, jsmsg, arr_struct))

#just subscribes to tf and passes that callback the joint state message, and thus the data contained in it
def js_callback(jsmsg, arr_struct):
	#global sub_tf
	sub_tf = rospy.Subscriber("tf", tfMessage, tf_callback, (jsmsg, arr_struct))
   
def listener(demo_number):
	#create the initial empty arrays (creates a first row that will be removed later for clean data
	time_arr = np.empty([2, 1])	
	joint_arr = np.empty([6, 1])
	effort_arr = np.empty([6, 1])
	pos_arr = np.empty([7, 1])
	force_arr = np.empty([3, 1])
	torq_arr = np.empty([3, 1])
	#put all the arrays into a structure for easy passing
	arr_struct = Arr_Struct(time_arr, joint_arr, effort_arr, pos_arr, force_arr, torq_arr)
#	grip_status_arr = np.empty([1])
	#subscribe to joint_states, passing the structure of arrays
	#global sub_js
	sub_js = rospy.Subscriber("joint_states", JointState, js_callback, arr_struct)
	#create a service that shuts down the node when called. This is because the data being put into the array is behind real-time data, so the service actually shuts down the node once all the data that existed and the time of shutdown call has been processed
	srvc = rospy.Service('end_demo', shutdown_request, shutdown_self)
        # spin() simply keeps python from exiting until this node is stopped
	global recording
	while recording == 1:	
		time.sleep(1)
	#remove the empty first row of each array
	arr_struct.joint_arr = np.delete(arr_struct.joint_arr, 0, 1)
	arr_struct.effort_arr = np.delete(arr_struct.effort_arr, 0, 1)
	arr_struct.time_arr = np.delete(arr_struct.time_arr, 0, 1)
	arr_struct.pos_arr = np.delete(arr_struct.pos_arr, 0, 1)
	arr_struct.force_arr = np.delete(arr_struct.force_arr, 0, 1)
	arr_struct.torq_arr = np.delete(arr_struct.torq_arr, 0, 1)
#	grip_status_arr = np.delete(grip_status_arr, 0)
	#debug stuff
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
	save = raw_input('Would you like to save this demo? (y/n)')
	rospy.loginfo("You entered: %s", save)
	if (save == 'y'):
		save_demo(arr_struct, demo_number)

def begin_listener(req):
	demo_number = 1
	if req.a == 1:
		listener(demo_number)
		demo_number  = demo_number + 1

if __name__ == '__main__':
	#create the node
	rospy.init_node('demo_recorder', anonymous=True)
	srvc = rospy.Service('start_demo', shutdown_request, begin_listener)	
	while not rospy.is_shutdown():
		rospy.spin()
    		
