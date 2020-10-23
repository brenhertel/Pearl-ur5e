#!/usr/bin/env python

#all necessary imports
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
from brendan_ur5e.msg import gripper_pos
from brendan_ur5e.srv import gripper_data_request

# create subscribers for joint state, tf, wrench
#each subscriber writes data and timestamps to file (txt)
#if demo is chosen to be saved, read from the txt files
#compare timestamps
#if timestamps match, write to an array
#save array to h5 file

def gr_callback(grmsg, gr_file):
	gr_file.write(str(grmsg.header.stamp.secs) + ', ' + str(grmsg.header.stamp.nsecs) + ', ' + str(grmsg.gripper_pos) + '\n')

def wr_callback(wrmsg, wr_file):
	wr_file.write(str(wrmsg.header.stamp.secs) + ', ' + str(wrmsg.header.stamp.nsecs) + ', ' + str(wrmsg.wrench.force.x) + ', ' + str(wrmsg.wrench.force.y) + ', ' + str(wrmsg.wrench.force.z) + ', ' + str(wrmsg.wrench.torque.x) + ', ' + str(wrmsg.wrench.torque.y) + ', ' + str(wrmsg.wrench.torque.z) + '\n')
	

#if tf and joint_states are at the same time, and tf has tool0 data, call on wrench
def tf_callback(tfmsg, tf_file):	
	if tfmsg.transforms[0].child_frame_id == 'tool0_controller':
		tf_file.write(str(tfmsg.transforms[0].header.stamp.secs) + ', ' + str(tfmsg.transforms[0].header.stamp.nsecs) + ', ' + str(tfmsg.transforms[0].transform.translation.x) + ', ' + str(tfmsg.transforms[0].transform.translation.y) + ', ' + str(tfmsg.transforms[0].transform.translation.z) + ', ' + str(tfmsg.transforms[0].transform.rotation.x) + ', ' + str(tfmsg.transforms[0].transform.rotation.y) + ', ' + str(tfmsg.transforms[0].transform.rotation.z) + ', ' + str(tfmsg.transforms[0].transform.rotation.w) + '\n')
#just subscribes to tf and passes that callback the joint state message, and thus the data contained in it
def js_callback(jsmsg, js_file):
	js_file.write(str(jsmsg.header.stamp.secs) + ', ' + str(jsmsg.header.stamp.nsecs) + ', ' + str(jsmsg.position) + ', ' + str(jsmsg.velocity) + ', ' + str(jsmsg.effort) + '\n')
   
def save_demo():
	js_fp = open('joint_data.txt', 'r')
	tf_fp = open('tf_data.txt', 'r')
	wr_fp = open('wrench_data.txt', 'r')
	gr_fp = open('gripper_data.txt', 'r')
	js_time_arr = np.zeros((1, 2))
	js_pos_arr = np.zeros((1, 6))
	js_vel_arr = np.zeros((1, 6))
	js_eff_arr = np.zeros((1, 6))
	tf_time_arr = np.zeros((1, 2))
	tf_pos_arr = np.zeros((1, 3))
	tf_rot_arr = np.zeros((1, 4))
	wr_time_arr = np.zeros((1, 2))
	wr_force_arr = np.zeros((1, 3))
	wr_torq_arr = np.zeros((1, 3))
	gr_time_arr = np.zeros((1, 2))
	gr_pos_arr = np.zeros((1, 1))
	#for ln in js_fp:
	#    l_split = ln.split(',')
	#    js_time_arr
	for ln in gr_fp:
	    print(ln)
	    
'''
>>> import numpy as np
>>> strings = '1, 65, 93093, 2'
>>> strings
'1, 65, 93093, 2'
>>> l_split = strings.split(',')
>>> l_split
['1', ' 65', ' 93093', ' 2']
>>> np.array(l_split)
array(['1', ' 65', ' 93093', ' 2'], dtype='|S6')
>>> np.array(l_split).astype(int)
array([    1,    65, 93093,     2])
>>> 
'''
   
def end_record(js_file, tf_file, wr_file, gr_file):
    print(time.time())
    
    js_file.close()
    tf_file.close()
    wr_file.close()
    gr_file.close()
	
    save = raw_input('Would you like to save this demo? (y/n)')
    rospy.loginfo("You entered: %s", save)  
    if (save == 'y'):
    	save_demo()

    cont = raw_input('Would you like to start another demo? (y/n)')
    rospy.loginfo("You entered: %s", cont)
    if (cont == 'y'):
        demo_recorder()
   
def demo_recorder():
    rospy.wait_for_service('gripper_data_request')	
    try:
        #create joint states file
        js_fp = open('joint_data.txt', 'w')
        #create tf data file
        tf_fp = open('tf_data.txt', 'w')
    	#create wrench data file
    	wr_fp = open('wrench_data.txt', 'w')
    	#create gripper data file
    	gr_fp = open('gripper_data.txt', 'w')
    	
    	rospy.init_node('demo_recorder', anonymous=True)
    	grp_req = rospy.ServiceProxy('gripper_data_request', gripper_data_request)
    	grp_req(0)
    	
    	print('Press [Enter] to start recording')
    	raw_input()
    	print(time.time())
    	
    	#create subscribers to topics
    	
    	sub_js = rospy.Subscriber("joint_states", JointState, js_callback, js_fp)
    		
    	sub_tf = rospy.Subscriber("tf", tfMessage, tf_callback, tf_fp)
			
    	sub_wr = rospy.Subscriber("wrench", WrenchStamped, wr_callback, wr_fp)
    	
    	sub_gr = rospy.Subscriber("/gripper_data/position", gripper_pos, gr_callback, gr_fp)
    		
    	rospy.spin()
       	end_record(js_fp, tf_fp, wr_fp, gr_fp)
        
    except rospy.ROSInterruptException:
        end_record(js_fp, tf_fp, wr_fp, gr_fp)
    except rospy.ServiceException, e: 
        end_record(js_fp, tf_fp, wr_fp, gr_fp)
        print "Service call failed: %s"%e
    except KeyboardInterrupt:
        end_record(js_fp, tf_fp, wr_fp, gr_fp)
        
if __name__ == '__main__':
	demo_recorder()

