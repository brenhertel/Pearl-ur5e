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
	js_file.write(str(jsmsg.header.stamp.secs) + ', ' + str(jsmsg.header.stamp.nsecs).replace(')', '').replace('(', '') + ', ' + str(jsmsg.position).replace(')', '').replace('(', '') + ', ' + str(jsmsg.velocity).replace(')', '').replace('(', '') + ', ' + str(jsmsg.effort).replace(')', '').replace('(', '') + '\n')
 
def getline_data(fp):
	return np.array(fp.readline().split(',')).astype(int)
   
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
	
	name = 'recorded_demo ' + time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()) + '.h5'
	fp = h5py.File(name, 'w')
	
	try:
	    js_data = getline_data(js_fp)
	    tf_data = getline_data(tf_fp)
	    wr_data = getline_data(wr_fp)
	    gr_data = getline_data(gr_fp)
	    while True:
	        js_time = js_data[0] + (js_data[1] * 10.0**-9)
	        tf_time = tf_data[0] + (tf_data[1] * 10.0**-9)
	        wr_time = wr_data[0] + (wr_data[1] * 10.0**-9)
	        gr_time = gr_data[0] + (gr_data[1] * 10.0**-9)
	        if js_time == tf_time and js_time == wr_time and js_time == gr_time:
	            #record
	            js_time_arr = np.vstack((js_time_arr, js_data[0:2]))
	            js_pos_arr = np.vstack((js_pos_arr, js_data[2:8]))
	            js_vel_arr = np.vstack((js_vel_arr, js_data[8:14]))
	            js_eff_arr = np.vstack((js_eff_arr, js_data[14:end]))
	            
	            tf_time_arr = np.vstack((tf_time_arr, tf_data[0:2]))
	            tf_pos_arr = np.vstack((tf_pos_arr, tf_data[2:5]))
	            tf_rot_arr = np.vstack((tf_rot_arr, tf_data[5:end]))
	            
	            wr_time_arr = np.vstack((wr_time_arr, wr_data[0:2]))
	            wr_force_arr = np.vstack((wr_force_arr, wr_data[2:5]))
	            wr_torq_arr = np.vstack((wr_torq_arr, wr_data[5:end]))
	            
	            gr_time_arr = np.vstack((gr_time_arr, gr_data[0:2]))
	            gr_pos_arr = np.vstack((gr_pos_arr, gr_data[2:end]))
	            
	            js_data = getline_data(js_fp)
	            tf_data = getline_data(tf_fp)
	            wr_data = getline_data(wr_fp)
	            gr_data = getline_data(gr_fp)
	        else:
	            if min([js_time, tf_time, wr_time, gr_time]) == js_time:
	                js_data = getline_data(js_fp)
	            elif min([js_time, tf_time, wr_time, gr_time]) == tf_time:
	                tf_data = getline_data(tf_fp)
	            elif min([js_time, tf_time, wr_time, gr_time]) == wr_time:
	                wr_data = getline_data(wr_fp)
	            elif min([js_time, tf_time, wr_time, gr_time]) == gr_time:
	                gr_data = getline_data(gr_fp)
	            else:
	                rospy.loginfo('Should never get here')
	except EOFError:
	    rospy.loginfo('Finished demo recording')
	    
	    js_fp.close()
	    tf_fp.close()
	    wr_fp.close()
	    gr_fp.close()
	    
	    dset_jt = fp.create_dataset('/joint_state_info/joint_time', data=js_time_arr)
	    dset_jp = fp.create_dataset('/joint_state_info/joint_positions', data=js_pos_arr)
	    dset_jv = fp.create_dataset('/joint_state_info/joint_velocities', data=js_vel_arr)
	    dset_je = fp.create_dataset('/joint_state_info/joint_effort', data=js_eff_arr)
	    
	    dset_tt = fp.create_dataset('/transform_info/transform_time', data=tf_time_arr)
	    dset_tp = fp.create_dataset('/transform_info/transform_positions', data=tf_pos_arr)
	    dset_tr = fp.create_dataset('/transform_info/transform_orientations', data=tf_rot_arr)
	    
	    dset_wt = fp.create_dataset('/wrench_info/wrench_time', data=wr_time_arr)
	    dset_wf = fp.create_dataset('/wrench_info/wrench_force', data=wr_force_arr)
	    dset_wm = fp.create_dataset('/wrench_info/wrench_torque', data=wr_torq_arr)
	    
	    dset_gt = fp.create_dataset('/gripper_info/gripper_time', data=gr_time_arr)
	    dset_gp = fp.create_dataset('/gripper_info/gripper_position', data=gr_pos_arr)
	    
	fp.close()
	    
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
    print('Starting recorder, waiting for gripper')
    #rospy.wait_for_service('gripper_data_request')	
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
    	#grp_req = rospy.ServiceProxy('gripper_data_request', gripper_data_request)
    	#grp_req(0)
    	
    	print('Press [Enter] to start recording')
    	raw_input()
    	print(time.time())
    	
    	#create subscribers to topics
    	
    	sub_js = rospy.Subscriber("joint_states", JointState, js_callback, js_fp)
    		
    	sub_tf = rospy.Subscriber("tf", tfMessage, tf_callback, tf_fp)
			
    	sub_wr = rospy.Subscriber("wrench", WrenchStamped, wr_callback, wr_fp)
    	
    	sub_gr = rospy.Subscriber("/gripper_data/position", gripper_pos, gr_callback, gr_fp)
    		
    	rospy.loginfo('Recording has started')
    		
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

