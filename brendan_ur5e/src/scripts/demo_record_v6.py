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
	

def tf0_callback(tfmsg, tf0_file):	
	tf0_file.write(str(tfmsg.transforms[0].header.stamp.secs) + ', ' + str(tfmsg.transforms[0].header.stamp.nsecs) + ', ' + str(tfmsg.transforms[0].transform.translation.x) + ', ' + str(tfmsg.transforms[0].transform.translation.y) + ', ' + str(tfmsg.transforms[0].transform.translation.z) + ', ' + str(tfmsg.transforms[0].transform.rotation.x) + ', ' + str(tfmsg.transforms[0].transform.rotation.y) + ', ' + str(tfmsg.transforms[0].transform.rotation.z) + ', ' + str(tfmsg.transforms[0].transform.rotation.w) + '\n')
        
def tf1_callback(tfmsg, tf1_file):	
	tf1_file.write(str(tfmsg.transforms[0].header.stamp.secs) + ', ' + str(tfmsg.transforms[0].header.stamp.nsecs) + ', ' + str(tfmsg.transforms[0].transform.translation.x) + ', ' + str(tfmsg.transforms[0].transform.translation.y) + ', ' + str(tfmsg.transforms[0].transform.translation.z) + ', ' + str(tfmsg.transforms[0].transform.rotation.x) + ', ' + str(tfmsg.transforms[0].transform.rotation.y) + ', ' + str(tfmsg.transforms[0].transform.rotation.z) + ', ' + str(tfmsg.transforms[0].transform.rotation.w) + '\n')
        
def js_callback(jsmsg, js_file):
	js_file.write(str(jsmsg.header.stamp.secs) + ', ' + str(jsmsg.header.stamp.nsecs).replace(')', '').replace('(', '') + ', ' + str(jsmsg.position).replace(')', '').replace('(', '') + ', ' + str(jsmsg.velocity).replace(')', '').replace('(', '') + ', ' + str(jsmsg.effort).replace(')', '').replace('(', '') + '\n')
 
def getline_data(fp):
	return np.array([float(i) for i in fp.readline().split(', ')])
   
def save_demo():
	js_fp = open('joint_data.txt', 'r')
	tf0_fp = open('tf0_data.txt', 'r')
	tf1_fp = open('tf1_data.txt', 'r')
	wr_fp = open('wrench_data.txt', 'r')
	gr_fp = open('gripper_data.txt', 'r')
	js_time_arr = np.zeros((1, 2))
	js_pos_arr = np.zeros((1, 6))
	js_vel_arr = np.zeros((1, 6))
	js_eff_arr = np.zeros((1, 6))
	tf0_time_arr = np.zeros((1, 2))
	tf0_pos_arr = np.zeros((1, 3))
	tf0_rot_arr = np.zeros((1, 4))
	tf1_time_arr = np.zeros((1, 2))
	tf1_pos_arr = np.zeros((1, 3))
	tf1_rot_arr = np.zeros((1, 4))
	wr_time_arr = np.zeros((1, 2))
	wr_force_arr = np.zeros((1, 3))
	wr_torq_arr = np.zeros((1, 3))
	gr_time_arr = np.zeros((1, 2))
	gr_pos_arr = np.zeros((1, 1))
	
	name = 'recorded_demo ' + time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()) + '.h5'
	fp = h5py.File(name, 'w')
	
	try:
	    js_data = getline_data(js_fp)
	    tf0_data = getline_data(tf0_fp)
	    tf1_data = getline_data(tf1_fp)
	    wr_data = getline_data(wr_fp)
	    gr_data = getline_data(gr_fp)
	    while True:
	        #print(js_data)
	        #print(tf_data)
	        #print(wr_data)
	        #print(gr_data)
	        js_time = js_data[0] + (js_data[1] * 10.0**-9)
	        tf0_time = tf0_data[0] + (tf0_data[1] * 10.0**-9)
	        tf1_time = tf1_data[0] + (tf1_data[1] * 10.0**-9)
	        wr_time = wr_data[0] + (wr_data[1] * 10.0**-9)
	        gr_time = gr_data[0] + (gr_data[1] * 10.0**-9)
	        ctime_arr = [js_time, tf0_time, tf1_time, wr_time, gr_time]
	        #print(ctime_arr)
	        if max(ctime_arr) - min(ctime_arr) < 0.001:
	            print('found collective record at t=' + str(js_time))
	            #record
	            #print(len(js_data[0:2]))
	            js_time_arr = np.vstack((js_time_arr, js_data[0:2]))
	            #print(len(js_data[2:8]))
	            js_pos_arr = np.vstack((js_pos_arr, js_data[2:8]))
	            #print(len(js_data[8:14]))
	            js_vel_arr = np.vstack((js_vel_arr, js_data[8:14]))
	            #print(len(js_data[14:20]))
	            js_eff_arr = np.vstack((js_eff_arr, js_data[14:20]))
	            
	            tf0_time_arr = np.vstack((tf0_time_arr, tf0_data[0:2]))
	            tf0_pos_arr = np.vstack((tf0_pos_arr, tf0_data[2:5]))
	            tf0_rot_arr = np.vstack((tf0_rot_arr, tf0_data[5:9]))
                
	            tf1_time_arr = np.vstack((tf1_time_arr, tf1_data[0:2]))
	            tf1_pos_arr = np.vstack((tf1_pos_arr, tf1_data[2:5]))
	            tf1_rot_arr = np.vstack((tf1_rot_arr, tf1_data[5:9]))
	            
	            wr_time_arr = np.vstack((wr_time_arr, wr_data[0:2]))
	            wr_force_arr = np.vstack((wr_force_arr, wr_data[2:5]))
	            wr_torq_arr = np.vstack((wr_torq_arr, wr_data[5:8]))
	            
	            gr_time_arr = np.vstack((gr_time_arr, gr_data[0:2]))
	            gr_pos_arr = np.vstack((gr_pos_arr, gr_data[2]))
	            
	            #print('reading js')
	            js_data = getline_data(js_fp)
	            #print('reading tf')
	            tf0_data = getline_data(tf0_fp)
	            tf1_data = getline_data(tf1_fp)
	            #print('reading wr')
	            wr_data = getline_data(wr_fp)
	            #print('reading gr')
	            gr_data = getline_data(gr_fp)
	        else:
	            if min(ctime_arr) == js_time:
	                #print('reading js')
	                js_data = getline_data(js_fp)
	            elif min(ctime_arr) == tf0_time:
	                #print('reading tf')
	                tf0_data = getline_data(tf0_fp)
	            elif min(ctime_arr) == tf1_time:
	                #print('reading tf')
	                tf1_data = getline_data(tf1_fp)
	            elif min(ctime_arr) == wr_time:
	                #print('reading wr')
	                wr_data = getline_data(wr_fp)
	            elif min(ctime_arr) == gr_time:
	                #print('reading gr')
	                gr_data = getline_data(gr_fp)
	            else:
	                rospy.loginfo('Should never get here')
	except ValueError:
	    rospy.loginfo('Finished demo recording')
	    
	js_fp.close()
	tf0_fp.close()
	tf1_fp.close()
	wr_fp.close()
	gr_fp.close()
	
	#delete first row of 0's
	js_time_arr = np.delete(js_time_arr, 0, 0)
	js_pos_arr = np.delete(js_pos_arr, 0, 0)
	js_vel_arr = np.delete(js_vel_arr, 0, 0)
	js_eff_arr = np.delete(js_eff_arr, 0, 0)
	
	tf0_time_arr = np.delete(tf0_time_arr, 0, 0)
	tf0_pos_arr = np.delete(tf0_pos_arr, 0, 0)
	tf0_rot_arr = np.delete(tf0_rot_arr, 0, 0)
    
	tf1_time_arr = np.delete(tf1_time_arr, 0, 0)
	tf1_pos_arr = np.delete(tf1_pos_arr, 0, 0)
	tf1_rot_arr = np.delete(tf1_rot_arr, 0, 0)
	
	wr_time_arr = np.delete(wr_time_arr, 0, 0)
	wr_force_arr = np.delete(wr_force_arr, 0, 0)
	wr_torq_arr = np.delete(wr_torq_arr, 0, 0)
	
	gr_time_arr = np.delete(gr_time_arr, 0, 0)
	gr_pos_arr = np.delete(gr_pos_arr, 0, 0)
	
	dset_jt = fp.create_dataset('/joint_state_info/joint_time', data=js_time_arr)
	dset_jp = fp.create_dataset('/joint_state_info/joint_positions', data=js_pos_arr)
	dset_jv = fp.create_dataset('/joint_state_info/joint_velocities', data=js_vel_arr)
	dset_je = fp.create_dataset('/joint_state_info/joint_effort', data=js_eff_arr)
	
	dset_tt0 = fp.create_dataset('/transform_info/tool0/transform_time', data=tf0_time_arr)
	dset_tp0 = fp.create_dataset('/transform_info/tool0/transform_positions', data=tf0_pos_arr)
	dset_tr0 = fp.create_dataset('/transform_info/tool0/transform_orientations', data=tf0_rot_arr)
	dset_tt1 = fp.create_dataset('/transform_info/tool1/transform_time', data=tf1_time_arr)
	dset_tp1 = fp.create_dataset('/transform_info/tool1/transform_positions', data=tf1_pos_arr)
	dset_tr1 = fp.create_dataset('/transform_info/tool1/transform_orientations', data=tf1_rot_arr)
	
	dset_wt = fp.create_dataset('/wrench_info/wrench_time', data=wr_time_arr)
	dset_wf = fp.create_dataset('/wrench_info/wrench_force', data=wr_force_arr)
	dset_wm = fp.create_dataset('/wrench_info/wrench_torque', data=wr_torq_arr)
	
	dset_gt = fp.create_dataset('/gripper_info/gripper_time', data=gr_time_arr)
	dset_gp = fp.create_dataset('/gripper_info/gripper_position', data=gr_pos_arr)
	
	fp.close()
	
def end_record(js_file, tf0_file, tf1_file, wr_file, gr_file):
    print(time.time())
    
    js_file.close()
    tf0_file.close()
    tf1_file.close()
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
        tf0_fp = open('tf0_data.txt', 'w')
        tf1_fp = open('tf1_data.txt', 'w')
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
    		
    	sub_tf0 = rospy.Subscriber("/tool0", tfMessage, tf0_callback, tf0_fp)
    	sub_tf1 = rospy.Subscriber("/tool1", tfMessage, tf1_callback, tf1_fp)
			
    	sub_wr = rospy.Subscriber("wrench", WrenchStamped, wr_callback, wr_fp)
    	
    	sub_gr = rospy.Subscriber("/gripper_data/position", gripper_pos, gr_callback, gr_fp)
    		
    	rospy.loginfo('Recording has started')
    		
    	rospy.spin()
       	end_record(js_fp, tf0_fp, tf1_fp, wr_fp, gr_fp)
        
    except rospy.ROSInterruptException:
        end_record(js_fp, tf0_fp, tf1_fp, wr_fp, gr_fp)
    except rospy.ServiceException, e: 
        end_record(js_fp, tf0_fp, tf1_fp, wr_fp, gr_fp)
        print "Service call failed: %s"%e
    except KeyboardInterrupt:
        end_record(js_fp, tf0_fp, tf1_fp, wr_fp, gr_fp)
        
if __name__ == '__main__':
	demo_recorder()

