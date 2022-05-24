#!/usr/bin/env python

import numpy as np
import h5py
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

NUM_JOINTS = 6

def read_data(fname):
    hf = h5py.File(fname, 'r')
    print(list(hf.keys()))
    js = hf.get('joint_state_info')
    joint_time = np.array(js.get('joint_time'))
    joint_pos = np.array(js.get('joint_positions'))
    joint_vel = np.array(js.get('joint_velocities'))
    joint_eff = np.array(js.get('joint_effort'))
    joint_data = [joint_time, joint_pos, joint_vel, joint_eff]
    
    tf = hf.get('transform_info')
    tf_time = np.array(tf.get('transform_time'))
    tf_pos = np.array(tf.get('transform_positions'))
    tf_rot = np.array(tf.get('transform_orientations'))
    tf_data = [tf_time, tf_pos, tf_rot]
    print(tf_pos)
    
    wr = hf.get('wrench_info')
    wrench_time = np.array(wr.get('wrench_time'))
    wrench_frc = np.array(wr.get('wrench_force'))
    wrench_trq = np.array(wr.get('wrench_torque'))
    wrench_data = [wrench_time, wrench_frc, wrench_trq]
    
    gp = hf.get('gripper_info')
    gripper_time = np.array(gp.get('gripper_time'))
    gripper_pos = np.array(gp.get('gripper_position'))
    gripper_data = [gripper_time, gripper_pos]
    
    hf.close()
    
    return joint_data, tf_data, wrench_data, gripper_data

def display_data(joint_data, tf_data, wrench_data, gripper_data):
    print('joint_time: ' + str(np.shape(joint_data[0])))
    print('joint_positions: ' + str(np.shape(joint_data[1])))
    print('joint_velocities: ' + str(np.shape(joint_data[2])))
    print('joint_effort: ' + str(np.shape(joint_data[3])))
    
    print('transform_time: ' + str(np.shape(tf_data[0])))
    print('transform_positions: ' + str(np.shape(tf_data[1])))
    print('transform_orientations: ' + str(np.shape(tf_data[2])))
    
    print('wrench_time: ' + str(np.shape(wrench_data[0])))
    print('wrench_force: ' + str(np.shape(wrench_data[1])))
    print('wrench_torque: ' + str(np.shape(wrench_data[2])))
    
    print('gripper_time: ' + str(np.shape(gripper_data[0])))
    print('gripper_position: ' + str(np.shape(gripper_data[1])))
    return

def plot_joint_data(joint_data):
	js_fig, (ax1, ax2, ax3) = plt.subplots(3, 1)
	js_fig.suptitle('Joints')
	time = joint_data[0][:, 0] + joint_data[0][:, 1] * (10.0**-9)
	for ax, data in [(ax1, joint_data[1]), (ax2, joint_data[2]), (ax3, joint_data[3])]:
		for i in range(NUM_JOINTS):
			ax.plot(time, data[:, i], label= 'joint' + str(i))
		ax.legend()
	ax3.set_xlabel('time')
	ax1.set_ylabel('positions (rad)')
	ax2.set_ylabel('velocities (rad/s)')
	ax3.set_ylabel('effort')
	
def plot_tf_data(tf_data):
	tf_fig, (ax1, ax2) = plt.subplots(2, 1)
	tf_fig.suptitle('tf')
	time = tf_data[0][:, 0] + tf_data[0][:, 1] * (10.0**-9)
	ax1.plot(time, tf_data[1][:, 0], label='x')
	ax1.plot(time, tf_data[1][:, 1], label='y')
	ax1.plot(time, tf_data[1][:, 2], label='z')
	ax2.plot(time, tf_data[2][:, 0], label='x')
	ax2.plot(time, tf_data[2][:, 1], label='y')
	ax2.plot(time, tf_data[2][:, 2], label='z')
	ax2.plot(time, tf_data[2][:, 3], label='w')
	
	ax1.legend()
	ax2.legend()
	
	ax2.set_xlabel('time')
	ax1.set_ylabel('position')
	ax2.set_ylabel('orientation')
	
	fig = plt.figure()
	fig.suptitle('Trajectory')
	ax = plt.axes(projection='3d')
	ax.plot3D(tf_data[1][:, 0], tf_data[1][:, 1], tf_data[1][:, 2], 'k')
	ax.set_xlabel('x')
	ax.set_ylabel('y')
	ax.set_zlabel('z')
	
def plot_wrench_data(wrench_data):
	wr_fig, (ax1, ax2) = plt.subplots(2, 1)
	wr_fig.suptitle('Wrench')
	time = wrench_data[0][:, 0] + wrench_data[0][:, 1] * (10.0**-9)
	
	ax1.plot(time, wrench_data[1][:, 0], label='x')
	ax1.plot(time, wrench_data[1][:, 1], label='y')
	ax1.plot(time, wrench_data[1][:, 2], label='z')
	ax2.plot(time, wrench_data[2][:, 0], label='x')
	ax2.plot(time, wrench_data[2][:, 1], label='y')
	ax2.plot(time, wrench_data[2][:, 2], label='z')
	
	ax1.legend()
	ax2.legend()
	
	ax2.set_xlabel('time')
	ax1.set_ylabel('force')
	ax2.set_ylabel('torque')
	
def plot_gripper_data(gripper_data):
	gp_fig, ax = plt.subplots(1, 1)
	gp_fig.suptitle('Gripper')
	time = gripper_data[0][:, 0] + gripper_data[0][:, 1] * (10.0**-9)
	ax.plot(time, gripper_data[1])
	ax.set_xlabel('time')
	ax.set_ylabel('position')
	
def plot_data(fname):
    joint_data, tf_data, wrench_data, gripper_data = read_data(fname)
    display_data(joint_data, tf_data, wrench_data, gripper_data)
    plot_joint_data(joint_data)
    plot_tf_data(tf_data)
    plot_wrench_data(wrench_data)
    plot_gripper_data(gripper_data)
    plt.show()
    return

def main():
    filename = '/home/bhertel/catkin_ws/src/brendan_ur5e/src/scripts/recorded_demo 2022-04-05 11:45:11(success,ARbox).h5'#raw_input('Enter the filename of the .h5 demo: ')
    plot_data(filename)



if __name__ == '__main__':
  main()
