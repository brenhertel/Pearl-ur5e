#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import h5py
import preprocessing

def process_xyz(filename):
  #open the file
  hf = h5py.File(filename, 'r')
  #navigate to necessary data and store in numpy arrays
  #first layer
  demo = hf.get('demo1')
  #second layer
  time_info = demo.get('time_info')
  js_info = demo.get('joint_state_info')
  tf_info = demo.get('tf_info')
  gripper_info = demo.get('gripper_info')
  #third layer
  time_data = time_info.get('time_data')
  jp_data = js_info.get('joint_positions')
  je_data = js_info.get('joint_effort')
  pos_rot_data = tf_info.get('pos_rot_data')
  force_data = gripper_info.get('force_data')
  torq_data = gripper_info.get('torque_data')
  #convert to numpy arrays
  time_data = np.array(time_data)
  jp_data = np.array(jp_data)
  je_data = np.array(je_data)
  pos_rot_data = np.array(pos_rot_data)
  force_data = np.array(force_data)
  torq_data = np.array(torq_data)
  #close out file
  hf.close()
  #convert ns to secs and add
  time = time_data[0]
  for i in range (len(time)):
    time[i] = time[i] + (time_data[1, i] * 0.000000001)
  time = np.reshape(time, (1, len(time)))
  print(time)
  print(np.shape(time))
  print(np.shape(jp_data))
  print(np.shape(je_data))
  print(np.shape(pos_rot_data))
  print(np.shape(force_data))
  print(np.shape(torq_data))
  num_points = 50
  print('Splicing')
  pos_data = pos_rot_data[0:3][:]
  rot_data = pos_rot_data[3:7][:]
  print(np.shape(pos_data))
  print(np.shape(rot_data))
  [pos_data, actual_start, actual_end] = preprocessing.preprocess_nd(pos_data, num_points)#, start=-1, end=-1)
  rot_data = preprocessing.cut_nd(pos_data, actual_start, actual_end)[0]
  pos_rot_data = np.vstack((pos_data, rot_data))
  print(np.shape(pos_rot_data))
  print(actual_start)
  print(actual_end)
  time = preprocessing.preprocess_nd(time, num_points, actual_start, actual_end)[0]
  jp_data = preprocessing.preprocess_nd(jp_data, num_points, actual_start, actual_end)[0]
  je_data = preprocessing.preprocess_nd(je_data, num_points, actual_start, actual_end)[0]
  force_data = preprocessing.preprocess_nd(force_data, num_points, actual_start, actual_end)[0]
  torq_data = preprocessing.preprocess_nd(torq_data, num_points, actual_start, actual_end)[0]
  print(np.shape(time))
  print(np.shape(jp_data))
  print(np.shape(je_data))
  print(np.shape(pos_rot_data))
  print(np.shape(force_data))
  print(np.shape(torq_data))

  fp = h5py.File ('preprocessed' + filename, 'w')
  demo_name = 'demo1'
  dset_time = fp.create_dataset(demo_name + '/time_info/time_data', data=time)
  dset_joint = fp.create_dataset(demo_name + '/joint_state_info/joint_positions', data=jp_data)
  dset_eff = fp.create_dataset(demo_name + '/joint_state_info/joint_effort', data=je_data)
  dset_pos = fp.create_dataset(demo_name + '/tf_info/pos_rot_data', data=pos_rot_data)
  dset_force = fp.create_dataset(demo_name + '/gripper_info/force_data', data=force_data)
  dset_torq = fp.create_dataset(demo_name + '/gripper_info/torque_data', data=torq_data)
  fp.close()

def process_entire_demo(filename):
  #open the file
  hf = h5py.File(filename, 'r')
  #navigate to necessary data and store in numpy arrays
  #first layer
  demo = hf.get('demo1')
  #second layer
  time_info = demo.get('time_info')
  js_info = demo.get('joint_state_info')
  tf_info = demo.get('tf_info')
  gripper_info = demo.get('gripper_info')
  #third layer
  time_data = time_info.get('time_data')
  jp_data = js_info.get('joint_positions')
  je_data = js_info.get('joint_effort')
  pos_rot_data = tf_info.get('pos_rot_data')
  force_data = gripper_info.get('force_data')
  torq_data = gripper_info.get('torque_data')
  #convert to numpy arrays
  time_data = np.array(time_data)
  jp_data = np.array(jp_data)
  je_data = np.array(je_data)
  pos_rot_data = np.array(pos_rot_data)
  force_data = np.array(force_data)
  torq_data = np.array(torq_data)
  #close out file
  hf.close()
  #convert ns to secs and add
  time = time_data[0]
  for i in range (len(time)):
    time[i] = time[i] + (time_data[1, i] * 0.000000001)
  time = np.reshape(time, (1, len(time)))
  print(time)
  print(np.shape(time))
  print(np.shape(jp_data))
  print(np.shape(je_data))
  print(np.shape(pos_rot_data))
  print(np.shape(force_data))
  print(np.shape(torq_data))
  num_points = 50
  [pos_rot_data, actual_start, actual_end] = preprocessing.preprocess_nd(pos_rot_data, num_points)#, start=-1, end=-1)
  print(np.shape(pos_rot_data))
  print(actual_start)
  print(actual_end)
  time = preprocessing.preprocess_nd(time, num_points, actual_start, actual_end)[0]
  jp_data = preprocessing.preprocess_nd(jp_data, num_points, actual_start, actual_end)[0]
  je_data = preprocessing.preprocess_nd(je_data, num_points, actual_start, actual_end)[0]
  force_data = preprocessing.preprocess_nd(force_data, num_points, actual_start, actual_end)[0]
  torq_data = preprocessing.preprocess_nd(torq_data, num_points, actual_start, actual_end)[0]
  print(np.shape(time))
  print(np.shape(jp_data))
  print(np.shape(je_data))
  print(np.shape(pos_rot_data))
  print(np.shape(force_data))
  print(np.shape(torq_data))

  fp = h5py.File ('preprocessed' + filename, 'w')
  demo_name = 'demo1'
  dset_time = fp.create_dataset(demo_name + '/time_info/time_data', data=time)
  dset_joint = fp.create_dataset(demo_name + '/joint_state_info/joint_positions', data=jp_data)
  dset_eff = fp.create_dataset(demo_name + '/joint_state_info/joint_effort', data=je_data)
  dset_pos = fp.create_dataset(demo_name + '/tf_info/pos_rot_data', data=pos_rot_data)
  dset_force = fp.create_dataset(demo_name + '/gripper_info/force_data', data=force_data)
  dset_torq = fp.create_dataset(demo_name + '/gripper_info/torque_data', data=torq_data)
  fp.close()


if __name__ == '__main__':
  #get filename from user
  filename = raw_input('Enter the filename of the .h5 demo: ')
  process_xyz(filename)
