#!/usr/bin/env python

import numpy as np
import h5py
import perform_deformations as pd
import preprocessing
import matplotlib.pyplot as plt

filename = None

def get_file_data():
  #ask user for the file which the deformation is for
  global filename
  filename = raw_input('Enter the filename of the .h5 demo: ')
  #open the file
  hf = h5py.File(filename, 'r')
  #navigate to necessary data and store in numpy arrays
  demo = hf.get('demo1')
  tf_info = demo.get('tf_info')
  pos_rot_data = tf_info.get('pos_rot_data')
  pos_rot_data = np.array(pos_rot_data)
  #close out file
  hf.close()
  return pos_rot_data

def main():
  #get file data
  pos_rot_data = get_file_data()
  print(np.shape(pos_rot_data))
  print(pos_rot_data)
  #preprocess data
  [preprocessed_pos_rot_data, actual_start, actual_end] = preprocessing.preprocess_nd(pos_rot_data, 1000, start=-1, end=-1)
  #deform trajectories
  print(np.shape(preprocessed_pos_rot_data))
  print(preprocessed_pos_rot_data)
  [pos_x_lte_traj, pos_x_ja_traj, pos_x_dmp_traj] = pd.perform_all_deformations(preprocessed_pos_rot_data[0], None, None, 10.0)
  [pos_y_lte_traj, pos_y_ja_traj, pos_y_dmp_traj] = pd.perform_all_deformations(preprocessed_pos_rot_data[1], None, None, 10.0)
  [pos_z_lte_traj, pos_z_ja_traj, pos_z_dmp_traj] = pd.perform_all_deformations(preprocessed_pos_rot_data[2], None, None, 10.0)
  [rot_x_lte_traj, rot_x_ja_traj, rot_x_dmp_traj] = pd.perform_all_deformations(preprocessed_pos_rot_data[3], None, None, 10.0)
  [rot_y_lte_traj, rot_y_ja_traj, rot_y_dmp_traj] = pd.perform_all_deformations(preprocessed_pos_rot_data[4], None, None, 10.0)
  [rot_z_lte_traj, rot_z_ja_traj, rot_z_dmp_traj] = pd.perform_all_deformations(preprocessed_pos_rot_data[5], None, None, 10.0)
  [rot_w_lte_traj, rot_w_ja_traj, rot_w_dmp_traj] = pd.perform_all_deformations(preprocessed_pos_rot_data[6], None, None, 10.0)
  #turn deformed trajectories into numpy arrays of each type
  lte_trajs = np.array([[pos_x_lte_traj], [pos_y_lte_traj], [pos_z_lte_traj], [rot_x_lte_traj], [rot_y_lte_traj], [rot_z_lte_traj], [rot_w_lte_traj]])
  ja_trajs = np.array([[pos_x_ja_traj], [pos_y_ja_traj], [pos_z_ja_traj], [rot_x_ja_traj], [rot_y_ja_traj], [rot_z_ja_traj], [rot_w_ja_traj]])
  dmp_trajs = np.array([[pos_x_dmp_traj], [pos_y_dmp_traj], [pos_z_dmp_traj], [rot_x_dmp_traj], [rot_y_dmp_traj], [rot_z_dmp_traj], [rot_w_dmp_traj]])
  #store data
  global filename
  fp = h5py.File('deformed ' + filename, 'w')
  dset_lte = fp.create_dataset('/LTE', data=lte_trajs)
  dset_joint = fp.create_dataset('/JA', data=ja_trajs)
  dset_eff = fp.create_dataset('/DMP', data=dmp_trajs)
  fp.close()


if __name__ == '__main__':
  main()
