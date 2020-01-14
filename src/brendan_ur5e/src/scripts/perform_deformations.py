#!/usr/bin/env python

import rospy
import roslib
import numpy as np
import h5py
import matplotlib.pyplot as plt
import lte

def get_lasa_traj():
    #ask user for the file which the playback is for
    #filename = raw_input('Enter the filename of the .h5 demo: ')
    #open the file
    filename = '/home/bhertel/catkin_ws//lasa_dataset.h5'
    hf = h5py.File(filename, 'r')
    #navigate to necessary data and store in numpy arrays
    shape = hf.get('Angle')
    demo = shape.get('demo1')
    pos_info = demo.get('pos')
    pos_data = np.array(pos_info)
    y_data = np.delete(pos_data, 0, 1)
    x_data = np.delete(pos_data, 1, 1)
    #close out file
    hf.close()
    return [x_data, y_data]


def main():
  ## Undeformed ##
  #retrive data from file
  [x_data, y_data] = get_lasa_traj()
  
  ## LTE ##
  #set up fixed points for lte deformations
  indeces = [1, len(x_data) - 1]
  x_positions = [-50, -5]
  y_positions = [5, 5]
  x_fixed_points = lte.generate_fixed_points(indeces, x_positions)
  y_fixed_points = lte.generate_fixed_points(indeces, y_positions)
  #perform lte deformatiosn
  x_lte_traj = lte.perform_lte(np.transpose(x_data), x_fixed_points)
  y_lte_traj = lte.perform_lte(np.transpose(y_data), y_fixed_points)

  #plot data
  plt.plot(x_data, y_data)
  plt.plot(x_lte_traj, y_lte_traj)
  plt.show()
  return

if __name__ == '__main__':
  main()
