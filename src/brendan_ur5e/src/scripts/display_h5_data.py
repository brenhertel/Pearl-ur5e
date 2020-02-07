#!/usr/bin/env python

import numpy as np
import h5py
import matplotlib.pyplot as plt

def main():
  filename = raw_input('Enter the filename of the .h5 demo: ')
  #open the file
  hf = h5py.File(filename, 'r')
  #navigate to necessary data and store in numpy arrays
  demo = hf.get('demo1')
  tf_info = demo.get('tf_info')
  #js_info = demo.get('joint_state_info')
  #time_info = demo.get('time_info')
  pos_rot_data = tf_info.get('pos_rot_data')
  pos_rot_data = np.array(pos_rot_data)
  #js_data = js_info.get('joint_positions')
  #js_data = np.array(js_data)
  #time_data = time_info.get('time_data')
  #time_data = np.array(time_data)
  #close out file
  hf.close()
  pos_x_data = np.array(pos_rot_data[0]).reshape(max(np.shape(pos_rot_data)))
  pos_y_data = np.array(pos_rot_data[1]).reshape(max(np.shape(pos_rot_data)))
  pos_z_data = np.array(pos_rot_data[2]).reshape(max(np.shape(pos_rot_data)))
  rot_x_data = np.array(pos_rot_data[3]).reshape(max(np.shape(pos_rot_data)))
  rot_y_data = np.array(pos_rot_data[4]).reshape(max(np.shape(pos_rot_data)))
  rot_z_data = np.array(pos_rot_data[5]).reshape(max(np.shape(pos_rot_data)))
  rot_w_data = np.array(pos_rot_data[6]).reshape(max(np.shape(pos_rot_data)))
  #time = time_data[0]
  #convert ns to secs and add
  #for i in range (len(time)):
  #  time[i] = time[i] + (time_data[1, i] * 0.000000001)
  
  print(np.shape(pos_x_data))
  print(pos_x_data)

  n = np.linspace(1, max(np.shape(pos_x_data)), max(np.shape(pos_x_data))).reshape(np.shape(pos_x_data))
 
  print(np.shape(n))
  print(n)

  #plot data
  plt.subplot(7, 1, 1)
  #plt.plot(time, pos_x_data)
  plt.plot(n, pos_x_data)
  plt.title(filename)
  plt.ylabel('pos_x')

  plt.subplot(7, 1, 2)
  #plt.plot(time, pos_y_data)
  plt.plot(n, pos_y_data)
  plt.ylabel('pos_y')

  plt.subplot(7, 1, 3)
  #plt.plot(time, pos_z_data)
  plt.plot(n, pos_z_data)
  plt.ylabel('pos_z')

  plt.subplot(7, 1, 4)
  #plt.plot(time, rot_x_data)
  plt.plot(n, rot_x_data)
  plt.ylabel('rot_x')

  plt.subplot(7, 1, 5)
  #plt.plot(time, rot_y_data)
  plt.plot(n, rot_y_data)
  plt.ylabel('rot_y')

  plt.subplot(7, 1, 6)
  #plt.plot(time, rot_z_data)
  plt.plot(n, rot_z_data)
  plt.ylabel('rot_z')

  plt.subplot(7, 1, 7)
  #plt.plot(time, rot_w_data)
  plt.plot(n, rot_w_data)
  plt.ylabel('rot_w')
  #plt.xlabel('time')
  plt.xlabel('n')

  plt.show() 




if __name__ == '__main__':
  main()
