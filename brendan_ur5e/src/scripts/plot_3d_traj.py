#!/usr/bin/env python

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import h5py		
import numpy as np
		
def plot_3d(x, y, z):
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	ax.plot(x, y, z, linewidth=2, markersize=12)
  #ax.title(filename)
  #ax.ylabel('pos_z')
  #ax.xlabel('pos_y')
  #ax.plot(-pos_rot_data[0],-pos_rot_data[1], pos_rot_data[2], 'r')
  #plt.title(filename + ' preprocessed + lte')
  #plt.ylabel('pos_z')
  #plt.xlabel('pos_y')
	plt.show()	

def main():
	filename = raw_input('Enter the filename of the .h5 demo: ')
  #open the file
	hf = h5py.File(filename, 'r')
  #navigate to necessary data and store in numpy arrays
	demo = hf.get('demo1')
	tf_info = demo.get('tf_info')
	js_info = demo.get('joint_state_info')
	pos_rot_data = tf_info.get('pos_rot_data')
	pos_rot_data = np.array(pos_rot_data)
	js_data = js_info.get('joint_positions')
	js_data = np.array(js_data)
	#close out file
	hf.close()
	plot_3d(-pos_rot_data[0],-pos_rot_data[1], pos_rot_data[2])
    
if __name__ == '__main__':
  main()
