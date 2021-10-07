import numpy as np
import matplotlib.pyplot as plt
import db_downsample
import douglas_peucker
import h5py
import os
import matplotlib as mpl
mpl.rc('font',family='Times New Roman')

# function to downsample a 1 dimensional trajectory to n points
# arguments
# traj: nxd vector, where n is number of points and d is number of dims
# n (optional): the number of points in the downsampled trajectory. Default is 100.
# returns the trajectory downsampled to n points
def downsample_traj(traj, n=100):
    n_pts, n_dims = np.shape(traj)
    npts = np.linspace(0, n_pts - 1, n)
    out = np.zeros((n, n_dims))
    for i in range(n):
        out[i][:] = traj[int(npts[i])][:]
    return out
    

def get_lasa_trajn(shape_name, n=1):
    # ask user for the file which the playback is for
    # filename = raw_input('Enter the filename of the .h5 demo: ')
    # open the file
    filename = '../h5 files/lasa_dataset.h5'
    hf = h5py.File(filename, 'r')
    # navigate to necessary data and store in numpy arrays
    shape = hf.get(shape_name)
    demo = shape.get('demo' + str(n))
    pos_info = demo.get('pos')
    pos_data = np.array(pos_info)
    y_data = np.delete(pos_data, 0, 1)
    x_data = np.delete(pos_data, 1, 1)
    # close out file
    hf.close()
    return [x_data, y_data]
    
[x, y] = get_lasa_trajn('Leaf_1')

traj = np.hstack((np.reshape(x, (len(x), 1)), np.reshape(y, (len(y), 1))))

n = 10

traj_naive = downsample_traj(traj, n)
traj_db = db_downsample.db_downsample(traj, n)
traj_dp = douglas_peucker.DouglasPeuckerPoints(traj, n)

fig = plt.figure()
demo, = plt.plot(traj[:, 0], traj[:, 1], 'k.', lw=3, ms=5, label='Demonstration')
naive, = plt.plot(traj_naive[:, 0], traj_naive[:, 1], 'm.-', lw=3, ms=9, label='Naive')
db, = plt.plot(traj_db[:, 0], traj_db[:, 1], 'b.-', lw=3, ms=9, label='Distance-based')
dp, = plt.plot(traj_dp[:, 0], traj_dp[:, 1], 'c.-', lw=3, ms=9, label='Douglas-Peucker')
plt.xticks([])
plt.yticks([])
plt.legend(fontsize='x-large', loc='best', bbox_to_anchor=(0.5, 0.5))
#plt.savefig('../pictures/method_testing/init_methods.png', dpi=300, bbox_inches='tight')
plt.show()