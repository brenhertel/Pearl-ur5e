import cv2
import numpy as np
import std_msgs
import h5py
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.patches import Circle
import mpl_toolkits.mplot3d.art3d as art3d
import copy

NUM_JOINTS = 6

import sys
# insert at 1, 0 is the script path (or '' in REPL)
sys.path.insert(1, '/home/bhertel/Downloads/TLFSD-main/scripts')

from TLFSD import *
import douglas_peucker as dp

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
    
    """
    #unsafe test
    cX = -0.4
    cY = -0.35
    
    #safe test
    cX = 0.12
    cY = -0.39
    
    min_x = cX - 0.13
    max_x = cX + 0.13
    min_y = cY - 0.10
    max_y = cY + 0.10
    min_z = 0.0
    max_z = 0.12
    """
    
    hf.close()
    
    
    return joint_data, tf_data, wrench_data, gripper_data#, min_x, max_x, min_y, max_y, min_z, max_z



#THIS DOES NOT WORK, out_pt variable broken  
def find_ar():
    
    vid = cv2.VideoCapture(2)
    
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
    parameters =  cv2.aruco.DetectorParameters_create()
    
    i = 0
    while vid.isOpened() and i < 100:
        _, frame = vid.read()
        i = i + 1
    #rospy.loginfo('Frame read, publishing...')
    cv2.imshow("image", frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    corners, ids, rejected = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
    print('A')
    print(corners)
    print('B')
    if len(corners) > 0:
        ids = ids.flatten()
        for (markerCorner, markerID) in zip(corners, ids):
			
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners

            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
			
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cZ = 1.0
            
            h = np.loadtxt('homography.txt')
            
            in_pt = np.array([cX, cY, cZ]).reshape((3, 1))
            out_pt = np.matmul(h, in_pt)
    
    min_x = -out_pt[0] - 0.13
    max_x = -out_pt[0] + 0.13
    min_y = -out_pt[1] - 0.10
    max_y = -out_pt[1] + 0.10
    min_z = 0.0
    max_z = 0.12
    
    return min_x, max_x, min_y, max_y, min_z, max_z

def plot_xyz():
    fig = plt.figure()
    plt.title('Trajectory')
    ax = plt.axes(projection='3d')
    
    ax.set_xlim([-0.75, 0.75])
    ax.set_ylim([-0.8, 0])
    ax.set_zlim([0, 0.2])
    scale_x = 2
    scale_y = 1
    scale_z = 1
    ax.get_proj = lambda: np.dot(Axes3D.get_proj(ax), np.diag([scale_x, scale_y, scale_z, 1]))
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    #plt.show()
    return ax
    
def plot_cube(min_x, max_x, min_y, max_y, min_z, max_z, ax):    
    #squares in x-y plane
    xv = [min_x, min_x, max_x, max_x]
    yv = [min_y, max_y, max_y, min_y]
    zv = [min_z, min_z, min_z, min_z]
    verts = [list(zip(xv,yv,zv))]
    poly = Poly3DCollection(verts, linewidth=1, alpha=0.3)
    poly.set_edgecolor('k')
    poly.set_facecolor('saddlebrown')
    poly.set_alpha(0.1)
    ax.add_collection3d(poly)
    xv = [min_x, min_x, max_x, max_x]
    yv = [min_y, max_y, max_y, min_y]
    zv = [max_z, max_z, max_z, max_z]
    verts = [list(zip(xv,yv,zv))]
    poly = Poly3DCollection(verts, linewidth=1, alpha=0.3)
    poly.set_edgecolor('k')
    poly.set_facecolor('saddlebrown')
    poly.set_alpha(0.1)
    ax.add_collection3d(poly)
    
    #squares in x-z plane
    xv = [min_x, min_x, max_x, max_x]
    yv = [min_y, min_y, min_y, min_y]
    zv = [min_z, max_z, max_z, min_z]
    verts = [list(zip(xv,yv,zv))]
    poly = Poly3DCollection(verts, linewidth=1, alpha=0.3)
    poly.set_edgecolor('k')
    poly.set_facecolor('saddlebrown')
    poly.set_alpha(0.1)
    ax.add_collection3d(poly)
    xv = [min_x, min_x, max_x, max_x]
    yv = [max_y, max_y, max_y, max_y]
    zv = [min_z, max_z, max_z, min_z]
    verts = [list(zip(xv,yv,zv))]
    poly = Poly3DCollection(verts, linewidth=1, alpha=0.3)
    poly.set_edgecolor('k')
    poly.set_facecolor('saddlebrown')
    poly.set_alpha(0.1)
    ax.add_collection3d(poly)
    
    #squares in y-z plane
    xv = [min_x, min_x, min_x, min_x]
    yv = [min_y, min_y, max_y, max_y]
    zv = [min_z, max_z, max_z, min_z]
    verts = [list(zip(xv,yv,zv))]
    poly = Poly3DCollection(verts, linewidth=1, alpha=0.3)
    poly.set_edgecolor('k')
    poly.set_facecolor('saddlebrown')
    #poly.set_alpha(0.1)
    ax.add_collection3d(poly)
    xv = [max_x, max_x, max_x, max_x]
    yv = [min_y, min_y, max_y, max_y]
    zv = [min_z, max_z, max_z, min_z]
    verts = [list(zip(xv,yv,zv))]
    poly = Poly3DCollection(verts, linewidth=1, alpha=0.3)
    poly.set_edgecolor('k')
    poly.set_facecolor('saddlebrown')
    #poly.set_alpha(0.1)
    ax.add_collection3d(poly)
    return
    
def crash_detect(min_x, max_x, min_y, max_y, min_z, max_z, tf_data):
    safe = True
    
    for coordinate in tf_data[1]:
        if coordinate[0] > min_x and coordinate[0] < max_x:
            if coordinate[1] > min_y and coordinate[1] < max_y:
                if coordinate[2] > min_z and coordinate[2] < max_z:
                    safe = False
    if safe:
        print('Trajectory is safe to travel')
    else:
        print('Trajectory is NOT safe to travel')
    
    return safe
                
def plot_data(fnames):
    #joint_data, tf_data, wrench_data, gripper_data, min_x, max_x, min_y, max_y, min_z, max_z = read_data(fname)
    #Uncomment when testing AR findings
    successful_demos = []
    failed_demos = []
    ax = plot_xyz()
    min_x, max_x, min_y, max_y, min_z, max_z = find_ar()
    plot_cube(min_x, max_x, min_y, max_y, min_z, max_z, ax)
    for fname in fnames:
        joint_data, tf_data, wrench_data, gripper_data = read_data(fname)
        safe = crash_detect(min_x, max_x, min_y, max_y, min_z, max_z, tf_data)
        traj = tf_data[1][:, :3]
        n_pts_resample = 100
        traj = dp.DouglasPeuckerPoints(traj, n_pts_resample)
        if safe:
           ax.plot3D(tf_data[1][:, 0], tf_data[1][:, 1], tf_data[1][:, 2], 'g')
           successful_demos.append(np.transpose(traj))
        else:
           ax.plot3D(tf_data[1][:, 0], tf_data[1][:, 1], tf_data[1][:, 2], 'r')
           failed_demos.append(np.transpose(traj))
           
    K = 10000.0
    obj = TLFSD(copy.copy(successful_demos), copy.copy(failed_demos))
    obj.encode_GMMs(3)
    
    inds = [0, n_pts_resample - 1]
    
    for i in range(1):
        consts = successful_demos[i][:, inds]
        print(consts)
        #consts[:, 1] = final_pt
        traj_fsil = obj.get_successful_reproduction(K, inds, consts)
        fsil_traj, = ax.plot(traj_fsil[0, :], traj_fsil[1, :], traj_fsil[2, :], 'k', lw=5)
        #ax.plot3D(consts[0, 0], consts[1, 0], consts[2, 0], 'k.', ms=12, mew=3)
        #ax.plot3D(consts[0, 1], consts[1, 1], consts[2, 1], 'kx', ms=12, mew=3)
    plt.show()
    np.savetxt('ben_daniel_tlfsd_repro100.txt', traj_fsil)
    return
                
def main():
    filename1 = '/home/bhertel/catkin_ws/src/brendan_ur5e/src/scripts/recorded_demo 2022-04-08 16:21:56.h5'#raw_input('Enter the filename of the .h5 demo: ')
    filename2 = '/home/bhertel/catkin_ws/src/brendan_ur5e/src/scripts/recorded_demo 2022-04-08 16:22:45.h5'
    filename3 = '/home/bhertel/catkin_ws/src/brendan_ur5e/src/scripts/recorded_demo 2022-04-08 16:23:24.h5'
    filename4 = '/home/bhertel/catkin_ws/src/brendan_ur5e/src/scripts/recorded_demo 2022-04-08 16:28:33.h5'
    fnames = [filename1, filename2, filename3, filename4]
    plot_data(fnames)
    
if __name__ == '__main__':
    main()
