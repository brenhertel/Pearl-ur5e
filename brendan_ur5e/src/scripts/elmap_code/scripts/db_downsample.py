import numpy as np

#downsample a trajectory of n points to m points based on distance between those points

#function to get the total distance of a n x d trajectory
#arguments
#traj: nxd vector, where n is number of points and d is number of dims
#returns the total distance of traj, calculated using euclidean distance  
def get_traj_dist(traj):
    dist = 0.
    for n in range(len(traj) - 1):
        dist = dist + np.linalg.norm(traj[n + 1] - traj[n])
    #if (DEBUG):
    #    print('Traj total dist: %f' % (dist))
    return dist
    
#downsample to a certain number of points
def db_downsample(traj, new_len):
    (n_pts, n_dims) = np.shape(traj)
    total_dist = get_traj_dist(traj)
    interval_len = total_dist / (new_len - 1)
    sum_len = 0.0
    out_traj = np.zeros((new_len, n_dims))
    ind = 0
    for n in range(n_pts - 1):
        if (sum_len >= 0.0):
            out_traj[ind, :] = traj[n, :]
            ind += 1
            sum_len -= interval_len
        sum_len += np.linalg.norm(traj[n + 1] - traj[n])
    out_traj[-1, :] = traj[-1, :]
    return out_traj
    
#downsample to a certain distance between points
def db_downsample_dist(traj, seg_len):
    (n_pts, n_dims) = np.shape(traj)
    interval_len = seg_len
    sum_len = 0.0
    out_traj = np.zeros((n_pts, n_dims))
    ind = 0
    for n in range(n_pts - 1):
        if (sum_len >= 0.0):
            out_traj[ind, :] = traj[n, :]
            ind += 1
            sum_len -= interval_len
        sum_len += np.linalg.norm(traj[n + 1] - traj[n])
    out_traj[ind, :] = traj[-1, :]
    ind += 1
    out_traj = out_traj[0:ind]
    return out_traj

#main program for testing    
if __name__ == '__main__':
    x = np.linspace(0, 10, 1000)
    #y = np.sin(x)
    y = (x - 5)**3
    traj = np.transpose(np.vstack((x, y)))
    ds_traj = db_downsample(traj, 10)
    import matplotlib.pyplot as plt
    plt.figure()
    plt.plot(traj[:, 0], traj[:, 1], 'b', lw=3)
    plt.plot(ds_traj[:, 0], ds_traj[:, 1], 'r-.', lw=3, ms=7)
    plt.show()
    
    