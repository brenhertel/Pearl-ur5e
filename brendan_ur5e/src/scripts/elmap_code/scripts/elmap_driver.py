from elmap_EM import *
from MapGeometry import *
from matlab_like_funcs import *
from rect2DMap import *
import numpy as np
import matplotlib.pyplot as plt
import db_downsample
import douglas_peucker
import kmeans_EM
import h5py
import os
import similaritymeasures
import time
import matplotlib as mpl
mpl.rc('font',family='Times New Roman')
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D

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
    
def perform_elmap(data, n=100, weighting='uniform', downsampling='naive', inds=None, consts=None):
    
    
    #weighting
    if (weighting == 'uniform'):
        nodeWeights=[]
    elif (weighting == 'curvature'):
        nodeWeights = [0]
        for i in range(1, len(data) - 1):
            weight = np.linalg.norm(data[i - 1] + data[i + 1] - 2 * data[i])
            nodeWeights.append(weight)
        nodeWeights.append(0)
        nodeWeights[0] = nodeWeights[1]
        nodeWeights[-1] = nodeWeights[-2]
        nodeWeights = np.array(nodeWeights).reshape((len(data), 1))
    elif (weighting == 'jerk'):
        nodeWeights = []
        for i in range(0, len(data)):
            if(i < 2 or i>len(data)-3):
                weight = 0
            else:
                weight = np.linalg.norm(data[i - 2] + 2*data[i - 1] - 2 * data[i+1] - data[i+2])
            nodeWeights.append(weight)
        nodeWeights[0] = nodeWeights[2]
        nodeWeights[1] = nodeWeights[2]
        nodeWeights[-1] = nodeWeights[-3]
        nodeWeights[-2] = nodeWeights[-3]
        nodeWeights = np.array(nodeWeights).reshape((len(data), 1))
    else:
        print('Weighting Method Unknown')
        
    if inds is not None:
        if consts is None:
            print('No constraints given, but indices are given. Cannot constrain reproduction.')
        else:
            for i in range(len(inds)):
                #print(np.shape(nodeWeights))
                #print(np.shape(data))
                nodeWeights = np.insert(nodeWeights, inds[i], np.size(data) / 100000, axis=0)
                data = np.insert(data, inds[i], consts[i], axis=0)
                #print(np.shape(nodeWeights))
                #print(np.shape(data))
        
    #downsampling
    if (downsampling == 'naive'):
        datad = downsample_traj(data, n)
    elif (downsampling == 'distance'):
        datad = db_downsample.db_downsample(data, n)
    elif (downsampling == 'douglas_peucker'):
        datad = douglas_peucker.DouglasPeuckerPoints(data, n)
    elif (downsampling == 'kmeans'):
        datad = kmeans_EM.new_kmeans(data, n)
        datad = np.sort(datad, axis=0)
    else:
        print('Downsampling Method Unknown')
        
    #initialize objects
    map = rect2DMap(n, 1)
    map.init(data, _type='random')
    map.mapped = datad
    #perform EM and return result
    EM(map, data, constStretching = 0.01, constBending = 0.001, weights=nodeWeights)
    nodes = map.getMappedCoordinates()
    return nodes
    
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
    
def get_my_trajn(skill_name, user_num=1, demo_num=1):
    # ask user for the file which the playback is for
    # filename = raw_input('Enter the filename of the .h5 demo: ')
    # open the file
    filename = '../h5 files/RAIL_' + skill_name + '.h5'
    hf = h5py.File(filename, 'r')
    # navigate to necessary data and store in numpy arrays
    user = hf.get('user' + str(user_num))
    demo = user.get('demo' + str(demo_num))
    pos_info = demo.get('pos')
    pos_data = np.transpose(np.array(pos_info))
    # close out file
    hf.close()
    return pos_data

def read_3D_h5(fname):
    #ask user for the file which the playback is for
    #filename = raw_input('Enter the filename of the .h5 demo: ')
    #open the file
    hf = h5py.File(fname, 'r')
    #navigate to necessary data and store in numpy arrays
    demo = hf.get('demo1')
    tf_info = demo.get('tf_info')
    pos_info = tf_info.get('pos_rot_data')
    pos_data = np.array(pos_info)
    
    x = pos_data[0, :]
    y = pos_data[1, :]
    z = pos_data[2, :]
    #close out file
    hf.close()
    return [x, y, z]
    
def calc_jerk(traj):
    (n_pts, n_dims) = np.shape(traj)
    ttl = 0.
    for i in range(2, n_pts - 2):
        ttl += np.linalg.norm(traj[i - 2] + 2*traj[i - 1] - 2 * traj[i+1] - traj[i+2])
    return ttl
    
def zip_demos(d):
    nd = len(d)
    (n_pts, n_dims) = np.shape(d[0])
    fd = np.zeros((n_pts * nd, n_dims))
    for i in range(n_pts):
        for id in range(nd):
            fd[(i * nd) + id][:] = d[id][i][:]
    return fd
    
def compare_gmm():
    #lambda = 0.1, mu = 100.0
    import sys
    # insert at 1, 0 is the script path (or '' in REPL)
    sys.path.insert(1, 'C:/Users/BH/Documents/GitHub/pearl_test_env/Guassian-Mixture-Models')
    from GMM_GMR import GMM_GMR
    
    skill_name = 'REACHING'
    num_users = 10
    num_demos = 2
    demo_nums = ['1', '3']
    weighting_scheme = 'curvature' #'curvature'
    downsample_alg = 'naive' #'distance'
                
    times = np.zeros((num_users, 2))
    fres = np.zeros((num_users, 2))
    crvs = np.zeros((num_users, 2))
    jerks = np.zeros((num_users, 2))
    print(np.shape(times))
    for i in range(num_users):
        demos = []
        #fig = plt.figure()
        #ax = fig.add_subplot(111, projection='3d')
        for j in range(num_demos):
            demos.append(get_my_trajn(skill_name, i + 1, demo_nums[j]))
            #ax.plot(demos[j][:, 0], demos[j][:, 1], demos[j][:, 2], label=str(j+1))
        #ax.legend()
        #plt.show()
        print(np.shape(demos[0]))
        
        plt_fpath = '../pictures/gmm_testing/user_' + str(i + 1) + '/'
        try:
            os.makedirs(plt_fpath)
        except OSError:
            print ("Creation of the directory %s failed" % plt_fpath)
        else:
            pass
            #print ("Successfully created the directory %s" % plt_fpath)
            
        traj = zip_demos(demos)
        ti = time.time()
        repro = perform_elmap(traj, n=100, weighting=weighting_scheme, downsampling=downsample_alg)
        tf = time.time()
        
        t = tf - ti
        
        frech = 0
        for j in range(num_demos):
            frech += similaritymeasures.frechet_dist(demos[j], repro)
        
        (nbNodes, nbDims) = np.shape(repro)
        L = 2.*np.diag(np.ones((nbNodes,))) - np.diag(np.ones((nbNodes-1,)),1) - np.diag(np.ones((nbNodes-1,)),-1)
        L[0,1] = -2.
        L[-1,-2] = -2.
        rep_crv = np.matmul(L, repro)
        
        crv = 0
        for j in range(num_demos):
            (nbNodes, nbDims) = np.shape(demos[j])
            L = 2.*np.diag(np.ones((nbNodes,))) - np.diag(np.ones((nbNodes-1,)),1) - np.diag(np.ones((nbNodes-1,)),-1)
            L[0,1] = -2.
            L[-1,-2] = -2.
            trj_crv = np.matmul(L, demos[j])
            
            crv += similaritymeasures.frechet_dist(trj_crv, rep_crv)
        
        jrk = calc_jerk(repro)
        
        print('ElMap-- time: %f, frechet: %f, curv_frechet: %f, jerk: %f' % (t, frech, crv, jrk)) 
        
        times[i, 0] = t
        fres[i, 0] = frech
        crvs[i, 0] = crv
        jerks[i, 0] = jrk
    
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        for j in range(num_demos):
            ax.plot(demos[j][:, 0], demos[j][:, 1], demos[j][:, 2], 'k', lw=5, alpha=0.4)
        ax.plot(repro[:, 0], repro[:, 1], repro[:, 2], 'r', lw=5)
        
        # GMM/GMR
        gmm_demos = []
        base_t = np.linspace(0, 1, len(demos[0]))
        new_t = np.linspace(0, 1, 100)
        for j in range(num_demos):
            gmm_demos.append(np.vstack((base_t, np.transpose(demos[j]))))
        gmm_data = np.hstack(gmm_demos)
        ti = time.time()
        gmmgmr = GMM_GMR(5)
        gmmgmr.fit(gmm_data)
        gmmgmr.predict(new_t)
        repro = np.transpose(gmmgmr.getPredictedData())[:, 1:4]
        tf = time.time()
        print(np.shape(repro))
        
        t = tf - ti
        
        frech = 0
        for j in range(num_demos):
            frech += similaritymeasures.frechet_dist(demos[j], repro)
        
        (nbNodes, nbDims) = np.shape(repro)
        L = 2.*np.diag(np.ones((nbNodes,))) - np.diag(np.ones((nbNodes-1,)),1) - np.diag(np.ones((nbNodes-1,)),-1)
        L[0,1] = -2.
        L[-1,-2] = -2.
        rep_crv = np.matmul(L, repro)
        
        crv = 0
        for j in range(num_demos):
            (nbNodes, nbDims) = np.shape(demos[j])
            L = 2.*np.diag(np.ones((nbNodes,))) - np.diag(np.ones((nbNodes-1,)),1) - np.diag(np.ones((nbNodes-1,)),-1)
            L[0,1] = -2.
            L[-1,-2] = -2.
            trj_crv = np.matmul(L, demos[j])
            
            crv += similaritymeasures.frechet_dist(trj_crv, rep_crv)
        
        jrk = calc_jerk(repro)
        
        print('GMM-- time: %f, frechet: %f, curv_frechet: %f, jerk: %f' % (t, frech, crv, jrk)) 
        
        times[i, 1] = t
        fres[i, 1] = frech
        crvs[i, 1] = crv
        jerks[i, 1] = jrk
        
        ax.plot(repro[:, 0], repro[:, 1], repro[:, 2], 'g', lw=5)
        plt.savefig(plt_fpath + 'repro.png', dpi=300, bbox_inches='tight')
        plt.close('all')
                
    np.savetxt('gmm_times.txt', times) 
    np.savetxt('gmm_frechets.txt', fres) 
    np.savetxt('gmm_curves.txt', crvs) 
    np.savetxt('gmm_jerks.txt', jerks)
    
    fp = h5py.File('gmm_results.h5', 'w')
    fp.create_dataset('times', data = times)
    fp.create_dataset('frechets', data = fres)
    fp.create_dataset('curves', data = crvs)
    fp.create_dataset('jerks', data = jerks)
    fp.close() #file cleanup
    
    #fp = h5py.File('lasa_results.h5', 'r')
    #times = np.array(fp.get('times'))
    #fres = np.array(fp.get('frechets'))
    #crvs = np.array(fp.get('curves'))
    #jerks = np.array(fp.get('jerks'))
    #fp.close() #file cleanup
    
    mean_times = np.mean(times, axis=0)
    print(np.shape(mean_times))
    print('MEAN TIMES')
    print(mean_times)
    
    mean_fres = np.mean(fres, axis=0)
    print('MEAN FRECHET')
    print(mean_fres)
    
    mean_crvs = np.mean(crvs, axis=0)
    print('MEAN CURVATURES')
    print(mean_crvs)
    
    mean_jerks = np.mean(jerks, axis=0)
    print('MEAN JERK')
    print(mean_jerks)
    
    np.savetxt('gmm_mtimes.txt', mean_times) 
    np.savetxt('gmm_mfrechets.txt', mean_fres) 
    np.savetxt('gmm_mcurves.txt', mean_crvs) 
    np.savetxt('gmm_mjerks.txt', mean_jerks)
    
def compare_n_vals():
    #lambda = 0.01, mu = 0.001
    weighting_scheme = 'curvature'
    downsample_alg = 'distance'
    lasa_names = ['Angle','BendedLine','CShape','DoubleBendedLine','GShape', \
                'heee','JShape','JShape_2','Khamesh','Leaf_1', \
                'Leaf_2','Line','LShape','NShape','PShape', \
                'RShape','Saeghe','Sharpc','Sine','Snake', \
                'Spoon','Sshape','Trapezoid','Worm','WShape', \
                'Zshape']
    
    N = np.arange(1, 301)
    
    times = np.zeros((len(lasa_names), len(N)))
    fres = np.zeros((len(lasa_names), len(N)))
    
    for j in range(len(lasa_names)):
        shape = lasa_names[j]
        [x, y] = get_lasa_trajn(shape)

        traj = np.hstack((np.reshape(x, (len(x), 1)), np.reshape(y, (len(y), 1))))
        for i in range(len(N)):
                
            ti = time.time()
            repro = perform_elmap(traj, n=N[i], weighting=weighting_scheme, downsampling=downsample_alg)
            tf = time.time()
            
            t = tf - ti
            
            frech = similaritymeasures.frechet_dist(traj, repro)
            times[j, i] = t
            fres[j, i] = frech
            print([shape, N[i], t, frech])
        
    np.savetxt('n_times.txt', times) 
    np.savetxt('n_frechets.txt', fres) 
        
    mfres = np.mean(fres, axis=0)
    mtimes = np.mean(times, axis=0)
        
    fig, ax1 = plt.subplots()

    color = 'tab:red'
    ax1.set_xlabel('N')
    ax1.set_ylabel('Similarity', color=color)
    ax1.set_yscale('log')
    ax1.plot(N, mfres, color=color)
    ax1.tick_params(axis='y', labelcolor=color)
    
    ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis
    
    color = 'tab:blue'
    ax2.set_ylabel('Time', color=color)  # we already handled the x-label with ax1
    ax2.plot(N, mtimes, color=color)
    ax2.tick_params(axis='y', labelcolor=color)
    
    fig.tight_layout()  # otherwise the right y-label is slightly clipped
    plt.savefig('../pictures/method_testing/N_effect.png', dpi=300, bbox_inches='tight')
    plt.show()
        
def compare_dmp():
    #lambda = 0.00001, mu = 0.00001
    import sys
    # insert at 1, 0 is the script path (or '' in REPL)
    sys.path.insert(1, './dmp_pastor_2009/')
    import perform_dmp as dmp
    
    skill_name = 'PUSHING'
    num_users = 10
    weighting_scheme = 'curvature' #'curvature'
    downsample_alg = 'distance' #'distance'
                
    times = np.zeros((num_users, 2))
    fres = np.zeros((num_users, 2))
    crvs = np.zeros((num_users, 2))
    jerks = np.zeros((num_users, 2))
    print(np.shape(times))
    for i in range(num_users):
        traj = get_my_trajn(skill_name, i + 1, 1)
        print(np.shape(traj))
        
        plt_fpath = '../pictures/dmp_testing/user_' + str(i + 1) + '/'
        try:
            os.makedirs(plt_fpath)
        except OSError:
            print ("Creation of the directory %s failed" % plt_fpath)
        else:
            pass
            #print ("Successfully created the directory %s" % plt_fpath)
            
        indeces = [0, -1]
        constants = [traj[indeces[0]] * 0.8, traj[indeces[1]]]
            
        ti = time.time()
        repro = perform_elmap(traj, n=100, weighting=weighting_scheme, downsampling=downsample_alg, inds=indeces, consts=constants)
        tf = time.time()
        
        t = tf - ti
        
        frech = similaritymeasures.frechet_dist(traj, repro)
        
        (nbNodes, nbDims) = np.shape(traj)
        L = 2.*np.diag(np.ones((nbNodes,))) - np.diag(np.ones((nbNodes-1,)),1) - np.diag(np.ones((nbNodes-1,)),-1)
        L[0,1] = -2.
        L[-1,-2] = -2.
        trj_crv = np.matmul(L, traj)
        
        (nbNodes, nbDims) = np.shape(repro)
        L = 2.*np.diag(np.ones((nbNodes,))) - np.diag(np.ones((nbNodes-1,)),1) - np.diag(np.ones((nbNodes-1,)),-1)
        L[0,1] = -2.
        L[-1,-2] = -2.
        rep_crv = np.matmul(L, repro)
        
        crv = similaritymeasures.frechet_dist(trj_crv, rep_crv)
        
        jrk = calc_jerk(repro)
        
        print('ElMap-- time: %f, frechet: %f, curv_frechet: %f, jerk: %f' % (t, frech, crv, jrk)) 
        
        times[i, 0] = t
        fres[i, 0] = frech
        crvs[i, 0] = crv
        jerks[i, 0] = jrk
    
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], 'k', lw=5)
        ax.plot(repro[:, 0], repro[:, 1], repro[:, 2], 'r:', lw=5)
        for j in range(len(indeces)):
            ax.plot(constants[j][0], constants[j][1], constants[j][2], 'mo', ms=15, mew=5)
        
        # DMPs
        trajd = db_downsample.db_downsample(traj, 100)
        ti = time.time()
        repro = dmp.perform_dmp_general(trajd, constants, indeces)
        tf = time.time()
        
        t = tf - ti
        
        frech = similaritymeasures.frechet_dist(traj, repro)
        
        (nbNodes, nbDims) = np.shape(traj)
        L = 2.*np.diag(np.ones((nbNodes,))) - np.diag(np.ones((nbNodes-1,)),1) - np.diag(np.ones((nbNodes-1,)),-1)
        L[0,1] = -2.
        L[-1,-2] = -2.
        trj_crv = np.matmul(L, traj)
        
        (nbNodes, nbDims) = np.shape(repro)
        L = 2.*np.diag(np.ones((nbNodes,))) - np.diag(np.ones((nbNodes-1,)),1) - np.diag(np.ones((nbNodes-1,)),-1)
        L[0,1] = -2.
        L[-1,-2] = -2.
        rep_crv = np.matmul(L, repro)
        
        crv = similaritymeasures.frechet_dist(trj_crv, rep_crv)
        
        jrk = calc_jerk(repro)
        
        print('DMP-- time: %f, frechet: %f, curv_frechet: %f, jerk: %f' % (t, frech, crv, jrk)) 
        
        times[i, 1] = t
        fres[i, 1] = frech
        crvs[i, 1] = crv
        jerks[i, 1] = jrk
        
        ax.plot(repro[:, 0], repro[:, 1], repro[:, 2], 'g:', lw=5)
        plt.savefig(plt_fpath + 'repro.png', dpi=300, bbox_inches='tight')
        plt.close('all')
                
    np.savetxt('dmp_times.txt', times) 
    np.savetxt('dmp_frechets.txt', fres) 
    np.savetxt('dmp_curves.txt', crvs) 
    np.savetxt('dmp_jerks.txt', jerks)
    
    fp = h5py.File('dmp_results.h5', 'w')
    fp.create_dataset('times', data = times)
    fp.create_dataset('frechets', data = fres)
    fp.create_dataset('curves', data = crvs)
    fp.create_dataset('jerks', data = jerks)
    fp.close() #file cleanup
    
    #fp = h5py.File('lasa_results.h5', 'r')
    #times = np.array(fp.get('times'))
    #fres = np.array(fp.get('frechets'))
    #crvs = np.array(fp.get('curves'))
    #jerks = np.array(fp.get('jerks'))
    #fp.close() #file cleanup
    
    mean_times = np.mean(times, axis=0)
    print(np.shape(mean_times))
    print('MEAN TIMES')
    print(mean_times)
    
    mean_fres = np.mean(fres, axis=0)
    print('MEAN FRECHET')
    print(mean_fres)
    
    mean_crvs = np.mean(crvs, axis=0)
    print('MEAN CURVATURES')
    print(mean_crvs)
    
    mean_jerks = np.mean(jerks, axis=0)
    print('MEAN JERK')
    print(mean_jerks)
    
    np.savetxt('dmp_mtimes.txt', mean_times) 
    np.savetxt('dmp_mfrechets.txt', mean_fres) 
    np.savetxt('dmp_mcurves.txt', mean_crvs) 
    np.savetxt('dmp_mjerks.txt', mean_jerks)
    
def multidemo_test():
    plt_fpath = '../pictures/exp1_test/'
    try:
        os.makedirs(plt_fpath)
    except OSError:
        print ("Creation of the directory %s failed" % plt_fpath)
    else:
        print ("Successfully created the directory %s" % plt_fpath)
        
    fig = plt.figure()
    demos = []
    for i in range(7):
        [x, y] = get_lasa_trajn('RShape', i + 1)
    
        traj = np.hstack((np.reshape(x, (len(x), 1)), np.reshape(y, (len(y), 1))))
    
        demos.append(traj)
        og, = plt.plot(traj[:, 0], traj[:, 1], 'k', lw=6)
    zip_data = zip_demos(demos)
    
    indeces = [3500]
    constants = [[20, 50]]

    repro = perform_elmap(zip_data, n=100, weighting='curvature', downsampling='distance', inds=indeces, consts=constants)
    rep, = plt.plot(repro[:, 0], repro[:, 1], 'r', lw=6)
    for i in range(len(indeces)):
        cst, = plt.plot(constants[i][0], constants[i][1], 'ko', ms=15, mew=5)
    plt.xticks([])
    plt.yticks([])
    plt.legend((og, rep, cst), ('Demonstration', 'Reproduction', 'Constraints'), fontsize='xx-large')#, handlelength=3.0)
    plt.savefig(plt_fpath + 'multidemo.png', dpi=300, bbox_inches='tight')
    plt.show()

def consts_test():

    plt_fpath = '../pictures/exp1_test/'
    try:
        os.makedirs(plt_fpath)
    except OSError:
        print ("Creation of the directory %s failed" % plt_fpath)
    else:
        print ("Successfully created the directory %s" % plt_fpath)
        
    [x, y] = get_lasa_trajn('RShape')

    traj = np.hstack((np.reshape(x, (len(x), 1)), np.reshape(y, (len(y), 1))))
    
    indeces = [0, 500, -1]
    constants = [[40, 40], [20, 35], [0, 0]]

    repro = perform_elmap(traj, n=100, weighting='curvature', downsampling='distance', inds=indeces, consts=constants)
    fig = plt.figure()
    og, = plt.plot(traj[:, 0], traj[:, 1], 'k', lw=6)
    rep, = plt.plot(repro[:, 0], repro[:, 1], 'r', lw=6)
    for i in range(len(indeces)):
        cst, = plt.plot(constants[i][0], constants[i][1], 'mo', ms=15, mew=5)
    plt.xticks([])
    plt.yticks([])
    plt.legend((og, rep, cst), ('Demonstration', 'Reproduction', 'Constraints'), fontsize='xx-large')#, handlelength=3.0)
    plt.savefig(plt_fpath + 'constraints1.png', dpi=300, bbox_inches='tight')
    plt.show()
    
    
def lasa_test():
    #lasa_names = ['Angle','BendedLine','CShape','DoubleBendedLine','GShape', \
    #            'heee','JShape','JShape_2','Khamesh','Leaf_1', \
    #            'Leaf_2','Line','LShape','NShape','PShape', \
    #            'RShape','Saeghe','Sharpc','Sine','Snake', \
    #            'Spoon','Sshape','Trapezoid','Worm','WShape', \
    #            'Zshape']
    #weighting_schemes = ['uniform', 'curvature', 'jerk']
    #downsample_algs = ['naive', 'distance', 'douglas_peucker']
    #            
    #times = np.zeros((len(lasa_names), len(weighting_schemes), len(downsample_algs)))
    #fres = np.zeros((len(lasa_names), len(weighting_schemes), len(downsample_algs)))
    #crvs = np.zeros((len(lasa_names), len(weighting_schemes), len(downsample_algs)))
    #jerks = np.zeros((len(lasa_names), len(weighting_schemes), len(downsample_algs)))
    #print(np.shape(times))
    #for i in range(len(lasa_names)):
    #    shape = lasa_names[i]
    #    [x, y] = get_lasa_trajn(shape)
    #    traj = np.hstack((np.reshape(x, (len(x), 1)), np.reshape(y, (len(y), 1))))
    #    
    #    for j in range(len(weighting_schemes)):
    #        weights = weighting_schemes[j]
    #        
    #        for k in range(len(downsample_algs)):
    #            downsample = downsample_algs[k]
    #            plt_fpath = '../pictures/lasa_testing/' + shape + '/' + weights + '/' + downsample + '/'
    #            try:
    #                os.makedirs(plt_fpath)
    #            except OSError:
    #                print ("Creation of the directory %s failed" % plt_fpath)
    #            else:
    #                pass
    #                #print ("Successfully created the directory %s" % plt_fpath)
    #        
    #            ti = time.time()
    #            repro = perform_elmap(traj, n=100, weighting=weights, downsampling=downsample)
    #            tf = time.time()
    #            t = tf - ti
    #            frech = similaritymeasures.frechet_dist(traj, repro)
    #            
    #            (nbNodes, nbDims) = np.shape(traj)
    #            L = 2.*np.diag(np.ones((nbNodes,))) - np.diag(np.ones((nbNodes-1,)),1) - np.diag(np.ones((nbNodes-1,)),-1)
    #            L[0,1] = -2.
    #            L[-1,-2] = -2.
    #            trj_crv = np.matmul(L, traj)
    #            
    #            (nbNodes, nbDims) = np.shape(repro)
    #            L = 2.*np.diag(np.ones((nbNodes,))) - np.diag(np.ones((nbNodes-1,)),1) - np.diag(np.ones((nbNodes-1,)),-1)
    #            L[0,1] = -2.
    #            L[-1,-2] = -2.
    #            rep_crv = np.matmul(L, repro)
    #            
    #            crv = similaritymeasures.frechet_dist(trj_crv, rep_crv)
    #            
    #            jrk = calc_jerk(repro)
    #            
    #            print(shape + '/' + weights + '/' + downsample + ' time: %f, frechet: %f, curv_frechet: %f, jerk: %f' % (t, frech, crv, jrk)) 
    #            
    #            times[i, j, k] = t
    #            fres[i, j, k] = frech
    #            crvs[i, j, k] = crv
    #            jerks[i, j, k] = jrk
    #
    #            plt.figure()
    #            plt.plot(traj[:, 0], traj[:, 1], 'k', lw=5)
    #            plt.plot(repro[:, 0], repro[:, 1], 'g:', lw=5)
    #            plt.savefig(plt_fpath + 'repro.png', dpi=300, bbox_inches='tight')
    #            plt.close('all')
    #            
    #np.savetxt('times.txt', times) 
    #np.savetxt('frechets.txt', fres) 
    #np.savetxt('curves.txt', crvs) 
    #np.savetxt('jerks.txt', jerks)
    
    #fp = h5py.File('lasa_results.h5', 'w')
    #fp.create_dataset('times', data = times)
    #fp.create_dataset('frechets', data = fres)
    #fp.create_dataset('curves', data = crvs)
    #fp.create_dataset('jerks', data = jerks)
    #fp.close() #file cleanup
    
    fp = h5py.File('lasa_results.h5', 'r')
    times = np.array(fp.get('times'))
    fres = np.array(fp.get('frechets'))
    crvs = np.array(fp.get('curves'))
    jerks = np.array(fp.get('jerks'))
    fp.close() #file cleanup
    
    mean_times = np.mean(times, axis=0)
    print(np.shape(mean_times))
    print('MEAN TIMES')
    print(mean_times)
    
    mean_fres = np.mean(fres, axis=0)
    print('MEAN FRECHET')
    print(mean_fres)
    
    mean_crvs = np.mean(crvs, axis=0)
    print('MEAN CURVATURES')
    print(mean_crvs)
    
    mean_jerks = np.mean(jerks, axis=0)
    print('MEAN JERK')
    print(mean_jerks)
    
    np.savetxt('mtimes.txt', mean_times) 
    np.savetxt('mfrechets.txt', mean_fres) 
    np.savetxt('mcurves.txt', mean_crvs) 
    np.savetxt('mjerks.txt', mean_jerks)
    
def main3D_test():
    import tkinter
    import matplotlib
    matplotlib.use('TkAgg')
    [x, y, z] = read_3D_h5('../../recorded_demo Tue Apr 20 09:53:21 2021.h5')

    plt.figure()
    plt.plot(x)
    plt.title('x')
    plt.figure()
    plt.plot(y)
    plt.title('y')
    plt.figure()
    plt.plot(z)
    plt.title('z')
    plt.show()

    traj = np.hstack((np.reshape(x, (len(x), 1)), np.reshape(y, (len(y), 1)), np.reshape(z, (len(z), 1))))

    trajd = downsample_traj(traj)

    indeces = [1580, 3060]
    constants = [traj[indeces[0]], traj[indeces[1]]]
    for i in range(len(indeces)):
    	constants[i][2] = constants[i][2] - 0.005
    repro = perform_elmap(traj, n=100, weighting='curvature', downsampling='distance', inds=indeces, consts=constants)
    
    #plt.figure()
    #plt.plot(repro[:, 0])
    #plt.title('x')
    #plt.figure()
    #plt.plot(repro[:, 1])
    #plt.title('y')
    #plt.figure()
    #plt.plot(repro[:, 2])
    #plt.title('z')
    #plt.show()
    
    np.savetxt('pressing_repro_const_mod.txt', repro)
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], 'k', lw=3)
    ax.plot(trajd[:, 0], trajd[:, 1], trajd[:, 2], 'g', lw=3)
    ax.plot(repro[:, 0], repro[:, 1], repro[:, 2], 'r--', lw=3)
    plt.show()
    
def main_test():
    [x, y] = get_lasa_trajn('WShape')

    traj = np.hstack((np.reshape(x, (len(x), 1)), np.reshape(y, (len(y), 1))))

    trajd = downsample_traj(traj)

    repro = perform_elmap(traj, n=100, weighting='uniform', downsampling='naive')
    
    
    plt.figure()
    plt.plot(traj[:, 0], traj[:, 1], 'k', lw=3)
    plt.plot(trajd[:, 0], trajd[:, 1], 'g', lw=3)
    plt.plot(repro[:, 0], repro[:, 1], 'r--', lw=3)
    plt.show()
    
    
if __name__ == '__main__':
    main3D_test()
