import cProfile
import pstats
import time

import numpy as np
import matplotlib.pyplot as plt
import h5py
from scipy.optimize import minimize

# function to downsample a 1 dimensional trajectory to n points
# arguments
# traj: nxd vector, where n is number of points and d is number of dims
# n (optional): the number of points in the downsampled trajectory. Default is 100.
# returns the trajectory downsampled to n points
def downsample_traj(traj, n=100):
    (n_pts, n_dims) = np.shape(traj)
    npts = np.linspace(0, n_pts - 1, n)
    out = np.zeros((n, n_dims))
    for i in range(n):
        out[i][:] = traj[int(npts[i])][:]
    return out
    
class ElMap(object):

    def __init__(self, given_data):
        self.data = given_data
        (self.n_pts, self.n_dims) = np.shape(self.data)
        
    def init_map(self, init_data):
        #self.map = downsample_traj(self.data, numNodes)
        self.map = init_data
        self.initialMap = self.map
        (self.n_map, self.dims) = np.shape(self.map)
        
    def init_weights(self, weights=None, weighting_scheme="curvature", stretching=0.0, bending=0.0):
        self.lmbda = stretching
        self.mu = bending
        if weights is not None:
            self.nodeWeights = weights
        else:
            curveWeightings = {"uniform": self.uniformWeights, "curvature":self.weightsByCurvature, "jerk":self.weightsByJerk}
            curveWeightings[weighting_scheme]()
        print(self.nodeWeights)
        self.s_weights = sum(self.nodeWeights)
        print(self.s_weights)
        
    def uniformWeights(self):
        self.nodeWeights = [1 for n in range(self.n_pts)]
        
    def weightsByCurvature(self):
        self.nodeWeights = [0]
        for i in range(1, self.n_pts - 1):
            weight = np.linalg.norm(self.data[i - 1] + self.data[i + 1] - 2 * self.data[i])
            self.nodeWeights.append(weight)
        self.nodeWeights.append(0)
        self.nodeWeights[0] = self.nodeWeights[1]
        self.nodeWeights[-1] = self.nodeWeights[-2]

    def weightsByJerk(self):
        self.nodeWeights = []
        for i in range(0, self.n_pts):
            if(i < 2 or i>self.n_pts-2):
                weight = 0
            else:
                weight = np.linalg.norm(self.data[i - 2] + 2*self.data[i - 1] - 2 * self.data[i+1] - self.data[i-1])
            self.nodeWeights.append(weight)
        self.nodeWeights[0] = self.nodeWeights[2]
        self.nodeWeights[1] = self.nodeWeights[2]
        self.nodeWeights[-1] = self.nodeWeights[-3]
        self.nodeWeights[-2] = self.nodeWeights[-3]
            
            
    #EM using algorithm found here: https://arxiv.org/ftp/arxiv/papers/0809/0809.0490.pdf (page 16)
    def calc_matrices(self):
        S = np.zeros((self.n_map, self.n_map))
        s_sub = np.array([[ self.mu / 4, -self.mu / 2,  self.mu / 4],
                          [-self.mu / 2,      self.mu, -self.mu / 2],
                          [ self.mu / 4, -self.mu / 2,  self.mu / 4]])
        for m in range(1, self.n_map - 1):
            S[m-1:m+2, m-1:m+2] += s_sub
            
        E = np.zeros((self.n_map, self.n_map))
        e_sub = np.array([[ self.lmbda, -self.lmbda],
                          [-self.lmbda,  self.lmbda]])
        for m in range(self.n_map - 1):
            E[m:m+2, m:m+2] += e_sub
        self.a_base = S + E
        
    
    def map_expectation(self):
        self.clustered_points = [[] for m in range(self.n_map)]
        # going through and assigning clusters
        
        #go through base data points, find lowest distance map node & assign
        for n in range(self.n_pts):
            dists = np.zeros((self.n_map))
            for m in range(self.n_map):
                dists[m] = np.linalg.norm(self.data[n] - self.map[m])
            self.clustered_points[np.argmin(dists)].append(n)
        return
        
    def map_maximization(self):
        delta = np.identity(self.n_map) / self.s_weights
        rhs = np.zeros((self.n_map, self.n_dims))
        for m in range(self.n_map):
            delta[m, m] *= sum(map(self.nodeWeights.__getitem__, self.clustered_points[m]))
            print(self.clustered_points[m])
            for ind in range(len(self.clustered_points[m])):
                rhs[m, :] += self.nodeWeights[ind] * self.data[ind, :]
        A = self.a_base + delta
        rhs /= self.s_weights
        prev_map = self.map
        print(A)
        print(rhs)
        #self.map, _, _, _ = np.linalg.lstsq(A, rhs, rcond=-1)
        self.map = np.linalg.solve(A, rhs)
        U = self.calc_Uy() + self.calc_Ue() + self.calc_Ur()
        return U
        
    def map_EM(self, tol=2, max_iters=1):
        #assume step 1 is done and map is initialized
        #self.calc_matrices() #step 2
        iter = 0
        d = 1e20
        U = 1e20
        t0 = time.time()
        print('Starting optimization at time t = %f' % (time.time()))

        while(iter < max_iters) and (d > tol): #step 6
            self.map_expectation() #step 3
            res = minimize(self.calc_costs, self.map.flatten(), tol=.1, method="Powell")
            print(res.x)
            nU = self.calc_costs(res.x)
            d = U - nU
            U = nU
            #self.map = np.reshape(res.x, ((self.n_map, self.dims)))
            #d = self.map_maximization() #step 4 & 5
            #print('Iteration: %d, Distance: %f, y: %f, e: %f, r:%f' %(iter, d, self.calc_Uy(), self.calc_Ue(), self.calc_Ur()))
            print('Iteration: %d, Energy: %f, time: %f' % (iter, U, time.time() - t0))
            iter += 1
        t1 = time.time()
        print("total time elapsed ", t1-t0)
        return self.map

    def calc_Uy(self):
        cost = 0.
        for m in range(self.n_map):  # i contains index of map_node
            #print(self.clustered_points[m])
            for ind in range(len(self.clustered_points[m])):
                cost += self.nodeWeights[ind] * np.linalg.norm(self.data[ind] - self.map[m])**2
        return cost #/ self.s_weights

    def calc_Ue(self):
        cost = 0.
        for m in range(self.n_map - 1):
            cost = cost + np.linalg.norm(self.map[m] - self.map[m + 1])**2
        return cost * self.lmbda

    def calc_Ur(self):
        cost = 0.
        for m in range(1, self.n_map - 1):
            cost = cost + np.linalg.norm(self.map[m - 1] + self.map[m + 1] - 2 * self.map[m])**2
        return cost * self.mu

    def calc_costs(self, X):
        self.map = np.reshape(X, ((self.n_map, self.dims)))
        cost = self.calc_Ue() + self.calc_Ur() + self.calc_Uy()
        print(cost)
        return cost

    def NRG_calc(self, aMap):
        nrg = np.zeros((self.n_map, self.n_dims))
        # find energy of initial point
        # NRG to other points Ux
        for ind in range(len(self.clustered_points[0])):
            for d in range(self.n_dims):
                nrg[0, d] += self.nodeWeights[ind] * np.linalg.norm(self.data[ind][d] - aMap[0, d]) ** 2
        if (len(self.clustered_points[0]) > 0):
            nrg[0] = nrg[0] / sum(map(self.nodeWeights.__getitem__, self.clustered_points[0]))
        # Stretching NRG, Ue
        for d in range(self.n_dims):
            nrg[0, d] += self.lmbda * np.linalg.norm(aMap[1, d] - aMap[0, d]) ** 2
        # no bending NRG for first node

        # find energy of other points (until last point
        for m in range(1, self.n_map - 1):
            # NRG to other points Ux
            for ind in range(len(self.clustered_points[m])):
                for d in range(self.n_dims):
                    nrg[m, d] += self.nodeWeights[ind] * np.linalg.norm(self.data[ind][d] - aMap[m, d]) ** 2
            if (len(self.clustered_points[m]) > 0):
                nrg[m] = nrg[m] / sum(map(self.nodeWeights.__getitem__, self.clustered_points[m]))
            # Stretching NRG, Ue
            for d in range(self.n_dims):
                nrg[m, d] += self.lmbda * np.linalg.norm(aMap[m - 1, d] - aMap[m, d]) ** 2
                nrg[m, d] += self.lmbda * np.linalg.norm(aMap[m + 1, d] - aMap[m, d]) ** 2
            # Bending NRG, Ur
            for d in range(self.n_dims):
                nrg[m, d] += self.mu * np.linalg.norm(aMap[m - 1, d] - 2 * aMap[m, d] + aMap[m + 1, d]) ** 2

        # find NRG for last node
        # NRG to other points Ux
        for ind in range(len(self.clustered_points[-1])):
            for d in range(self.n_dims):
                nrg[-1, d] += self.nodeWeights[ind] * np.linalg.norm(self.data[ind][d] - aMap[-1, d]) ** 2
        if (len(self.clustered_points[-1]) > 0):
            nrg[-1] = nrg[-1] / sum(map(self.nodeWeights.__getitem__, self.clustered_points[-1]))
        # Stretching NRG, Ue
        for d in range(self.n_dims):
            nrg[-1, d] += self.lmbda * np.linalg.norm(aMap[-2, d] - aMap[-1, d]) ** 2
        # no bending NRG for last node

        # calc total NRG
        U = np.linalg.norm(nrg)

        ##move points to new locations
        ####### NOT SURE IF THIS IS CORRECT ######
        # x = np.array(self.nodeWeights).reshape((self.n_pts, 1)) * self.data / sum(self.nodeWeights)
        # xmap = np.zeros((self.n_map, self.n_dims))
        # for m in range(self.n_map):
        #    for ind in range(len(self.clustered_points[m])):
        #        xmap[m] += x[ind]
        # print(np.shape(nrg))
        # print(np.shape(xmap))
        # map, _, _, _ = np.linalg.lstsq(nrg, xmap, rcond=-1)
        # print(np.shape(map))
        return U
        
    def min_fx(self, X):
        self.map = X.reshape((self.n_map, self.n_dims))
        U = self.NRG_calc(self.map)
        print(U)
        return U
        
    def map_minimizer(self):
        self.map_expectation()
        from scipy.optimize import minimize
        res = minimize(self.min_fx, self.map.flatten(), tol=0.1)
        return self.map
        
    def bad_map_EM(self, termination='iter', termination_cond=20):
        last_U = 1e20 #some large number
        dU = 1e20 #some large number
        if termination == 'iter':
            for _ in range(termination_cond):
                self.map_expectation()
                U = self.NRG_calc(self.map)
                print('Iteration: %d, Energy: %f' % (_, U))
        else:
            while (dU > termination_cond):
                self.map_expectation()
                new_U = self.NRG_calc(self.map)
                dU = last_U = new_U
                last_U = new_U
                print('Change in Energy: %f, New Energy: %f' % (dU, new_U))
        return self.map
        
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


def main_single_demo():
    [x, y] = get_lasa_trajn('WShape')
    numberOfNodes = 20
    traj = np.hstack((np.reshape(x, (len(x), 1)), np.reshape(y, (len(y), 1))))
    #profiler = cProfile.Profile()
    #profiler.enable()


    trajd = downsample_traj(traj, n=100)
    
    #diff = traj[-1] - traj[0]
    #bad initial map for testing optimization
    #startMap = np.array([traj[0]+diff*i/20. for i in range(20)])
    
    elmap = ElMap(traj)
    elmap.init_map(trajd)
    elmap.init_weights()
    repro = elmap.map_EM()
    
    #print(repro)
    #profiler.disable()
    #stats = pstats.Stats(profiler).sort_stats('tottime')
    #stats.print_stats()
    
    plt.figure()
    plt.plot(traj[:, 0], traj[:, 1], 'k', lw=3)
    plt.plot(trajd[:, 0], trajd[:, 1], 'g', lw=3)
    plt.plot(repro[:, 0], repro[:, 1], 'r--', lw=3)
    plt.show()
    
if __name__ == '__main__':
    main_single_demo()