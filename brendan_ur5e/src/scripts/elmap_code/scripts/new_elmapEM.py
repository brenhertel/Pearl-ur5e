import numpy as np
import matplotlib.pyplot as plt
import h5py

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
    
class ElMap(object):

    def __init__(self, given_data):
        self.data = given_data
        (self.n_pts, self.n_dims) = np.shape(self.data)
        
    def init_map(self, given_map):
        self.map = given_map
        (self.n_map, _) = np.shape(self.map)
        
    def init_weights(self, weights=None, weighting_scheme="curvature", stretching=0.0001, bending=0.0001):
        self.lmbda = stretching
        self.mu = bending
        if weights is not None:
            self.nodeWeights = weights
        else:
            curveWeightings = {"uniform": self.uniformWeights, "curvature":self.weightsByCurvature, "jerk":self.weightsByJerk}
            curveWeightings[weighting_scheme]()
        self.s_weights = sum(self.nodeWeights)
        
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
        #s_sub = np.array([[ self.mu / 4, -self.mu / 2,  self.mu / 4],
        #                  [-self.mu / 2,      self.mu, -self.mu / 2],
        #                  [ self.mu / 4, -self.mu / 2,  self.mu / 4]])
        #for m in range(1, self.n_map - 1):
        #    S[m-1:m+2, m-1:m+2] += s_sub
        s_sub = np.array([ self.mu, -2 * self.mu,  self.mu])
        for m in range(1, self.n_map - 1):
            S[m, m-1:m+2] += s_sub
            
        E = np.zeros((self.n_map, self.n_map))
        #e_sub = np.array([[ self.lmbda, -self.lmbda],
        #                  [-self.lmbda,  self.lmbda]])
        #for m in range(self.n_map - 1):
        #    E[m:m+2, m:m+2] += e_sub
        e_sub = np.array([ self.lmbda, -self.lmbda])
        for m in range(self.n_map - 1):
            E[m, m:m+2] += e_sub
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
            #if (len(self.clustered_points[m]) > 0):
            #    rhs[m, :] /= sum(map(self.nodeWeights.__getitem__, self.clustered_points[m]))
        A = self.a_base + delta
        rhs /= self.s_weights
        #prev_map = self.map
        print(A)
        print(rhs)
        self.map, _, _, _ = np.linalg.lstsq(A, rhs, rcond=-1)
        #self.map = np.linalg.solve(A, rhs)
        #dist = np.linalg.norm(prev_map - self.map)
        U = self.calc_Uy() + self.calc_Ue() + self.calc_Ur()
        return U
        
    def map_EM(self, tol=2.0, max_iters=50):
        #assume step 1 is done and map is initialized
        #self.lmbda = 0
        #self.mu = 0
        self.calc_matrices() #step 2
        iter = 0
        dU = 1e20
        lU = 1e20
        self.lmbda = 1e3
        self.mu = 1e3
        #self.lmbda = 0
        #self.mu = 0
        while(iter < max_iters) and (dU > tol): #step 6
            #self.calc_matrices()
            self.map_expectation() #step 3
            U = self.my_maximization() #step 4 & 5
            print('Iteration: %d, Energy: %f, params: %f, %f' % (iter, U, self.lmbda, self.mu))
            iter += 1
            dU = lU - U
            lU = U
            self.lmbda *= 0.8
            self.mu *= 0.8
        return self.map
        
    def calc_Uy(self):
        cost = 0.
        for m in range(self.n_map):  # i contains index of map_node
            for ind in range(len(self.clustered_points[m])):
                cost += self.nodeWeights[ind] * np.linalg.norm(self.data[ind] - self.map[m])**2
        return cost / self.s_weights

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
        
    def my_maximization(self):
        nrg = np.zeros((self.n_map, self.n_dims))
        force = np.zeros((self.n_map, self.n_dims))
        
        #find energy of initial point
        #NRG to other points Ux
        for ind in range(len(self.clustered_points[0])):
            for d in range(self.n_dims):
                nrg[0, d] += self.nodeWeights[ind] * (self.data[ind][d] - self.map[0, d])**2
                force[0, d] += self.nodeWeights[ind] * (self.data[ind][d] - self.map[0, d])
        #if (len(self.clustered_points[0]) > 0):
        #    nrg[0] = nrg[0] / sum(map(self.nodeWeights.__getitem__, self.clustered_points[0]))
        
        #Stretching NRG, Ue
        for d in range(self.n_dims):
            nrg[0, d] += self.lmbda * (self.map[1, d] - self.map[0, d])**2
            force[0, d] += self.lmbda * (self.map[1, d] - self.map[0, d])
        # no bending NRG for first node
        
        #find energy of other points (until last point
        for m in range(1, self.n_map - 1):
            #NRG to other points Ux
            for ind in range(len(self.clustered_points[m])):
                for d in range(self.n_dims):
                    nrg[m, d] += self.nodeWeights[ind] * np.linalg.norm(self.data[ind][d] - self.map[m, d])**2
                    force[m, d] += self.nodeWeights[ind] * (self.data[ind][d] - self.map[m, d])
            
            #if (len(self.clustered_points[m]) > 0):
            #    nrg[m] = nrg[m] / sum(map(self.nodeWeights.__getitem__, self.clustered_points[m]))
            
            #Stretching NRG, Ue
            for d in range(self.n_dims):
                nrg[m, d] += self.lmbda * (self.map[m - 1, d] - self.map[m, d])**2
                nrg[m, d] += self.lmbda * (self.map[m + 1, d] - self.map[m, d])**2
                force[m, d] += self.lmbda * (self.map[m - 1, d] - self.map[m, d])
                force[m, d] += self.lmbda * (self.map[m + 1, d] - self.map[m, d])
            
            #Bending NRG, Ur
            for d in range(self.n_dims):
                nrg[m, d] += self.mu * (self.map[m - 1, d] - 2 * self.map[m, d] + self.map[m + 1, d])**2
                force[m, d] += self.mu * (self.map[m - 1, d] - 2 * self.map[m, d] + self.map[m + 1, d])
        
        #find NRG for last node
        #NRG to other points Ux
        for ind in range(len(self.clustered_points[-1])):
            for d in range(self.n_dims):
                nrg[-1, d] += self.nodeWeights[ind] * np.linalg.norm(self.data[ind][d] - self.map[-1, d])**2
                force[-1, d] += self.nodeWeights[ind] * (self.data[ind][d] - self.map[-1, d])
        #if (len(self.clustered_points[-1]) > 0):
        #    nrg[-1] = nrg[-1] / sum(map(self.nodeWeights.__getitem__, self.clustered_points[-1]))
        #Stretching NRG, Ue
        for d in range(self.n_dims):
            nrg[-1, d] += self.lmbda * np.linalg.norm(self.map[-2, d] - self.map[-1, d])**2
            force[-1, d] += self.lmbda * (self.map[1, d] - self.map[-1, d])
        # no bending NRG for last node
        
        plt.figure()
        plt.plot(self.map[:, 0], self.map[:, 1], 'k', lw=3)
        
        self.map = self.map + 0.1 * (nrg / force) #m1 = m0 + d, W = Fd
        
        plt.plot(self.map[:, 0], self.map[:, 1], 'r.', lw=3)
        plt.show()
        U = self.calc_Uy() + self.calc_Ue() + self.calc_Ur()
        return U
        
    def min_fx(self, X):
        self.map = X.reshape((self.n_map, self.n_dims))
        U = self.NRG_calc()
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
                U = self.NRG_calc()
                print('Iteration: %d, Energy: %f' % (_, U))
        else:
            while (dU > termination_cond):
                self.map_expectation()
                new_U = self.NRG_calc()
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

    traj = np.hstack((np.reshape(x, (len(x), 1)), np.reshape(y, (len(y), 1))))

    trajd = downsample_traj(traj)

    elmap = ElMap(traj)
    elmap.init_map(trajd)
    elmap.init_weights()
    repro = elmap.map_EM()
    print(repro)
    plt.figure()
    plt.plot(traj[:, 0], traj[:, 1], 'k', lw=3)
    plt.plot(trajd[:, 0], trajd[:, 1], 'g', lw=3)
    plt.plot(repro[:, 0], repro[:, 1], 'r--', lw=3)
    plt.show()
    
if __name__ == '__main__':
    main_single_demo()