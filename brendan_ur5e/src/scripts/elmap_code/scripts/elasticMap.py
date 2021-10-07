import numpy as np
import h5py
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from scipy.interpolate import interp1d, UnivariateSpline
import cProfile


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


def get_euc_dist(p1, p2):
    return ((p1 - p2) ** 2) ** 0.5


class elastic_map(object):
    def __init__(self, given_traj, termination='iter', termination_condition=20, curveWeighting = "curvature"):
        self.traj = given_traj
        for i, node in enumerate(self.traj):
            np.append(node, i)
        (self.nodes, self.dims) = np.shape(self.traj)

        self.curveWeightings = {"uniform": self.uniformWeights, "curvature":self.weightsByCurvature, "jerk":self.weightsByJerk}
        self.curveWeightings[curveWeighting]()
        self.w = np.ones((self.nodes))
        #self.w[0] = self.w[-1] = 10

        self.term = termination
        self.term_cond = termination_condition

        self.map_nodes = downsample_traj(self.traj, 20)
        self.map_size = 20

        #self.map_nodes = np.vstack((self.traj[0], self.traj[-1]))
        #self.map_size = 2
        #self.placeByWeights(self.traj, termination_condition)

        self.lmbda = 1.
        self.mu = 1.

        self.energyEps = 0

    def placeByWeights(self, traj, num):
        weightSum = np.sum(self.nodeWeights)
        self.map_nodes = [traj[0]]
        sum = 0.
        for i in range(len(traj)):
            if sum > weightSum/(num):
                self.map_nodes.append(traj[i])
                sum -= weightSum/(num)
            sum += self.nodeWeights[i]
        #self.map_nodes.append(traj[-1])
        self.map_nodes = np.array(self.map_nodes)
        #self.map_nodes = np.array([traj[int(i)] for i in np.arange(0, len(traj), len(traj)*1.0/num)])
        self.map_size = len(self.map_nodes)

    def uniformWeights(self):
        self.nodeWeights = np.ones(len(self.traj))

    def weightsByCurvature(self):
        self.nodeWeights = [0]
        for i in range(1, len(self.traj) - 1):
            weight = np.linalg.norm(self.traj[i - 1] + self.traj[i + 1] - 2 * self.traj[i])
            self.nodeWeights.append(weight)
        self.nodeWeights.append(0)
        self.nodeWeights[0] = self.nodeWeights[1]
        self.nodeWeights[-1] = self.nodeWeights[-2]

    def weightsByJerk(self):
        self.nodeWeights = []
        for i in range(0, len(self.traj)):
            if(i < 2 or i>len(self.traj)-2):
                weight = 0
            else:
                weight = np.linalg.norm(self.traj[i - 2] + 2*self.traj[i - 1] - 2 * self.traj[i+1] - self.traj[i-1])
            self.nodeWeights.append(weight)

    #Add a new trajectory to the set and reoptimize
    def addTrajectory(self):
        pass

    def calc_grid(self, plot):
        if (plot):
            self.fig = plt.figure()
            plt.plot(self.traj[:, 0], self.traj[:, 1], 'k', lw=5)
            l, = plt.plot(self.map_nodes[:, 0], self.map_nodes[:, 1], 'r.-', lw=4, ms=20)
            self.fig.canvas.draw_idle()
            plt.pause(0.001)
            # print('Press Enter to start')
            # input()

        while not self.is_termination():
            print(self.map_nodes)
            if (plot):
                l.set_xdata(self.map_nodes[:, 0])
                l.set_ydata(self.map_nodes[:, 1])
                numPoints = len(self.map_nodes)
                if False: #numPoints > 3:
                    print(range(numPoints), self.map_nodes[:, 0])
                    #fx = UnivariateSpline(range(numPoints), self.map_nodes[:, 0])
                    #fy = UnivariateSpline(range(numPoints), self.map_nodes[:, 1])
                    fx = interp1d(range(numPoints), self.map_nodes[:, 0], kind="cubic", fill_value="extrapolate")
                    fy = interp1d(range(numPoints), self.map_nodes[:, 1], kind="cubic", fill_value="extrapolate")
                    resampled = np.array(
                        [(fx(i), fy(i)) for i in np.arange(0, len(self.map_nodes)-1, len(self.map_nodes) * 1.0 / 100)])
                    l.set_xdata(resampled[:, 0])
                    l.set_ydata(resampled[:, 1])
                self.fig.canvas.draw_idle()
                plt.pause(0.001)


            self.insert_node(plot)
            self.assign_clusters()
            self.optimize_map()

        """for i in range(5):
            self.assign_clusters()
            self.optimize_map()
        l.set_xdata(self.map_nodes[:, 0])
        l.set_ydata(self.map_nodes[:, 1])
        numPoints = len(self.map_nodes)
        if numPoints > 3:
            print(range(numPoints), self.map_nodes[:, 0])
            # fx = UnivariateSpline(range(numPoints), self.map_nodes[:, 0])
            # fy = UnivariateSpline(range(numPoints), self.map_nodes[:, 1])
            fx = interp1d(range(numPoints), self.map_nodes[:, 0], kind="cubic", fill_value="extrapolate")
            fy = interp1d(range(numPoints), self.map_nodes[:, 1], kind="cubic", fill_value="extrapolate")
            resampled = np.array(
                [(fx(i), fy(i)) for i in np.arange(0, len(self.map_nodes)-1, len(self.map_nodes) * 1.0 / 100)])
            l.set_xdata(resampled[:, 0])
            l.set_ydata(resampled[:, 1])"""
        self.assign_clusters()
        self.optimize_map(1e-2)
        print("Calculated cost at end: ", self.calc_costs(self.map_nodes))
        l.set_xdata(self.map_nodes[:, 0])
        l.set_ydata(self.map_nodes[:, 1])
        self.fig.canvas.draw_idle()
        plt.pause(0.001)
        if (plot):
            print('Finished!')
            plt.show()

        return self.map_nodes

    def assign_clusters(self):
        self.clusters = [[] for m in range(self.map_size)]
        for i in range(self.nodes):
            dists = []
            for j in range(self.map_size):
                dists.append(np.linalg.norm(self.traj[i] - self.map_nodes[j]))
            self.clusters[np.argmin(dists)].append(i)

    def calc_largest_load(self):
        # print(self.clusters)
        edge_loads = []
        for j in range(self.map_size - 1):
            cost = sum([self.nodeWeights[i] for i in self.clusters[j][:]]) + sum([self.nodeWeights[i] for i in self.clusters[j + 1][:]])
            #cost = np.linalg.norm(self.map_nodes[j - 1] + self.map_nodes[j + 1] - 2 * self.map_nodes[j])
            #cost += np.linalg.norm(self.map_nodes[j] + self.map_nodes[j + 2] - 2 * self.map_nodes[j+1])
            edge_loads.append(cost)
        print(edge_loads)
        return np.argmax(edge_loads)

    def insert_node(self, plot):
        self.assign_clusters()
        edge_num = self.calc_largest_load()
        edge = self.map_nodes[edge_num:edge_num + 2]
        print('edge')
        print(edge)
        print('')
        # if (plot):
        #    l2, = plt.plot(edge[:, 0], edge[:, 1], 'b.-', lw=3, ms=15)
        #    self.fig.canvas.draw_idle()
        #    plt.pause(0.001)
        #    l2.set_xdata([])
        #    l2.set_ydata([])
        #    self.fig.canvas.draw_idle()
        #    plt.pause(0.001)
        new_node_pos = np.mean(edge, axis=0)
        self.map_nodes = np.insert(self.map_nodes, edge_num + 1, new_node_pos, axis=0)
        self.map_size = self.map_size + 1

    def calc_Uy(self):
        cost = 0.
        for i in range(len(self.clusters)):  # i contains index of map_node
            for j in range(len(self.clusters[i])):  # j contains index of traj node of cluster. clusters[i][j] gives the index of node in traj & w
                cost = cost + self.w[self.clusters[i][j]] * np.linalg.norm(self.traj[self.clusters[i][j]] - self.map_guess[i])
        return cost / 2  # np.sum(self.w) #experimentally looks good

    def calc_Ue(self):
        cost = 0.
        for i in range(self.map_size - 1):
            cost = cost + np.linalg.norm(self.map_guess[i] - self.map_guess[i + 1])
        return cost * self.lmbda

    def calc_Ur(self):
        cost = 0.
        for i in range(1, self.map_size - 1):
            cost = cost + np.linalg.norm(self.map_guess[i - 1] + self.map_guess[i + 1] - 2 * self.map_guess[i])
        return cost * self.mu

    def calc_costs(self, X):
        self.map_guess = np.reshape(X, ((self.map_size, self.dims)))
        cost = self.calc_Ue() + self.calc_Ur() + self.calc_Uy()
        #print(cost)
        return cost

    def optimize_map(self, tol = 1.):
        init_guess = self.map_nodes
        res = minimize(self.calc_costs, init_guess, tol=tol)
        self.map_nodes = np.reshape(res.x, ((self.map_size, self.dims)))

    def is_termination(self):
        if (self.term == 'iter'):
            return self.map_size >= self.term_cond
        print('No termination found!')
        return False


def get_lasa_traj1(shape_name):
    # ask user for the file which the playback is for
    # filename = raw_input('Enter the filename of the .h5 demo: ')
    # open the file
    filename = '../h5 files/lasa_dataset.h5'
    hf = h5py.File(filename, 'r')
    # navigate to necessary data and store in numpy arrays
    shape = hf.get(shape_name)
    demo = shape.get('demo1')
    pos_info = demo.get('pos')
    pos_data = np.array(pos_info)
    y_data = np.delete(pos_data, 0, 1)
    x_data = np.delete(pos_data, 1, 1)
    # close out file
    hf.close()
    return [x_data, y_data]


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


def main_multi_demo():
    traj_full = None

    for i in range(1, 8):
        print(i)
        [x, y] = get_lasa_trajn('Saeghe', i)

        traj = np.hstack((np.reshape(x, (len(x), 1)), np.reshape(y, (len(y), 1))))

        traj = downsample_traj(traj, 20)

        traj_full = np.vstack((traj_full, traj)) if traj_full is not None else traj
        print(traj_full)

    elmap = elastic_map(traj_full)
    new_traj = elmap.calc_grid(plot=True)


def main_single_demo():
    [x, y] = get_lasa_trajn('WShape')

    traj = np.hstack((np.reshape(x, (len(x), 1)), np.reshape(y, (len(y), 1))))

    traj = downsample_traj(traj)

    elmap = elastic_map(traj)
    new_traj = elmap.calc_grid(plot=True)


if __name__ == '__main__':
    import cProfile, pstats

    profiler = cProfile.Profile()
    profiler.enable()
    main_single_demo()
    profiler.disable()
    stats = pstats.Stats(profiler).sort_stats('time')
    stats.print_stats()