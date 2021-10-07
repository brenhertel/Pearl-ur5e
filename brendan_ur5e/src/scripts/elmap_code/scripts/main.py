import numpy as np
from scipy.optimize import minimize
from scipy.optimize import Bounds
import matplotlib.pyplot as plt


def SSE(trj1, trj2):
    return np.sum(np.linalg.norm(trj1 - trj2, axis=1))


# metric as listed in https://en.wikipedia.org/wiki/Cosine_similarity
# used for optimization of parameters
def angular_distance(A, B):
    sim = np.dot(A.flatten(), B.flatten()) / (np.linalg.norm(A) * np.linalg.norm(B))
    # try:
    # sim = min(1.0, sim)
    if(sim > 1):
        print("this shouldn't happen, sim is: ", sim)
    return np.arccos(sim) / np.pi


def get_angular_distance(traj_1, traj_2):
    sum = 0.
    (n_pts, n_dims) = np.shape(traj_1)
    for i in range(n_pts - 1):
        sum += angular_distance(traj_1[i + 1] - traj_1[i], traj_2[i + 1] - traj_2[i])
    return sum


def get_inv_sigmoid(a, b, n):
    x = np.linspace(0, 1, n)
    return 1 - (1 / (1 + np.exp(-a * (x - b))))


def get_traj_dist(traj):
    dist = 0.
    for n in range(len(traj) - 1):
        dist = dist + (sum((traj[n + 1] - traj[n]) ** 2)) ** 0.5
    return dist


# may not need this
# may also use different function if needed
def normalize(vec, upper_bound, lower_bound, center):
    v = vec - center
    v_norm = (((upper_bound - center) / np.max(v)) * v) + center
    return v_norm


class TS_ALG(object):

    def __init__(self, init_traj):
        print('initializing object')
        self.org_traj = init_traj
        (self.n_pts, self.n_dims) = np.shape(self.org_traj)
        self.rel_traj = self.org_traj - self.org_traj[-1, :]
        self.DEBUG = True
        self.org_traj_dist = get_traj_dist(self.org_traj)

    def reproduce(self, initial=None, final=None, lmbda=0.5):
        self.l = lmbda
        self.init = initial if initial is not None else self.org_traj[0, :]
        self.fin = final if final is not None else self.org_traj[-1, :]
        self.trans_traj = self.rel_traj + self.fin
        self.init_vec = self.init - self.trans_traj[0, :]
        X_init = np.array([1])
        bnds = Bounds(1e-38, np.inf)
        res = minimize(self.get_traj_cost, X_init, bounds=bnds, tol=1e-6, options={'disp': self.DEBUG})
        # self.best_vec_w = np.reshape(get_inv_sigmoid(res.x[0], self.n_pts), (self.n_pts, 1))
        self.best_vec_w = np.reshape(1 - np.linspace(0, 1, self.n_pts) ** res.x[0], (self.n_pts, 1))
        self.best_vec = self.best_vec_w * np.reshape(self.init_vec, (1, self.n_dims))
        self.repro = self.trans_traj + self.best_vec
        return self.repro

    def get_traj_cost(self, X):
        # vec_w = np.reshape(get_inv_sigmoid(X[0], X[1], self.n_pts), (self.n_pts, 1))
        vec_w = np.reshape(1 - np.linspace(0, 1, self.n_pts) ** X[0], (self.n_pts, 1))
        vec = vec_w * np.reshape(self.init_vec, (1, self.n_dims))
        rep = self.trans_traj + vec
        cost_sse = SSE(self.org_traj, rep) / self.org_traj_dist
        cost_css = get_angular_distance(self.org_traj, rep)
        print('a: %f, sse: %f, css: %f' % (X[0], cost_sse, cost_css))
        return (self.l * cost_sse) + ((1 - self.l) * cost_css)

    def plots(self):
        fig = plt.figure()
        plt.title('Best Reproduction, lambda=' + str(self.l))
        plt.plot(self.org_traj[:, 0], self.org_traj[:, 1], 'k', label="orig")
        # plt.plot(self.rel_traj[:, 0], self.rel_traj[:, 1], 'r', label="rel")
        plt.plot(self.trans_traj[:, 0], self.trans_traj[:, 1], 'g', label="trans")
        plt.plot(self.repro[:, 0], self.repro[:, 1], 'b', label="repro")
        plt.plot(self.init[0], self.init[1], 'k.', ms=12)
        plt.plot(self.fin[0], self.fin[1], 'k.', ms=12)
        plt.legend(loc="best")
        plt.show()


if __name__ == '__main__':
    x = np.linspace(0, 10)
    y = np.sin(x)
    trj = np.transpose(np.vstack((x, y)))
    ts = TS_ALG(trj)
    ts.reproduce(initial=np.array([0, 1]), final=np.array([10, 1]), lmbda=0.5)
    ts.plots()
    ts.reproduce(initial=np.array([0, 1]), final=np.array([10, 1]), lmbda=1.0)
    ts.plots()
    ts.reproduce(initial=np.array([0, 1]), final=np.array([10, 1]), lmbda=0.0)
    ts.plots()
