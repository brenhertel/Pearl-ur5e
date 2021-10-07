import numpy as np
from scipy.stats import norm as normal
from scipy.stats import multivariate_normal as multi_normal

def kMeans(X, K, maxIters = 30):
    centroids = X[np.random.choice(np.arange(len(X)), K), :]
    for i in range(maxIters):
        C = np.array([np.argmin([np.dot(x_i-y_k, x_i-y_k) for y_k in centroids]) for x_i in X])
        centroids = [X[C == k].mean(axis = 0) for k in range(K)]
    return np.array(centroids) , C

def EM_init(Data, nbStates):
    nbVar, nbData = np.shape(Data)
    Priors = np.ndarray(shape = (1, nbStates))
    Sigma = np.ndarray(shape = (nbVar, nbVar, nbStates))
    Centers, Data_id = kMeans(np.transpose(Data), nbStates)
    Mu = np.transpose(Centers)
    for i in range (0,nbStates):
        idtmp = np.nonzero(Data_id==i)
        idtmp = list(idtmp)
        idtmp = np.reshape(idtmp,(np.size(idtmp)))
        Priors[0,i] = np.size(idtmp)
        a = np.concatenate((Data[:, idtmp],Data[:, idtmp]), axis = 1)
        Sigma[:,:,i] = np.cov(a)
        Sigma[:,:,i] = Sigma[:,:,i] + 0.00001 * np.diag(np.diag(np.ones((nbVar,nbVar))))
    Priors = Priors / nbData
    return (Priors, Mu, Sigma)

def new_kmeans(data, n_centers):
    (n_pts, n_dims) = np.shape(data)
    centers = data[np.random.choice(np.arange(n_pts), n_centers), :]
    belongs = np.zeros((n_pts, n_centers))
    old_belongs = belongs.copy()
    for p in range(n_pts):
        dists = []
        for k in range(n_centers):
            dists.append(np.linalg.norm(data[p] - centers[k]))
        belongs[p, np.argmin(dists)] = 1
    iter = 0
    while iter < 30  and not np.array_equal(belongs, old_belongs):
        for c in range(n_centers):
            if (np.sum(belongs[:, c]) > 0):
                weighted_sums = np.multiply(np.reshape(belongs[:, c], ((n_pts, 1))),  data)
                centers[c, :] = np.sum(weighted_sums, axis=0) / np.sum(belongs[:, c])
            else:
                centers[c, :] = data[np.random.choice(np.arange(n_pts), 1), :]
        belongs = np.zeros((n_pts, n_centers))
        old_belongs = belongs.copy()
        for p in range(n_pts):
            dists = []
            for k in range(n_centers):
                dists.append(np.linalg.norm(data[p] - centers[k]))
            belongs[p, np.argmin(dists)] = 1
        iter += 1
    return centers
    

class my_kmeans(object):

    def __init__(self, num_centers=4, eps=0.0001):
        self.n_centers = num_centers
        self.centers = []
        self.epsilon =  eps # the percision we want to reach
        
    ##General Implementation of k-means EM algorithm
    def k_means(self, data, w=None):
        iters = 0       # counter
        shift = 10000.0   # initial error (a big number)
        (self.n_pts, self.n_dims) = np.shape(data)
        self.D = data
        self.weights = np.ones((self.n_pts)) if w is None else w
        #guess initial centers randomly
        for c in range(self.n_centers):
            self.centers.append(data[np.random.randint(0, self.n_pts - 1)])
        self.centers = np.array(self.centers).reshape((self.n_centers, self.n_dims))
        while shift > self.epsilon and iters < 200:
            iters += 1
            self.belongs = np.zeros((self.n_pts, self.n_centers))
        
            # Expectation
            self.expectation()
            
            old_means = self.centers.copy()
            
            # Minimization
            self.maximization()
        
            # calculate the error
            shift = np.sum(np.linalg.norm(self.centers - old_means, axis=1))
        
            print("iteration {}, shift{}".format(iters,shift))
        Priors = np.ndarray(shape = (1, self.n_centers))
        Sigma = np.ndarray(shape = (self.n_dims, self.n_dims, self.n_centers))
        print(self.belongs)
        for c in range(self.n_centers):
            a = []
            for n in range(self.n_pts):
                if (self.belongs[n, c]):
                    a.append(self.D[n, :])
            num_a = len(a)
            a = np.vstack(a)
            print(self.n_dims)
            print(a)
            print(np.cov(np.transpose(a)))
            print(np.shape(Sigma[:,:,c]))
            print(np.cov(np.transpose(a)))
            Sigma[:,:,c] = np.cov(np.transpose(a))
            Sigma[:,:,c] = Sigma[:,:,c] + 0.00001 * np.diag(np.diag(np.ones((self.n_dims,self.n_dims))))
            Priors[0, c] = num_a / self.n_pts
        return (Priors, np.transpose(self.centers), Sigma)
            
    def expectation(self):
        for p in range(self.n_pts):
            dists = []
            for k in range(self.n_centers):
                dists.append(np.linalg.norm(self.D[p] - self.centers[k]))
            self.belongs[p, np.argmin(dists)] = 1
        return
    
    def maximization(self):
        for c in range(self.n_centers):
            weighted_sums = np.multiply(np.reshape(self.belongs[:, c], ((self.n_pts, 1))), np.multiply(np.reshape(self.weights, ((self.n_pts, 1))), self.D))
            self.centers[c, :] = np.sum(weighted_sums, axis=0) / np.sum(self.belongs[:, c])
        return 

## Implementation of weighted version as it appears in "Tactile Guidance for Policy Refinement and Reuse" by Brenna D. Argall, Eric L. Sauser and Aude G. Billard
def weighted_k_means(data, k=4, w=None, max_iters=150):
    (n_pts, n_dims) = np.shape(data)
    centers = np.zeros((k, n_dims))
    D = data
    weights = np.ones((n_pts)) if w is None else w
    sum_weights = np.sum(weights)
    #guess initial centers randomly
    for c in range(k):
       centers[c, :] = data[np.random.randint(0, n_pts - 1)]
    covar = np.cov(np.transpose(D))
    covariances = [covar for c in range(k)]
    
    P = np.random.rand(n_pts, k)
    E = np.ones((k))
    gamma = np.ones((k))
    iters = 0
    while iters < max_iters:
        old_E = np.sum(E)
        
        # Expectation
        P, E = weighted_expectation(D, weights, centers, covariances, gamma)
        
        # Maximization
        gamma, centers, covariances = weighted_maximization(D, weights, sum_weights, E, P, covariances)
    
        # calculate the error
        new_E = np.sum(E)
        shift = old_E - new_E
    
        print("iteration {}, shift{}".format(iters,shift))
        iters += 1
    Priors = np.sum(P, axis=0) / np.sum(P)
    
    out_cov = np.zeros((n_dims, n_dims, k))
    for i in range(k):
        out_cov[:, :, i] = covariances[i]
    
    return (Priors, np.transpose(centers), out_cov)

def weighted_expectation(D, weights, centers, covariances, gamma):
    (n_pts, n_dims) = np.shape(D)
    (n_centers, _) = np.shape(centers)
    P = np.zeros((n_pts, n_centers))
    E = np.zeros((n_centers))
    for j in range(n_pts):
        numers = []
        for k in range(n_centers):
            if n_dims < 2:
                numers.append(gamma[k] * normal.pdf(D[j], centers[k], covariances[k]))  
            else:
                numers.append(gamma[k] * multi_normal.pdf(D[j], centers[k], covariances[k], allow_singular=True))
        denom = sum(numers) + 1e-6
        for k in range(n_centers):
            P[j, k] = numers[k] / denom
    for k in range(n_centers):
        E[k] = np.sum(np.multiply(weights, P[:, k])) + 1e-9
    return P, E

def weighted_maximization(D, weights, sum_weights, E, P, covariances):
    n_centers = np.size(E)
    (n_pts, n_dims) = np.shape(D)
    gamma = np.zeros((n_centers))
    centers = np.zeros((n_centers, n_dims))
    for k in range(n_centers):
        gamma[k] = E[k] / sum_weights
        centers[k] = np.sum(np.multiply(np.reshape(weights, ((n_pts, 1))), np.multiply(np.reshape(P[:, k], ((n_pts, 1))), D)), axis=0) / E[k]
        sum_cov = np.zeros((n_dims, n_dims))
        for j in range(n_pts):
            diff = np.reshape(D[j] - centers[k], ((1, n_dims)))
            sum_cov = sum_cov + weights[j] * P[j, k] * np.matmul(np.transpose(diff), diff)
        covariances[k] = sum_cov / E[k]
    return gamma, centers, covariances