import numpy as np
import matplotlib.pyplot as plt

def kf_predict(X, P, A, Q, B, U):
    X = np.dot(A, X) + np.dot(B, U)
    P = np.dot(A, np.dot(P, A.T)) + Q
    return(X,P) 
    
def kf_update(X, P, Y, H, R):
    IM = np.dot(H, X)
    IS = R + np.dot(H, np.dot(P, H.T))
    K = np.dot(P, np.dot(H.T, np.linalg.inv(IS)))
    X = X + np.dot(K, (Y-IM))
    P = P - np.dot(K, np.dot(IS, K.T))
    LH = gauss_pdf(Y, IM, IS)
    return (X,P,K,IM,IS,LH)
    
def gauss_pdf(X, M, S):
    if M.shape[1] == 1:
        DX = X - np.tile(M, X.shape[1])
        E = 0.5 * np.sum(DX * (np.dot(np.linalg.inv(S), DX)), axis=0)
        E = E + 0.5 * M.shape[0] * np.log(2 * np.pi) + 0.5 * np.log(np.linalg.det(S))
        P = np.exp(-E)
    elif X.shape[1] == 1:
        DX = np.tile(X, M.shape()[1])- M
        E = 0.5 * np.sum(DX * (np.dot(np.linalg.inv(S), DX)), axis=0)
        E = E + 0.5 * M.shape[0] * np.log(2 * np.pi) + 0.5 * np.log(np.linalg.det(S))
        P = np.exp(-E)
    else:
        DX = X-M
        E = 0.5 * dot(DX.T, dot(inv(S), DX))
        E = E + 0.5 * M.shape[0] * np.log(2 * np.pi) + 0.5 * np.log(np.linalg.det(S))
        P = np.exp(-E)
        
    return (P[0],E[0])
    
#time step of mobile movement 
dt = 0.1 

# Initialization of state matrices 
X = np.array([[0.0], [0.0], [0.0], [0.0]]) 
P = np.diag((0.01, 0.01, 0.01, 0.01)) 
A = np.array([[1, 0, dt , 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]]) 
Q = np.eye(X.shape[0]) 
B = np.eye(X.shape[0]) 
U = np.zeros((X.shape[0],1)) 

# Measurement matrices 
Y = np.array([[X[0,0] + abs(np.random.randn(1)[0])], [X[1,0] + abs(np.random.randn(1)[0])]]) 
H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]]) 
R = np.eye(Y.shape[0]) 

# Number of iterations in Kalman Filter 
N_iter = 50 

plt.figure()
kal_x = []
kal_y = []
obs_x = []
obs_y = []
pred_x = []
pred_y = []

# Applying the Kalman Filter 
for i in np.arange(0, N_iter):
    #plt.plot(X[0], X[1], 'r.')
    #plt.plot(Y[0], Y[1], 'g.')
    (X, P) = kf_predict(X, P, A, Q, B, U)
    (X, P, K, IM, IS, LH) = kf_update(X, P, Y, H, R)
    Y = np.array([[X[0,0] + abs(0.1 * np.random.randn(1)[0])],[X[1, 0] + abs(0.1 * np.random.randn(1)[0])]])
    #plt.plot(IM[0], IM[1], 'b.')
    kal_x.append(X[0])
    kal_y.append(X[1])
    obs_x.append(Y[0])
    obs_y.append(Y[1])
    pred_x.append(IM[0])
    pred_y.append(IM[1])
    
plt.plot(obs_x, obs_y, 'g.-')
plt.plot(kal_x, kal_y, 'r.-')
plt.plot(pred_x, pred_y, 'b.-')
    
plt.show()
