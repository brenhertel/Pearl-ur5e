#!/usr/bin/env python


import numpy as np
from scipy import interpolate
import math

class endpoint(object):
  def __init__(self, position, velocity=0, acceleration=0):
    self.x = position
    self.v = velocity
    self.a = acceleration

class JA(object):
  def __init__(self, given_traj, given_points=0, given_lambda=-1, given_time_data=0, direction=1, method='fast'):
    self.traj = np.transpose(given_traj)
    self.nodes = np.shape(self.traj)[0]
    self.dims = np.shape(self.traj)[1]
    self.endpoints = given_points
    if self.endpoints == 0:
      self.endpoints = []
      for i in range (self.dims):
        endpnt1 = endpoint(self.traj[0, i])
        endpnt2 = endpoint(self.traj[self.nodes - 1, i])
        self.endpoints.append(endpnt1)
        self.endpoints.append(endpnt2)
    self.endpoints = np.reshape(self.endpoints, (2, self.dims))
    self.l = given_lambda
    if self.l <= 0:
      self.l = math.ceil(np.size(self.traj) / 20)
    self.l = self.l * (self.nodes / 250) * (self.dims**2 / 4)
    self.tt = given_time_data
    if self.tt == 0:
      self.tt = np.linspace(0, 1, self.nodes)
    print('traj')
    print(self.traj)
    print('endpoints')
    print(self.endpoints)
    print('lambda')
    print(self.l)
    print('time')
    print(self.tt)
    
  def generateTraj(self):
    for di in range (self.dims):
      F = interpolate.interp1d(self.tt, self.traj[:, di], 'cubic')
      rx0 = endpoints[0, di]
      rx1 = endpoints[1, di]


#in-file testing
def main():
  hJA = JA(np.ones((1, 5)))
  #hJA.generateLaplacian()
  #new_traj = hLTE.generateTraj()
  #print(new_traj)

if __name__ == '__main__':
  main()
