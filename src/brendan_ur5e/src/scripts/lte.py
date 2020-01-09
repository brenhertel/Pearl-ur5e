#!/usr/bin/env python

#ros implementation of laplacian trajectory editing.

#Each LTE object is used to create a deformed trajectory for any 1D given trajectory

import rospy
import roslib
import numpy as np

def my_diag(A, offset=0):
  for i in range (np.size(A, 0)):
    if i + offset < np.size(A, 1) and i + offset >= 0:
      A[i][i + offset] = 1
  return A

class point(object):
  def __init__(self, given_index, given_x):
    self.index = given_index;
    self.x = given_x;
    #self.y = given_y;

class LTE(object):
  def __init__(self, given_traj, given_points=0, given_weight=1e9, given_boundary_conds=0):
    #given trajectory is dim x Nodes, want to make it nodes * dim
    self.traj = np.transpose(given_traj)
    self.trajMod = self.traj
    self.fixedPos = given_points
    if given_points == 0:
      pos = []
      pos.append(point(0, self.traj[0]))
      pos.append(point(len(self.traj) - 1, self.traj[len(self.traj) - 1]))
      self.fixedPos = pos
    self.fixedWeight = given_weight
    self.boundCond = given_boundary_conds

  def generateDelta(self):
    delta = np.zeros((np.size(self.traj, 0), 1))
    for i in range (len(delta)):
      matrix_sum = 0
      for j in range (len(self.L)):
        matrix_sum = matrix_sum + (self.L[i][j] * self.traj[j])
      delta[i] = matrix_sum
    self.delta = delta

  def generateLaplacian(self):
    nbNodes = np.size(self.traj, 0)
    L = np.zeros((nbNodes, nbNodes))
    L = my_diag(L)
    L = L * 2
    A = np.zeros((nbNodes, nbNodes))
    A = my_diag(A, -1)
    L = L - A
    B = np.zeros((nbNodes, nbNodes))
    B = my_diag(B, 1)
    L = L - B    
    L[0][1] = -2.0
    L[nbNodes - 1][nbNodes - 2] = -2.0
    L = L / 2
    self.L = L
    self.generateDelta()
    if self.boundCond == 0:
      self.L = np.delete(self.L, 0, 0)
      self.L = np.delete(self.L, np.size(self.L, 0) - 1, 0)
      self.delta = np.delete(self.delta, 0, 0)
      self.delta = np.delete(self.delta, np.size(self.delta, 0) - 1, 0)
    print(self.L)
    print(self.delta)
    for i in range (np.size(self.fixedPos)):
      to_append_L = np.zeros(nbNodes)
      to_append_L[self.fixedPos[i].index] = self.fixedWeight
      self.L = np.vstack((self.L, to_append_L))
      to_append_delta = np.zeros(1)
      to_append_delta = self.fixedWeight * self.fixedPos[i].x
      self.delta = np.vstack((self.delta, to_append_delta))
  
  def generateTraj(self):
    new_traj = np.linalg.solve(self.L, self.delta)
    return new_traj

def main():
  hLTE = LTE(np.ones((1, 5)))
  hLTE.generateLaplacian()
  new_traj = hLTE.generateTraj()
  #print(new_traj)

if __name__ == '__main__':
  main()
