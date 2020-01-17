#!/usr/bin/env python

import rospy
import roslib
import numpy as np
from scipy.interpolate import InterpolatedUnivariateSpline
from scipy.interpolate import interp1d
from scipy import interpolate
import matplotlib.pyplot as plt

def preprocess_1d(traj, num_points, start=-1, end=-1):
  #print(traj)
  #I want to shave off the start and end where no motion is happening
  if start < 0:
    start = find_start(traj, max(np.shape(traj)))
  if end < 0:
    end = find_end(traj, max(np.shape(traj)))
  data = []
  for i in range (max(np.shape(traj))):
    if i >= start and i <= end:
      data.append(traj[0, i])
  n = max(np.shape(data))
  plot_points = np.linspace(1, n, n)
  tck = interpolate.splrep(plot_points, data, s=0)
  resample_points = np.linspace(1, n, num_points)
  resample_data = interpolate.splev(resample_points, tck, der=0)
  resample_data = np.reshape(resample_data, (1, num_points))
  return [resample_data, start, end]

def find_start(traj, n):
  i = 0
  while i < n:
    k = 1
    while k < 10 and i + k < n:
      dif = traj[0, i] - traj[0, i + k]
      if abs(dif) > 0.001:
        return i
      k = k + 1
    i = i + 10
  return 0

def find_end(traj, n):
  i = n - 1
  while i > 10:
    k = 1
    while k < 10 and i - k >= 0:
      dif = traj[0, i] - traj[0, i-k]
      if abs(dif) > 0.001:
        return i
    k = k + 1
  i = i - 10
  return n

def preprocess_nd(data, num_points, start=-1, end=-1):
  dims = min(np.shape(data))
  #get starts
  starts = np.zeros(dims)
  for i in range (dims):
    starts[i] = find_start(data[i], max(np.shape(data[i])))
  if start >= 0:
    starts.append(start)
  actual_start = min(starts)
  #get end
  ends = np.zeros(dims)
  for i in range (dims):
    ends[i] = find_end(data[i], max(np.shape(data[i])))
  if end >= 0:
    ends.append(end)
  actual_end = max(ends)
  #preprocess
  resample_data = []
  for i in range (dims):
    [data_new, s, e] = preprocess_1d(data[i], num_points, actual_start, actual_end)
    resample_data.append(data_new)
    resample_data = np.reshape(resample_data, (i, num_points))
  return [resample_data, actual_start, actual_end]

#in-file testing
def main():
  n = 300
  leg_1 = np.linspace(1, 1, n / 3)
  leg_2 = np.linspace(1, 50, n / 3)
  leg_3 = np.linspace(50, 51, n / 3)
  traj = np.array([leg_1, leg_2, leg_3]).reshape((1, n))
  [resample_data, start, end] = preprocess_1d(traj, n)
  x_vals_orig = np.linspace(0, n, n).reshape((1, n))
  x_vals_new = np.linspace(start, end, n).reshape((1, n))
  plt.plot(x_vals_orig[0], traj[0])
  plt.plot(x_vals_new[0], resample_data[0], 'r--')
  plt.show()


if __name__ == '__main__':
  main()
