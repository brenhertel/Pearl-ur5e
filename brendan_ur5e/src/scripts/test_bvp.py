#!/usr/bin/env python

import numpy as np
from scipy.integrate import solve_bvp
import matplotlib.pyplot as plt


def fun(x, y):
    print('x')
    print(np.shape(x))
    print(x)
    print('y')
    print(np.shape(y))
    t1 = np.exp(-19846/y[1])*(1 - y[0])
    print('t1')
    print(np.shape(t1))
    dy21 = y[2] - y[1]
    res = np.vstack((3.769911184e12*t1,
                      0.2056315191*dy21 + 6.511664773e14*t1,
                      1.696460033*dy21))
    print('res')
    print(np.shape(res))
    print(res)
    return res

def bc(ya, yb):
    return np.array([ya[0], ya[1] - 673, yb[2] - 200])


n = 25
x = np.linspace(0, 1, n)
y = np.array([x, np.full_like(x, 673), np.linspace(800, 200, n)])

print(np.shape(x))
print(np.shape(y))

sol = solve_bvp(fun, bc, x, y)

if sol.status != 0:
    print("WARNING: sol.status is %d" % sol.status)
print(sol.message)

#plt.plot(x, y)
plt.plot(sol.x, sol.y[0])
plt.plot(sol.x, sol.y[1])
plt.plot(sol.x, sol.y[2])
#plt.subplot(2, 1, 1)
#plt.plot(sol.x, sol.y[0], color='#801010', label='$y_0(x)$')
#plt.grid(alpha=0.5)
#plt.legend(framealpha=1, shadow=True)
#plt.subplot(2, 1, 2)
#plt.plot(sol.x, sol.y[1], '-', color='c0', label='$y_1(x)$')
#plt.plot(sol.x, sol.y[2], '--', color='c0', label='$y_2(x)$')
#plt.xlabel('$x$')
#plt.grid(alpha=0.5)
#plt.legend(framealpha=1, shadow=True)
plt.show()
