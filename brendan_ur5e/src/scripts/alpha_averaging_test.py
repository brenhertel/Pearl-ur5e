import numpy as np
import matplotlib.pyplot as plt

x = np.linspace(0, 10, 100)
y = np.zeros(np.shape(x))
y[50:] = 1

ynew = np.zeros(np.shape(x))

for i in range(1, len(ynew)):
    ynew[i] = (0.9 * ynew[i-1]) + (0.1 * y[i])
    
plt.figure()
plt.plot(x, y, 'r')
plt.plot(x, ynew, 'g')
plt.show()
