import numpy as np
from matplotlib import pyplot as plt

plt.figure(1)
d = np.linspace(-1.0,1.0,num=100)
d_2 = d**2
l = 0.5*d_2 + 0.5*np.log(1e-5 + d_2)
plt.plot(d, l)
plt.xlabel("Distance")
plt.ylabel("Cost")
plt.show()
