import matplotlib.pyplot as plt
import numpy as np

fig = plt.figure()
ob = np.load("obstacles.npy")
print (ob)
plt.scatter(ob[:, 0], ob[:, 1])
plt.show()
