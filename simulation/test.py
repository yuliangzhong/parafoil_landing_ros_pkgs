import numpy as np
rpy3x1 = np.array([1,2,3]).reshape(-1,1)
c = np.array([4,5,6]).reshape(-1,1)
print(np.cross(rpy3x1.reshape(-1),c.reshape(-1)).reshape(-1,1))
# print(np.random.multivariate_normal([0,0,0], [[1,0,0],[0,1,0],[0,0,0]], 1).T)