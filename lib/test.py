import numpy as np

a = np.zeros((10,2))
a = np.delete(a, 0, axis=0)
a = np.append(a, [[0.3, 1]], axis=0)
[b, c] = [1 , 1]
[b, c] = [b,c] + a[-1]
print(b)