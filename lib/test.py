import numpy as np

a = np.array([    [(210, 250), (210, 342), (263, 399), (430, 399), (500,305), (500, 250)]    ])
b = np.array([[1,2], [3,4], [5,6], [7,8]])
c = np.array([[ b[-index] for index in range(1,4,2)]])
d = np.flip(c, axis=1)
print(d)