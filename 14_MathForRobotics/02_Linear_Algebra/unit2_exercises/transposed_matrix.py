import numpy as np
import matplotlib.pyplot as plt

# create a matrix with numpy
A = np.array(((3,2,1),
             (6,5,4),
             (9,8,7)))
print('Matrix A: ')
print(A)

# transpose matrix with numpy
A_transposed = A.T
print('Matrix A transposed: ')
print(A_transposed)
