import numpy as np

# create a matrix with numpy
A = np.array(((1,0,0),
             (0,2,0),
             (0,0,3)))
print('Matrix A: ')
print(A)

# eigenvalues and eigenvectors of a matrix with numpy
A_eigvalues, A_eigvectors = np.linalg.eig(A)
print('Matrix A eigenvalues: ',A_eigvalues)
print('Matrix A eigenvect: ')
print(A_eigvectors)
