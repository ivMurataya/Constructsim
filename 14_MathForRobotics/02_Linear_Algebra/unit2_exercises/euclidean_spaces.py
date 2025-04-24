import numpy as np

# vector
x = np.array((3,4))

# norm by definition
norm = np.sqrt(sum(x_i**2 for x_i in x))
print('Norm by definition: ',norm)

# norm with numpy
norm_numpy = np.linalg.norm(x)
print('Norm calculated with numpy: ',norm_numpy)
