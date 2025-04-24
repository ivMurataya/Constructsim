import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from utilities import plot_function_3d

# define the x_1 nd x_2 values
x_1 = np.arange(0,6,1)
x_2 = np.arange(0,6,1)
x_1,x_2 = np.meshgrid(x_1, x_2)

# function f(x_1,x_2)
f = -3*x_1 + x_2**3

# plot function in 3d
plot_function_3d(x_1,x_2,f,'f = -3*x_1 + x_2^3')
