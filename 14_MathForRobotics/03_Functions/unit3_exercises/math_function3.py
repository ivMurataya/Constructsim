import numpy as np
from utilities import plot_function

# define the x values
x = np.arange(-6,6,1)

# function f(x)
f = x**2

# plot the function
plot_function(x,f,'f = x**2')
