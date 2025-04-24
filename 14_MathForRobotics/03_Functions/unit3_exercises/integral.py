import numpy as np
from utilities import plot_integral

# define the x values
x = np.arange(0,9,1)

#define the limits
a = 1
b = 4

# define the function
def func(x):
    return 3*x
f = func(x)

# plot the area
plot_integral(x,f,func,a,b)
