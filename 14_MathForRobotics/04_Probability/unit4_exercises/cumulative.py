import numpy as np
from utilities import plot_distro

# set range limits
a = 2
b = 6
# set x values
x = np.arange(a,b,0.01)
# set prob mx+n for all values
y = 0.125*(x-a)

plot_distro(x,y,'cumulative')
