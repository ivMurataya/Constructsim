import numpy as np
from utilities import plot_distro

# set range limits
a = 2
b = 6
# set x values
x = np.arange(a,b,0.01)
# set value p = 1/b-a for all values
y = [((b-a)**(-1)) for i in range(len(x)-2)]

plot_distro(x,y,'uniform')
