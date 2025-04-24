import numpy as np
from utilities import plot_distro
from scipy.stats import norm

# set range limits
a = 2
b = 6
# set x values
x = np.arange(a,b,0.01)
# mu
mu = 4
# scale
sigma = 0.5
# set prob N(mu,sigma) for all values
rv = norm(mu,sigma)
y = rv.pdf(x)

plot_distro(x,y,'normal')
