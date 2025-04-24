import numpy as np
from utilities import plot_function

# define the x values
T_C = np.arange(0,6,1)

# function T_F(T_C)
T_F = 1.8*T_C + 32

# plot the function
plot_function(T_C,T_F,'T_F = 1.8*T_C + 32')

# first derivative
dT_F = [1.8 for i in range(len(T_C))]

# plot first derivative
plot_function(T_C,dT_F,'dT_F = 1.8')

# second derivative
d2T_F = [0 for i in range(len(T_C))]

# plot second derivative
plot_function(T_C,d2T_F,'d2T_F = 0')


import numpy as np
from utilities import plot_function

# define the x values
radius = np.arange(0,6,1)

# function Volume
Volume = (4/3)*np.pi*radius**3

# plot the function
plot_function(radius,Volume,'V = (4/3)*pi*r^3')

# first derivative
dVolume = 4*np.pi*radius**2

# plot first derivative
plot_function(radius,dVolume,'dV = 4*pi*r^2')

# second derivative
d2Volume = 8*np.pi*radius

# plot second derivative
plot_function(radius,d2Volume,'d2V = 8*pi*r')


import numpy as np
from utilities import plot_function

# define the x values
t = np.arange(0,6,1)

# function population N(t)
N_0 = 1000
r = 0.5
N_t = N_0 * np.exp(r*t)

# plot the equation
plot_function(t,N_t,'N_t = N_0*e^rt')

# first derivative
dN_t = N_0*r*np.exp(r*t)

# plot first derivative
plot_function(t,dN_t,'dN_t = N_0*r*e^rt')

# second derivative
d2N_t = N_0*(r**2)*np.exp(r*t)

# plot second derivative
plot_function(t,d2N_t,'d2N_t = N_0*r^2*e^rt')


import numpy as np
from utilities import plot_function

# define the x values
t = np.arange(0,6,1)

# function population N(t)
N_0 = 1000
r = 0.5
t = (1/r)*np.log(N_t/N_0)

# plot the equation
plot_function(N_t,t,'t = (1/r)*np.log(N_t/N_0)')

# first derivative
dt = 1/(r*N_t)

# plot first derivative
plot_function(N_t,dt,'dt = 1/r*N_t')

# second derivative
d2t = -1/(r*N_t**2)

# plot second derivative
plot_function(N_t,d2t,'d2t = -1/r*N_t**2')


import numpy as np
from utilities import plot_function

# define the x values
t = np.arange(-1, 1, 0.1)

# function Volume
x_0 = 2
w = 3
phi = 0.5
x_t = x_0 * np.sin(w*t + phi)

# plot the function
plot_function(t, x_t, 'x(t)= x_0*sin(w*t+phi)')

# first derivative
dx_t = x_0*w*np.cos(w*t + phi)

# plot the first derivative
plot_function(t, dx_t, 'dx(t) = x_0*w*cos(w*t+phi)')

# second derivative
d2x_t = -x_0*(w**2)*np.sin(w*t + phi)

# plot the second derivative
plot_function(t, d2x_t, 'd2x(t) = -x_0*w^2*sin(w*t + phi)')
