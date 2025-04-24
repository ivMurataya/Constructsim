import numpy as np
from utilities import plot_equation

#--------------- Temperature ---------------
# define the x values
T_C = np.arange(0,6,1)
# function T_F(T_C)
T_F = 1.8*T_C + 32
# plot the equation
plot_equation(T_C,T_F,'T_F = 1.8*T_C + 32','Temperature in Celsius','Temperature in Fahrenheits')

#----------Sphere Radius------------------------------------
radius = np.arange(0,6,1)
# function Volume
Volume = (4/3)*np.pi*radius**3
# plot the equation
plot_equation(radius,Volume,'V = (4/3)*pi*r^3','Radius of the sphere','Volume of the sphere')

#-------------- Exponential and Logaritmical
# define the x values
t = np.arange(0,6,1)
# function population N(t)
N_0 = 1000
r = 0.75
N_t = N_0 * np.exp(r*t)
# plot the equation
plot_equation(t,N_t,'N_t = N_0*e^rt','Time t','Population infected N(t)')
plot_equation(N_t,t,'t = (1/r)*ln(N(t)/N_0)','Population infected N(t)','Time t')

#----------------- Trigonometrical
t = np.arange(-1,1,0.1)
# function Volume
x_0 = 2
w = 3
phi = 0.5
x_t = x_0 * np.sin(w*t + phi)
# plot the equation
plot_equation(t,x_t,'x(t)= x_0*sin(w*t+phi)','Time t','Position x(t)')