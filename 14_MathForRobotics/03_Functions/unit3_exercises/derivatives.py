import numpy as np
from utilities import plot_tangent
from utilities import plot_derivative
    
    
# define the x values
x = np.arange(0.0,11.0,1)

# function f(x)
f = np.log(x+1)
t1 = x/3 + 0.43
t2 = x/10 + 1.42

# plot the function
plot_tangent(x,f,t1,t2,'f=ln(1+x)','tangent at x=2','tangent at x=9')


# define the x values
x = np.arange(0,6,1)
# function f(x)
f = 3*x
df = [3 for i in range(len(x))]
# plot the function
plot_derivative(x,f,df,'f = 3*x','df = 3')

# define the x values
x = np.arange(0,6,1)
# function f(x)
f = x**2
df = 2*x
# plot the function
plot_derivative(x,f,df,'f = x^2','df = 2*x')