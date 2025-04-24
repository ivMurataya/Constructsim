import numpy as np
from utilities import plot_tangent
from utilities import plot_derivative
from utilities import plot_function
    
    
# define the x values
x = np.arange(0,100,1)

# define the x values
f = ((0.5)*(x**2)) +3

plot_function(x,f,'f = x^2 + 3')
df = x 
plot_function(x,df,'f = x')
dff = [1 for i in range(len(x))]
plot_function(x,dff,'f = C')
# plot the function
