import numpy as np
from utilities import plot_distro

#def plot_distro(x,y,meth): uniform, cumulative, normal

x = [0  ,1   ,2   ,3   ,4   ,5  ,6   ,7   ,8   ,9   ,10]
p = [0.0,0.04,0.08,0.12,0.16,0.2,0.16,0.12,0.08,0.04,0.0]

expected_value = np.dot(x, p)
print("expected Value", expected_value)

x_squared = np.multiply(x, x)
print("x_Squared",x_squared)

expected_x_squared = np.dot(x_squared, p)
print("expectedXSquared",expected_x_squared)

Var = expected_x_squared - expected_value**2
print("Variance", Var)
std_dev = np.sqrt(Var)
print("std_Deviatrion",std_dev)

prob_safe = np.sum(p[x <= 7])
print("Probability Robot is Safe (X <= 7): ",{prob_safe})

plot_distro(x,p,'normal')