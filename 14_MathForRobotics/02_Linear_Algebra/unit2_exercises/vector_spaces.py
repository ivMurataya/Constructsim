import numpy as np
import matplotlib.pyplot as plt
from utilities import plot_vectors

# basis
e_1 = np.array((1,0))
e_2 = np.array((0,1))
# vector v as a linear combination of e_1 and e_2
v = 3*e_1 + 2*e_2

vectors = (e_1,e_2,v)

plot_vectors(vectors,'vectors')
