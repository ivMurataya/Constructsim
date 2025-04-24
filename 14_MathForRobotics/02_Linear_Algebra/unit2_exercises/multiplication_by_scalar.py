import numpy as np
import matplotlib.pyplot as plt
from utilities import plot_vectors

# vectors
vector = np.array((1,2))

# calculate subtract
alpha = 2
multiplication_ = alpha * vector

plot_vectors([vector,multiplication_],'sum')