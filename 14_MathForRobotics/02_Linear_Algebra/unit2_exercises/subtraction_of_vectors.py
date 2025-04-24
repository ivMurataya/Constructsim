import numpy as np
import matplotlib.pyplot as plt
from utilities import plot_vectors

# vectors
vectors = [np.array((1,2)),np.array((2,0))]

# calculate subtract
subtract_ = vectors[0]-vectors[1]
vectors.append(subtract_)

plot_vectors(vectors,'subtract')
