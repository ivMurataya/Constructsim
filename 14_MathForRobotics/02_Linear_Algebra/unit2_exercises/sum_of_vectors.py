import numpy as np
import matplotlib.pyplot as plt
from utilities import plot_vectors

#vectors
vectors = [np.array((1,2)), np.array((2,0))]

#Calculate Sum
sum_ = vectors[0] + vectors[1]
vectors.append(sum_)

plot_vectors(vectors,'sum')

