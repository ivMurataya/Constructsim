import numpy as np
import matplotlib.pyplot as plt

from utilities import plot_vectors

# vectors
vectors = [np.array((4, 2)), np.array((-3, 3)), np.array((-2, -2))]

try:
    plot_vectors(vectors, 'vectors')
except KeyboardInterrupt:
    # Clean shutdown on Ctrl+C
    plt.close()
