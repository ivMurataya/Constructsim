import numpy as np
import matplotlib.pyplot as plt
from utilities import plot_vectors

v = np.array([3, 4])
u = np.array([1, .5])


lengthV = np.linalg.norm(v)
print("Length of the vector:", lengthV)

proj = (np.dot(v, u) / np.dot(u, u)) * u
lengthP = np.linalg.norm(proj)
print("Length of the vector:", lengthP)
print("Projection:", proj)  # Output: [3. 0.]
plot_vectors((v,u,proj),'sum')