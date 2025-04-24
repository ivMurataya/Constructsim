import numpy as np
import matplotlib.pyplot as plt
from utilities import plot_vectors

v = np.array([4, 0])
u = np.array([0, 3])

lengthV = np.linalg.norm(v)
lengthU = np.linalg.norm(u)

# calculate sum
hypotenusa = np.sqrt(lengthV**2 + lengthU**2)
print("V: ", lengthV)
print("U: ", lengthU)
print("hyp: ", hypotenusa)

plot_vectors((v,u,hypotenusa),'sum')

