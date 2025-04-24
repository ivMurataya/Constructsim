import numpy as np
import matplotlib.pyplot as plt
from utilities import plot_vectors

# vectors
x = np.array(((4,1)))
y = np.array(((1,3)))


# calculate sum
hypotenusa = x + y
print(hypotenusa)

plot_vectors((x,y,hypotenusa),'sum')
