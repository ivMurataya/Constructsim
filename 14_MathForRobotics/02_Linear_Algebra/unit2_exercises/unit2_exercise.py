"""
a) Calculate the sum of these vectors and plot the three of them using a 
list of the vectors and the helper function plot_vectors, like in the previous examples.

b) Calculate the angles between leg1
and leg2, and between leg2 and the vector sum, call them 12 and 2sum, and convert them from radians to degrees.

"""
import numpy as np
import matplotlib.pyplot as plt
from utilities import plot_vectors


def getAngle(a,b):
    # Compute dot product
    dot = np.dot(a,b)
    # Compute norms (magnitudes)
    norm_a = np.linalg.norm(a)
    norm_b = np.linalg.norm(b)
    # Compute angle in radians
    cos_theta = dot / (norm_a * norm_b)
    angle_rad = np.arccos(cos_theta)
    # Convert to degrees (optional)
    angle_deg = np.degrees(angle_rad)
    print("Angle in radians: ",{angle_rad})
    print("Angle in degrees: ",{angle_deg})

#vectors
vectors = [np.array((4,0)), np.array((0,3))]

#Calculate Sum
sum_ = vectors[0] + vectors[1]
vectors.append(sum_)

plot_vectors(vectors,'sum')

a1 = vectors[0]
b1= vectors[1]
c1 = vectors[2]
getAngle(a=a1,b=b1)
getAngle(b1,c1)