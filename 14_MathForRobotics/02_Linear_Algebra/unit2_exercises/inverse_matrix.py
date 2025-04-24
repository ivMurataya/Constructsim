import numpy as np 
import matplotlib.pyplot as plt 

#create a matrix with numpy
A = np.array(((1,2),
                (3,4)))

B = np.array([[1, 2, 3],
              [0, 1, 4],
              [5, 6, 0]])

           
print('Matrix A: ')
print(A)

A_inverted = np.linalg.inv(A)
print('Inverted A: ')
print(A_inverted)


print('Matrix B ')
print(B)

B_inverted = np.linalg.inv(B)
print('Inverted B: ')
print(B_inverted)