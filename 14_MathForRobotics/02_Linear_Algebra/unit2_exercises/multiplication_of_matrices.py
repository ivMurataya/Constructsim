import numpy as np
import matplotlib.pyplot as plt

# create matrices with numpy
A = np.array(((1,2,3),
              (4,5,6)))

B = np.array(((1,1,1)))

print('Matrix A: ')
print(A)
print('Matrix B: ')
print(B)

# sum of matrices with numpy
sum_ = A+B
print('Sum of matrices A + B: ')
print(sum_)

# Multiplication for a scalar
multiply_ = A*3
print('Multiplication of matrix A * 3: ')
print(multiply_)

# Product of matrices with numpy
product_ = np.dot(A,B)
print('Product of matrices A * B = C: ')
print('C = ',product_)
