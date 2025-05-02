#!/usr/bin/env python3

import math
from sympy import *

# Define symbolic parameters
D1, D2, D3 = symbols("B B2 B3")
theta_1, theta_2, theta_3 = symbols("theta_1 theta_2 theta_3")
A1, A2, A3 = symbols("r1 r2 r3")
alpha_1, alpha_2, alpha_3 = symbols("alpha_1 alpha_2 alpha_3")

# Transformation from frame 0 to frame 1
A_00_ = Matrix([
    [cos(theta_1), -sin(theta_1), 0, 0],
    [sin(theta_1),  cos(theta_1), 0, 0],
    [0,             0,             1, D1],
    [0,             0,             0, 1]
])

A_0_1 = Matrix([
    [1, 0,              0,           A1],
    [0, cos(alpha_1), -sin(alpha_1), 0],
    [0, sin(alpha_1),  cos(alpha_1), 0],
    [0, 0,              0,           1]
])

A01 = A_00_ * A_0_1
preview(A01, viewer='file', filename="A01.png", dvioptions=['-D','300'])
A01_G = A01.subs({alpha_1: 0.0, D1: 0.0})
preview(A01_G, viewer='file', filename="A01_subs.png", dvioptions=['-D','300'])

# Transformation from frame 1 to frame 2
A_11_ = Matrix([
    [cos(theta_2), -sin(theta_2), 0, 0],
    [sin(theta_2),  cos(theta_2), 0, 0],
    [0,             0,             1, D2],
    [0,             0,             0, 1]
])

A_1_2 = Matrix([
    [1, 0,              0,           A2],
    [0, cos(alpha_2), -sin(alpha_2), 0],
    [0, sin(alpha_2),  cos(alpha_2), 0],
    [0, 0,              0,           1]
])

A12 = A_11_ * A_1_2
preview(A12, viewer='file', filename="A12.png", dvioptions=['-D','300'])
A12_G = A12.subs({alpha_2: 0.0, D2: 0.0})
preview(A12_G, viewer='file', filename="A12_subs.png", dvioptions=['-D','300'])

# Transformation from frame 2 to frame 3
A_22_ = Matrix([
    [cos(theta_3), -sin(theta_3), 0, 0],
    [sin(theta_3),  cos(theta_3), 0, 0],
    [0,             0,             1, D3],
    [0,             0,             0, 1]
])

A_2_3 = Matrix([
    [1, 0,              0,           A3],
    [0, cos(alpha_3), -sin(alpha_3), 0],
    [0, sin(alpha_3),  cos(alpha_3), 0],
    [0, 0,              0,           1]
])

A23 = A_22_ * A_2_3
preview(A23, viewer='file', filename="A23.png", dvioptions=['-D','300'])
A23_G = A23.subs({alpha_3: 0.0, D3: 0.0})
preview(A23_G, viewer='file', filename="A23_subs.png", dvioptions=['-D','300'])

# Compute full transformation A_0123
A0123 = simplify(A01 * A12 * A23)
preview(A0123, viewer='file', filename="A0123.png", dvioptions=['-D','300'])

# Simplified substitution result
A0123_G = A0123.subs({
    alpha_1: 0.0, alpha_2: 0.0, alpha_3: 0.0,
    D1: 0.0, D2: 0.0, D3: 0.0
})
preview(A0123_G, viewer='file', filename="A0123_subs.png", dvioptions=['-D','300'])

# Trig simplified
A03_simplify = trigsimp(A0123)
preview(A03_simplify, viewer='file', filename="A0123_simp.png", dvioptions=['-D','300'])
