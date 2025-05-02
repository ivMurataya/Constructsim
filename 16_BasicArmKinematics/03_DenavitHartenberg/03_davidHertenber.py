#!/usr/bin/env python3

import math
from sympy import   *

#theta =  Joint Angle
#D = Link Ofset
#A = Link Lengt
#Alpha = Link Twist


D1 = Symbol("B") # 0.1 m distance UP
theta_1 = Symbol("theta_1") #0.0 angle of difference on the end
A1 = Symbol("r1") # 1.0m  distance of the join
alpha_1 = Symbol("alpha_1") # 0.0 angle differenced of rotation in Z axis

# A_00′: rotation about Z by theta, translation along Z by d
A_00_ = Matrix([
    [cos(theta_1), -sin(theta_1), 0, 0],
    [sin(theta_1),  cos(theta_1), 0, 0],
    [0,           0,          1, D1],
    [0,           0,          0, 1]
])

# A_0′1: translation along X by a, rotation about X by alpha
A_0_1 = Matrix([
    [1, 0,           0,          A1],
    [0, cos(alpha_1), -sin(alpha_1), 0],
    [0, sin(alpha_1),  cos(alpha_1), 0],
    [0, 0,           0,          1]
])

A01 = A_00_*A_0_1

alha_value = 0.0
D1_value = 0.0
preview(A01, viewer='file', filename="A01.png", dvioptions=['-D','300'])
A01_G = A01.subs(alpha_1,alha_value).subs(D1,D1_value)
preview(A01_G, viewer='file', filename="A01_subs.png", dvioptions=['-D','300'])


#--------------------------------------------------------
D2 = Symbol("B2") # 0.1 m distance UP
theta_2 = Symbol("theta_2") #0.0 angle of difference on the end
A2 = Symbol("r2") # 1.0m  distance of the join
alpha_2 = Symbol("alpha_2") # 0.0 angle differenced of rotation in Z axis

# A_00′: rotation about Z by theta, translation along Z by d
A_11_ = Matrix([
    [cos(theta_2), -sin(theta_2), 0, 0],
    [sin(theta_2),  cos(theta_2), 0, 0],
    [0,           0,          1, D2],
    [0,           0,          0, 1]
])

# A_0′1: translation along X by a, rotation about X by alpha
A_1_2 = Matrix([
    [1, 0,           0,          A2],
    [0, cos(alpha_2), -sin(alpha_2), 0],
    [0, sin(alpha_2),  cos(alpha_2), 0],
    [0, 0,           0,          1]
])

A12 = A_11_*A_1_2

alha2_value = 0.0
D2_value = 0.0
preview(A12, viewer='file', filename="A12.png", dvioptions=['-D','300'])
A12_G = A12.subs(alpha_2,alha2_value).subs(D2,D2_value)
preview(A12_G, viewer='file', filename="A12_subs.png", dvioptions=['-D','300'])

#--------------------------------------------------------
D3 = Symbol("B3") # 0.1 m distance UP
theta_3 = Symbol("theta_3") #0.0 angle of difference on the end
A3 = Symbol("r3") # 1.0m  distance of the join
alpha_3 = Symbol("alpha_3") # 0.0 angle differenced of rotation in Z axis

# A_00′: rotation about Z by theta, translation along Z by d
A_22_ = Matrix([
    [cos(theta_3), -sin(theta_3), 0, 0],
    [sin(theta_3),  cos(theta_3), 0, 0],
    [0,           0,          1, D3],
    [0,           0,          0, 1]
])

# A_0′1: translation along X by a, rotation about X by alpha
A_2_3 = Matrix([
    [1, 0,           0,          A3],
    [0, cos(alpha_3), -sin(alpha_3), 0],
    [0, sin(alpha_3),  cos(alpha_3), 0],
    [0, 0,           0,          1]
])

A23 = A_22_*A_2_3

alha3_value = 0.0
D3_value = 0.0
preview(A23, viewer='file', filename="A23.png", dvioptions=['-D','300'])
A23_G = A23.subs(alpha_3,alha3_value).subs(D3,D3_value)
preview(A23_G, viewer='file', filename="A23_subs.png", dvioptions=['-D','300'])


#----------------------------------------------------------------------------

# this Computation WAS NOT DISPLAYED AS IN THE ANSWERS
# Compute full transformation A_0123
A0123 = simplify(A01 * A12 * A23)


# Save image of symbolic matrix
preview(A0123, viewer='file', filename="0A0123.png", dvioptions=['-D','300'])

# If you want to substitute values (like alphas and Ds = 0)
A0123_G = A0123.subs({
    alpha_1: 0.0, alpha_2: 0.0, alpha_3: 0.0,
    D1: 0.0, D2: 0.0, D3: 0.0
})
preview(A0123_G, viewer='file', filename="0A0123_subs.png", dvioptions=['-D','300'])


A03_simplify = trigsimp(A0123)
preview(A03_simplify, viewer='file', filename="0A0123_simp.png", dvioptions=['-D','300'])

"""
Three LINKS with THREE REVOLUTE JOINTS.

Place FOUR FRAMES using the DENAVIT HARTENBERG method.
ALL frames remember that they have to have their Z-axis, where the joints have their axis of revolution.
FRAME 0: Has to be fixed to the ground.
FRAMES 1-2-3: Have to be fixed to each of the three links.
FRAME 3: Is the frame of the gripper or end effector, and therefore, has to be on the tip of the last link (Link 3).

Get the DENAVIT HARTENBERG PARAMETERS of each of the links. The lengths of each link must be data for the exercise.
r1,r2,r3 = 1m


"""
