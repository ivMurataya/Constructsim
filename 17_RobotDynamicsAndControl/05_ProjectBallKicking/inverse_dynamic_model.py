from __future__ import print_function, division
from sympy import symbols, simplify, trigsimp
from sympy.physics.mechanics import (dynamicsymbols, ReferenceFrame, Point,
                                     inertia, RigidBody, KanesMethod)
from sympy.physics.vector import init_vprinting, vlatex
init_vprinting(use_latex='mathjax', pretty_print=False)

inertial_frame = ReferenceFrame('I')
lower_link_frame = ReferenceFrame('L')
upper_link_frame = ReferenceFrame('U')

theta_1, theta_2 = dynamicsymbols('theta_1 theta_2')

tau_1, tau_2 = dynamicsymbols('tau_1 tau_2')
l_1, l_2 = symbols('l_1 l_2', positive=True)
r_1, r_2 = symbols('r_1 r_2', positive=True)
m_1, m_2, b, g = symbols('m_1 m_2 b g')
I_1_yy, I_2_yy = symbols('I_{1yy}, I_{2yy}')

lower_link_frame.orient(inertial_frame, 'Axis', [theta_1, inertial_frame.y])
upper_link_frame.orient(lower_link_frame, 'Axis', [
                        theta_2, lower_link_frame.y])

shoulder = Point('S')
shoulder.set_vel(inertial_frame, 0)

elbow = Point('E')
elbow.set_pos(shoulder, l_1 * lower_link_frame.z)
elbow.v2pt_theory(shoulder, inertial_frame, lower_link_frame)

CM_1 = Point('CM_1')
CM_1.set_pos(shoulder, r_1 * lower_link_frame.z)
CM_1.v2pt_theory(shoulder, inertial_frame, lower_link_frame)

CM_2 = Point('CM_2')
CM_2.set_pos(elbow, r_2 * upper_link_frame.z)
CM_2.v2pt_theory(elbow, inertial_frame, upper_link_frame)

I_1 = inertia(lower_link_frame, 0, I_1_yy, 0)
BODY_1 = RigidBody('lower_link', CM_1, lower_link_frame, m_1, (I_1, CM_1))

I_2 = inertia(upper_link_frame, 0, I_2_yy, 0)
BODY_2 = RigidBody('upper_link', CM_2, upper_link_frame, m_2, (I_2, CM_2))

P_1 = m_1 * g * inertial_frame.z
r_1_CM = CM_1.pos_from(shoulder).express(inertial_frame)
BODY_1.potential_energy = r_1_CM.dot(P_1)

P_2 = m_2 * g * inertial_frame.z
r_2_CM = CM_2.pos_from(shoulder).express(inertial_frame).simplify()
BODY_2.potential_energy = r_2_CM.dot(P_2)

omega_1, omega_2 = dynamicsymbols('omega_1, omega_2')

kinematical_differential_equations = [omega_1 - theta_1.diff(),
                                      omega_2 - theta_2.diff()]

coordinates = [theta_1, theta_2]
speeds = [omega_1, omega_2]
specified = [tau_1, tau_2]

kane = KanesMethod(inertial_frame, coordinates, speeds,
                   kinematical_differential_equations)

lower_link_frame.set_ang_vel(inertial_frame, omega_1*inertial_frame.y)
upper_link_frame.set_ang_vel(lower_link_frame, omega_2*inertial_frame.y)

lower_link_grav_force_vector = -m_1 * g * inertial_frame.z
lower_link_grav_force = (CM_1, lower_link_grav_force_vector)
upper_link_grav_force_vector = -m_2 * g * inertial_frame.z
upper_link_grav_force = (CM_2, upper_link_grav_force_vector)

lower_link_torque_vector = (tau_1 - b * omega_1) * \
    inertial_frame.y - (tau_2 - b * omega_2) * inertial_frame.y
lower_link_torque = (lower_link_frame, lower_link_torque_vector)
upper_link_torque_vector = (tau_2 - b * omega_2) * inertial_frame.y
upper_link_torque = (upper_link_frame, upper_link_torque_vector)

loads = [lower_link_grav_force,
         upper_link_grav_force,
         lower_link_torque,
         upper_link_torque]

bodies = [BODY_1, BODY_2]

fr, frstar = kane.kanes_equations(bodies, loads)

mm = trigsimp(kane.mass_matrix)

fc = trigsimp(kane.forcing)

print(mm)
print(fc)

torque_1_expr = mm[0,0]*omega_1 + mm[0,1]*omega_2 - fc[0] + tau_1
torque_2_expr = mm[1,0]*omega_1 + mm[1,1]*omega_2 - fc[1] + tau_2


from numpy import array

constants = [l_1, r_1, m_1, I_1_yy, l_2, r_2, m_2, I_2_yy, b, g]

numerical_constants = array([1.000,  # lower_link_length [m]
                             0.450,  # lower_link_com_length [m]
                             1.000,  # lower_link_mass [kg]
                             0.084,  # lower_link_inertia [kg*m^2]
                             1.000,  # upper_link_length [m]
                             0.450,  # upper_link_com_length
                             1.000,  # upper_link_mass [kg]
                             0.084,  # upper_link_inertia [kg*m^2]
                             0.700,  # joint damping [N s / m]
                             9.81],  # acceleration due to gravity [m/s^2]
                            ) 

parameter_dict = dict(zip(constants, numerical_constants))

torque_1_expr = torque_1_expr.subs(parameter_dict)
torque_2_expr = torque_2_expr.subs(parameter_dict)


from sympy.utilities.lambdify import lambdify

torque_1_function = lambdify([theta_1, theta_2, omega_1, omega_2], torque_1_expr, "numpy")
torque_2_function = lambdify([theta_1, theta_2, omega_1, omega_2], torque_2_expr, "numpy")


import numpy as np

def theta_interp(theta_s, theta_g, t_g, control_freq=50):
    num_iter = control_freq * t_g + 1
    u = np.linspace(0, 1, num_iter)
    t = u * t_g
    theta = theta_s + (theta_g-theta_s)*(3*u*u-2*u*u*u)
    dtheta = (theta_g-theta_s)*(6*u-6*u*u)/t_g
    return t, theta, dtheta

t, th1, w1 = theta_interp(-np.pi,-4*np.pi/5,3)
t, th2, w2 = theta_interp(0.0, np.pi/5,3)

import matplotlib.pyplot as plt

plt.subplot('121')
plt.plot(t, th1, t, w1)
plt.legend(('th_1', 'w_1'))
plt.subplot('122')
plt.plot(t, th2, t, w2)
plt.legend(('th_2', 'w_2'))
plt.show()

torque_1 = torque_1_function(th1, th2, w1, w2)
torque_2 = torque_2_function(th1, th2, w1, w2)

torque = np.stack([torque_1, torque_2], axis=-1)

np.save('torque.npy', torque)

plt.plot(t, torque_1, t, torque_2)
plt.legend(('tau_1', 'tau_2'))
plt.show()

