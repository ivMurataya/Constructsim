from __future__ import print_function, division
from sympy import symbols, simplify, trigsimp
from sympy.physics.mechanics import (dynamicsymbols, ReferenceFrame, Point, 
                                     inertia, RigidBody, Lagrangian, LagrangesMethod)
from sympy.physics.vector import init_vprinting, vlatex
init_vprinting(use_latex='mathjax', pretty_print=False)
inertial_frame = ReferenceFrame('I') 
lower_link_frame = ReferenceFrame('L') 
upper_link_frame = ReferenceFrame('U') 


theta_1, theta_2 = dynamicsymbols('theta_1 theta_2')
dtheta_1, dtheta_2 = dynamicsymbols('theta_1 theta_2', 1)

tau_1, tau_2 = dynamicsymbols('tau_1 tau_2')
l_1, l_2 = symbols('l_1 l_2', positive = True)
r_1, r_2 = symbols('r_1 r_2', positive = True)
m_1, m_2, b, g = symbols('m_1 m_2 b g')
I_1_yy, I_2_yy = symbols('I_{1yy}, I_{2yy}') 


lower_link_frame.orient(inertial_frame, 'Axis', [theta_1, inertial_frame.y])
upper_link_frame.orient(lower_link_frame, 'Axis', [theta_2, lower_link_frame.y])


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

L = Lagrangian(inertial_frame, BODY_1, BODY_2)
print(L)


FL = [(lower_link_frame, (tau_1 - b * dtheta_1) * inertial_frame.y - (tau_2 - b * dtheta_2) * inertial_frame.y),
      (upper_link_frame, (tau_2 - b * dtheta_2) * inertial_frame.y)]


LM = LagrangesMethod(L, [theta_1, theta_2], frame=inertial_frame, forcelist=FL)
L_eq = LM.form_lagranges_equations()
print(trigsimp(L_eq))


mass_matrix = trigsimp(LM.mass_matrix_full)
print(mass_matrix)


forcing_vector = trigsimp(LM.forcing_full)
print(forcing_vector)


from numpy import deg2rad, rad2deg, array, zeros, linspace
from scipy.integrate import odeint
from pydy_code_gen.code import generate_ode_function


from matplotlib.pyplot import rcParams
import matplotlib.pyplot as plt
rcParams['figure.figsize'] = (14.0, 6.0)


constants = [l_1,
             r_1,
             m_1,
             I_1_yy,
             l_2,
             r_2,
             m_2,
             I_2_yy,
             b,
             g]
constants


coordinates = [theta_1, theta_2]
speeds = [dtheta_1, dtheta_2]
specified = [tau_1, tau_2]


right_hand_side = generate_ode_function(mass_matrix, forcing_vector, constants,
                                        coordinates, speeds, specified,
                                        generator='lambdify')

x0 = zeros(4)
x0[:2] = deg2rad(0.01)


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


numerical_specified = zeros(2)

args = {'constants': numerical_constants,
        'specified': numerical_specified}

frames_per_sec = 60
final_time = 30

t = linspace(0.0, final_time, final_time * frames_per_sec)


y = odeint(right_hand_side, x0, t, args=(args,))


plt.plot(t, rad2deg(y[:, :2]))
plt.xlabel('Time [s]')
plt.ylabel('Angle [deg]')
plt.legend(["${}$".format(vlatex(c)) for c in coordinates])
plt.show()

plt.plot(t, rad2deg(y[:, 2:]))
plt.xlabel('Time [s]')
plt.ylabel('Angular Rate [deg/s]')
plt.legend(["${}$".format(vlatex(s)) for s in speeds])
plt.show()



