from numpy import deg2rad, rad2deg, array, zeros, linspace, load
from sympy.physics.mechanics import dynamicsymbols 
from sympy.physics.vector import vlatex

from matplotlib.pyplot import rcParams
import matplotlib.pyplot as plt
rcParams['figure.figsize'] = (14.0, 6.0)

t = load('/home/user/catkin_ws/src/rrbot_dynamics/scripts/t.npy')
y = load('/home/user/catkin_ws/src/rrbot_dynamics/scripts/y.npy')
theta_1, theta_2 = dynamicsymbols('theta_1 theta_2')
omega_1, omega_2 = dynamicsymbols('omega_1, omega_2')
coordinates = [theta_1, theta_2]
speeds = [omega_1, omega_2]

plt.plot(t, rad2deg(y[:, :2]))
plt.xlabel('Time [s]')
plt.ylabel('Angle [deg]')
plt.legend(["${}$".format(vlatex(c)) for c in coordinates])
plt.show()

(rad2deg(y[-1,0]), rad2deg(y[-1,0]+y[-1,1]))
plt.plot(t, rad2deg(y[:, 2:]))
plt.xlabel('Time [s]')
plt.ylabel('Angular Rate [deg/s]')
plt.legend(["${}$".format(vlatex(s)) for s in speeds])
plt.show()
