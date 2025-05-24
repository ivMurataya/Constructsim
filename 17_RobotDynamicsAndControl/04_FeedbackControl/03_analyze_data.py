import matplotlib.pyplot as plt
import math, numpy as np, rosbag
from sympy.physics.mechanics import dynamicsymbols
from sympy.physics.vector import vlatex

bag = rosbag.Bag('/home/user/catkin_ws/src/rrbot_feedback/scripts/data.bag')
t, position, velocity, effort = [], [], [], []
theta_1, theta_2 = dynamicsymbols('theta_1 theta_2')
coordinates = [theta_1, theta_2]

for topic, msg, tm in bag.read_messages(topics=['/rrbot/joint_states']):
    t.append(tm.secs+tm.nsecs/1000000000.0)
    th1 = msg.position[0]
    th1 = math.atan2(math.sin(th1), math.cos(th1))
    th2 = msg.position[1]
    th2 = math.atan2(math.sin(th2), math.cos(th2))
    position.append([math.degrees(th1), math.degrees(th2)])

bag.close()
t = np.array(t) 
position = np.array(position)

plt.plot(t-t[0], position,'.')
plt.xlabel('Time [s]')
plt.ylabel('Angle [deg]')
plt.legend(["${}$".format(vlatex(c)) for c in coordinates])
plt.show()
