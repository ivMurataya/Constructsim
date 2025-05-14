import matplotlib.pyplot as plt
import numpy as np
import rosbag
bag = rosbag.Bag('/home/user/catkin_ws/src/data.bag')
t, data = [], []
for topic, msg, tm in bag.read_messages(topics=['/dynamic_data']):
    t.append(tm.secs+tm.nsecs/1000000000.0)
    data.append(msg.data)
bag.close()
m = np.array(data)

vx2, vy2, vx3, vy3 = m[:, 3], m[:, 4], m[:, 9], m[:, 10]
plt.plot(t, vx2, t, vy2, t, vx3, t, vy3)
plt.legend(['$v_{x2}$', '$v_{y2}$', '$v_{x3}$', '$v_{y3}$'])
plt.title('Linear velocity of the center of mass')
plt.show()
