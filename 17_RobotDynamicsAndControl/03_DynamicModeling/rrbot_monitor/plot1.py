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

x2, y2, x3, y3 = m[:, 0], m[:, 1], m[:, 6], m[:, 7]
plt.plot(x2, y2, x3, y3)
plt.axis('equal')
plt.legend(['$CM_2$', '$CM_3$'])
plt.title('Trajectory of the center of mass')
plt.show()
