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

w2, w3 = m[:, 5], m[:, 11]
plt.plot(t, w2, t, w3)
plt.legend(['$w_2$', '$w_3$'])
plt.title('Angular velocity of the center of mass')
plt.show()
