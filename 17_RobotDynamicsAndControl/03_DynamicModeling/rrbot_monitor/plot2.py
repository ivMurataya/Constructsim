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

th2, th3 = m[:, 2], m[:, 8]
plt.plot(t, th2, t, th3)
plt.legend(['$th_2$', '$th_3$'])
plt.title('Orientation of the link')
plt.show()
