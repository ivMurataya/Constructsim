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

pe, ke = m[:, 12], m[:, 13]
pe_ref = pe[-1]
plt.plot(t, pe-pe_ref, t, ke)
plt.legend(['potential energy', 'kinetic energy'])
plt.show()
