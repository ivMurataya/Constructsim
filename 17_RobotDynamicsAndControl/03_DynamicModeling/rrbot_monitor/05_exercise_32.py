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
#plt.plot(t, th2, t, th3)
# plt.legend(['$th_2$', '$th_3$'])
# plt.title('Orientation of the link')
# plt.show()

(np.rad2deg(th2[-1]), np.rad2deg(th3[-1]))

w2, w3 = m[:,5], m[:,11]
plt.plot(t,w2,t,w3)
#plt.plot(t, th2, t, th3,t,w2,t,w3)
plt.legend(['$w_2$','$w_3$'])
plt.title('Angular velocity of the center of mass')
plt.show()


# pe, ke = m[:, 12], m[:, 13]
# pe_ref = pe[-1]
# #plt.plot(t, th2, t, th3,t,w2,t,w3,t, pe-pe_ref, t, ke)
# plt.plot(t, pe-pe_ref, t, ke)
# plt.legend(['potential energy', 'kinetic energy'])
# plt.show()


