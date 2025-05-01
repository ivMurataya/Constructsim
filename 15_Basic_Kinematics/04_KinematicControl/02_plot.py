import numpy as np
from plot_functions import plot_trajectory
from plot_functions import plot_error

path_to_file = '/home/user/catkin_ws/src/unit5/'
open_loop_trajectory = np.load(path_to_file+'trajectory.npy')
plot_trajectory(open_loop_trajectory)

#path_to_file = '/home/user/catkin_ws/src/unit5/'
#open_loop_trajectory = np.load(path_to_file+'/trajectory.npy')

#waypoints = np.load(path_to_file+'waypoints.npy')
#plot_trajectory(open_loop_trajectory, waypoints)

#ref_points = [(0,0),(1,-1),(2,-2),(3,-2),(4,-1),(3.5,-0.5),(3,0),(3,1),(2,1),(1,0),(0,0)]
#plot_error(waypoints, ref_points)