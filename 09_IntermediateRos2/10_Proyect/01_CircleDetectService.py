import rclpy
import time
import matplotlib.pyplot as plt
import numpy as np

from sklearn.cluster import AgglomerativeClustering
from sklearn.neighbors import kneighbors_graph
from std_srvs.srv import Empty

from sensor_msgs.msg import LaserScan
from rclpy.node import Node

from numpy import inf

class LaserDataProcess(Node):

    def __init__(self, plot_debug=False, pop_last_point=True):
        super().__init__('laser_data_processor')
        self._pop_last_point = pop_last_point
        self._plot_debug = plot_debug
        # Subscribe to the laser scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10)
                # Create a service to trigger find_circles function
        self.srv = self.create_service(Empty, '/find_circles', self.service_callback)
        self.latest_laser_msg = None
        

        self.get_logger().info("Laser Data Processor Node Initialized")

    def laser_callback(self, laser_msg):
        #self.get_logger().info("Received LaserScan message")
        # Store the latest LaserScan message
        self.latest_laser_msg = laser_msg

    def service_callback(self, request, response):
        self.get_logger().info("Service call received, executing find_circles")
        if self.latest_laser_msg:
            cicle_dict, cicle_dict_result = self.find_circles(self.latest_laser_msg)
            self.get_logger().info(f"Detected Circles: {cicle_dict_result}")
            self.get_logger().info(f"Circle Dict: {cicle_dict}")
            
            max_key, max_value = max(((key, value[1]) for key, value in cicle_dict_result.items() if value[0]),  key=lambda x: x[1], default=(None, None))
            # Extract X, Y, Z values
            X, Y, Z = cicle_dict.get(max_key, (None, None, None))

            

            print(f"Key: {max_key}, Value: {max_value}")
            print(f"X: {X}, Y: {Y}, Z: {Z}")
        else:
            self.get_logger().info("No LaserScan message received yet.")
        return response

    
    def find_circles(self, laser_msg):
        laser_X_numpy_array = self.ros_msg_laser_to_numpy_array(laser_msg)
        objects_array, cicle_dict, cicle_dict_result = self.cluster_numpy_array(in_X=laser_X_numpy_array, n_clusters = 15)
        
        return cicle_dict, cicle_dict_result

    def ros_msg_laser_to_numpy_array(self, laser_msg):

        # Xtract data form laser
        self.frame_id = laser_msg.header.frame_id
        laser_angle_min = laser_msg.angle_min
        laser_angle_max = laser_msg.angle_max
        laser_angle_increment = laser_msg.angle_increment

        in_LASER_SAMPLE_ranges = laser_msg.ranges

        self.get_logger().info(f"values: min{laser_angle_min}, max {laser_angle_max}, inc {laser_angle_increment}, samp {in_LASER_SAMPLE_ranges}")
        
        # We replace all the inf values for 0.0 to avoud issues with the clustering
        index_value = 0
        for value in in_LASER_SAMPLE_ranges:
            if value == inf:
                in_LASER_SAMPLE_ranges[index_value] = 0.0
            index_value += 1

        # We remove un last point due to missing one point after doing incremental angle gen
        if self._pop_last_point:
            in_LASER_SAMPLE_ranges.pop()

        in_LASER_NP_ARRAY = np.array(in_LASER_SAMPLE_ranges)
        
        # We replace Infinite value for 0.0
        laser_fixed_array = in_LASER_NP_ARRAY.reshape(1,-1)

        # from laser_data_sample import LASER_NP_ARRAY_FIXED
        # laser_fixed_array = LASER_NP_ARRAY_FIXED

        angle_range = laser_angle_max - laser_angle_min
        # Create angle range
        #angle_range = np.arange(laser_angle_min, laser_angle_max, laser_angle_increment)
        angle_range = np.linspace(laser_angle_min, laser_angle_max, num=len(in_LASER_SAMPLE_ranges), endpoint=False)

        angle_range_fix = angle_range.reshape(1,-1)
        t_sin = np.sin(angle_range_fix)
        t_cos = np.cos(angle_range_fix)

        x = np.multiply(laser_fixed_array, t_cos)
        y = np.multiply(laser_fixed_array, t_sin)

        aux_x = np.concatenate((x, y))
        laser_X_numpy_array = aux_x.T

        return laser_X_numpy_array

    def cluster_numpy_array(self, in_X, n_clusters = 15, linkage = 'single'):
        
        knn_graph = kneighbors_graph(in_X, n_clusters, include_self=False)

        connectivity = knn_graph
        
        model = AgglomerativeClustering(linkage=linkage,
                                        connectivity=connectivity,
                                        n_clusters=n_clusters)
        t0 = time.time()
        model.fit(in_X)

        unique_model_labels = np.unique(model.labels_)
        num_labels = unique_model_labels.shape[0]

        # objects_array = np.full((num_labels, 1), None)
        objects_array = {}

        index_label = 0
        for element in in_X:
            
            label = str(model.labels_[index_label])
            

            # We place each element in different arrays, depending on their Label
            if label not in objects_array:
                objects_array[label] = [element]
            else:
                objects_array[label].append(element)

            index_label += 1



        elapsed_time = time.time() - t0

        

        if self._plot_debug:
            plt.figure(figsize=(10, 4))
            index = 0
            plt.subplot(1, 1, index + 1)

            plt.scatter(in_X[:, 0], in_X[:, 1], c=model.labels_,
                        cmap=plt.cm.nipy_spectral)
            plt.title('linkage=%s\n(time %.2fs)' % (linkage, elapsed_time),
                        fontdict=dict(verticalalignment='top'))
            plt.axis('equal')
            plt.axis('off')

            plt.subplots_adjust(bottom=0, top=.83, wspace=0,
                                left=0, right=1)
            plt.suptitle('n_cluster=%i, connectivity=%r' %
                            (n_clusters, connectivity is not None), size=17)

            plt.show()

        # Separte all clusters by labels, Plot all the different clusters by labels

        cicle_dict = {}
        cicle_dict_result = {}

        for key, value in objects_array.items():
            value_NP_ARRAY = np.array(value)

            ## Detect circle

            cicle_array_data = self.define_circle(data_array=value_NP_ARRAY)
            cicle_dict[key] = cicle_array_data
            cicle_center_np = np.array([cicle_array_data[0],cicle_array_data[1]])

            result, circle_rating = self.check_if_circle(center_point=cicle_center_np, 
                            radius=cicle_array_data[2], 
                            testing_points_array=value_NP_ARRAY,
                            delta=0.009,
                            rating_limit=0.7)
            
            cicle_dict_result[key] = [result, circle_rating]

            # Plot
            if self._plot_debug:

                plt.figure(figsize=(10, 4))
                index = 0 
                linkage = 'single'
                plt.subplot(1, 1, index + 1)

                plt.scatter(value_NP_ARRAY[:, 0], value_NP_ARRAY[:, 1])
                plt.title("Label="+key+",CIRCLE="+str(result)+",circle_rating="+str(circle_rating))
                plt.axis('equal')
                plt.axis('off')

                plt.subplots_adjust(bottom=0, top=.83, wspace=0,
                                    left=0, right=1)
                
                plt.show()

        return objects_array, cicle_dict, cicle_dict_result


    def define_circle(self, data_array, min_num_points_circle=10):
        """
        Returns the center and radius of the circle passing the given 3 points.
        In case the 3 points form a line, returns (None, infinity).
        """
        length_array_aux = data_array.shape
        length_array = length_array_aux[0]
        if length_array < min_num_points_circle:
            # We cant calculate it or we can but its three points and its too low
            cx = None
            cy = None
            radius = None
        else:    
            p1 =  data_array[0]
            p2 = data_array[int(length_array/2)]
            p3 = data_array[length_array-1]

            temp = p2[0] * p2[0] + p2[1] * p2[1]
            bc = (p1[0] * p1[0] + p1[1] * p1[1] - temp) / 2
            cd = (temp - p3[0] * p3[0] - p3[1] * p3[1]) / 2
            det = (p1[0] - p2[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p2[1])

            if abs(det) < 1.0e-6:
                # For the case of infinite radius value            
                cx = 0
                cy = 0
                radius = None
            else:
                # Center of circle
                cx = (bc*(p2[1] - p3[1]) - cd*(p1[1] - p2[1])) / det
                cy = ((p1[0] - p2[0]) * cd - (p2[0] - p3[0]) * bc) / det

                radius = np.sqrt((cx - p1[0])**2 + (cy - p1[1])**2)
        
        return [cx, cy, radius]


    def check_if_circle(self, center_point, radius, testing_points_array, delta=0.009, rating_limit=0.8):

        if radius is not None:
            length_testing_points = testing_points_array.shape[0]
            acumulated_success = 0.0

            for test_point in testing_points_array:

                dist = np.linalg.norm(center_point-test_point)
                difference = abs(radius - dist)
                is_circle = difference <= delta
                acumulated_success += int(is_circle)

            circle_rating = float(acumulated_success) / float(length_testing_points)


            result= circle_rating > rating_limit
        else:
            result = False
            circle_rating = 0.0

        return result, circle_rating


def test_laser_circle_detector():
    laser_Data_obj = LaserDataProcess(plot_debug=True, pop_last_point=False)
    laser_data = LaserScan()
    # We define inf as 0.0 to avoid issues in numpy

    from laser_data_sample import RAW_LASER_SAMPLE_ranges
    from laser_data_sample import angle_min, angle_max, angle_increment, LASER_frame_id

    laser_data.ranges = RAW_LASER_SAMPLE_ranges
    laser_data.angle_min = angle_min
    laser_data.angle_max = angle_max
    laser_data.angle_increment = angle_increment
    laser_data.header.frame_id = LASER_frame_id

    cicle_dict, cicle_dict_result = laser_Data_obj.find_circles(laser_data)
    print("CIRCLE DICT="+str(cicle_dict))
    print("CICLE RESULT="+str(cicle_dict_result))



def main():
    rclpy.init()
    node = LaserDataProcess(plot_debug=True, pop_last_point=False)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
