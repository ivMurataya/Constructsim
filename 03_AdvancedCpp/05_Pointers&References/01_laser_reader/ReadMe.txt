

Write a ROS node that subscribes to the laser of robot1 and assigns the ranges array to a pointer.

Tasks:

    Create a new C++ ROS package called laser_reader which depends on roscpp and sensor_msgs.
    Create a node with a subscriber to get laser scan data using the code below (laser_reader.h and laser_reader.cpp)
    Then retrieve the ranges array from every laser message that arrives at the callback and assign it to a pointer to array named last_laser_ranges, which is a member of the TurtleClass class.
    Print on the screen the range measured by the ray at the front of the robot for the last scan received.

Hints for Exercise 5.1:

    Use the laser_reader.h and laser_reader.cpp files below as your template for your program.
    You need to get the laser data on the laser_callback method.
    The sensor_msgs::LaserScan::ConstPtr is a pointer to a message of type LaserScan which contains the data of the latest laser measurement in the ranges field. Then in order to access its elements, you will need to use pointers nomenclature.
    The structure of the LaserScan message is as follows:

Header header            # timestamp in the header is the acquisition time of 
                         # the first ray in the scan.
                         #
                         # in frame frame_id, angles are measured around 
                         # the positive Z axis (counterclockwise, if Z is up)
                         # with zero angle being forward along the x axis
                         
float32 angle_min        # start angle of the scan [rad]
float32 angle_max        # end angle of the scan [rad]
float32 angle_increment  # angular distance between measurements [rad]

float32 time_increment   # time between measurements [seconds] - if your scanner
                         # is moving, this will be used in interpolating position
                         # of 3d points
float32 scan_time        # time between scans [seconds]

float32 range_min        # minimum range value [m]
float32 range_max        # maximum range value [m]

float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
float32[] intensities    # intensity data [device-specific units].  If your
                         # device does not provide intensities, please leave
                         # the array empty.




    Each laser scan has 720 distance values
    You will need to do a new of the last_laser_ranges in the constructor of the class
    Use the new operator to allocate memory in the required size.
    Then for every call of the laser callback, copy the ranges array into your last_laser_ranges.
    Remember to do a delete of the pointer in the destructor of the class, so the memory gets freed

