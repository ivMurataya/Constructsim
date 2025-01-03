"""


    Let's create an object detector using subscribers and publishers.
    We want to start developing a script that will detect plants through the RGB camera sensor.
    Create a script named plant_detector_node.py.
    It should subscribe to the image topic /leo/camera/image_raw.
    We will use an AI model developed by NASA's space biology division to detect plants.
    Then, when we detect something, we will publish a String message stating that we have detected a plant in a topic named /plant_detector.

    We now have to download the AI Model and python script provided by the NASA's space biology division. 
    You can have a look at it if you want, but there is no need to understand the inner workings of this model in this course.

    cd ~/ros2_ws/src/
    git clone https://bitbucket.org/theconstructcore/basic_ros2_extra_files.git
    cp basic_ros2_extra_files/plant_detector/plant_detector.py ~/ros2_ws/src/mars_rover_tasks/mars_rover_tasks/


"""

#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String  # Import String message type
from cv_bridge import CvBridge
import cv2
from mars_rover_tasks.plant_detector import PlantDetector  # Import the AI class

class PlantDetectorNode(Node):
    def __init__(self):
        super().__init__('plant_detector_node')
        
        # Initialize the CvBridge
        self.bridge = CvBridge()
        
        # Initialize the PlantDetector
        path_to_model = "/home/user/ros2_ws/src/basic_ros2_extra_files/plant_detector/best_plant_detector_model.pth"
        self.plant_detector = PlantDetector(model_path=path_to_model)
        
        # Subscribe to the image topic
        self.subscription = self.create_subscription(
            Image,
            '/leo/camera/image_raw',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Initialize the Publisher for plant detection results
        self.publisher_ = self.create_publisher(String, '/plant_detector', 10)
    
    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Use the PlantDetector to make a prediction
        prediction = self.plant_detector.predict(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
        
        # Determine the result message based on the prediction
        if prediction > 0.5:
            result = f"Plant detected with confidence: {prediction:.2f}"
            self.get_logger().warning(result)
        else:
            result = f"No plant detected. Confidence: {1 - prediction:.2f}"
            self.get_logger().info(result)
        
        # Publish the result as a String message
        msg = String()
        msg.data = result
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    plant_detector_node = PlantDetectorNode()

    rclpy.spin(plant_detector_node)

    # Destroy the node explicitly (optional)
    plant_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


