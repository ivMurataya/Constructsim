#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class ShowingImage(object):

    def __init__(self, eyeC, faceC):
        # Initialize with the provided eye and face cascade classifiers
        self.eyeClass = eyeC
        self.faceClass = faceC
        # Subscribe to the image topic
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
        # Initialize the CvBridge object for converting ROS images to OpenCV images
        self.bridge_object = CvBridge()

    def camera_callback(self, data):
        try:
            # Convert the ROS image message to an OpenCV image
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            return

        # Get image paths
        #image_1 = '/home/user/catkin_ws/src/opencv_for_robotics_images/Unit_3/Course_images/many.jpg'
        #image_1 = '/home/user/catkin_ws/src/unit3_exercises/images/test.jpg'
        image_1 = '/home/user/catkin_ws/src/opencv_for_robotics_images/Unit_3/Course_images/chris.jpg'

        
        # Read images using OpenCV
        img = cv2.imread(image_1)
        
        # Resize the image
        img = cv2.resize(img, (500, 300))

        # Convert the image to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Set parameters for the classifier
        ScaleFactor = 1.2
        minNeighbors = 5

        # Detect faces in the grayscale image
        faces = self.faceClass.detectMultiScale(gray, ScaleFactor, minNeighbors)
        for (x, y, w, h) in faces:
            # Draw a rectangle around each detected face
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            # Define the region of interest (ROI) for the face
            roi = img[y:y + h, x:x + w]

            # Detect eyes within the face ROI
            eyes = self.eyeClass.detectMultiScale(roi)
            for (ex, ey, ew, eh) in eyes:
                # Draw a rectangle around each detected eye
                cv2.rectangle(roi, (ex, ey), (ex + ew, ey + eh), (255, 0, 0), 2)
                # Define the ROI for the eye
                roi_eyes = roi[ey:ey + eh, ex:ex + ew]
                
        # Display the images with the rectangles
        cv2.imshow('Original', img)
        #cv2.imshow('Face ROI', roi)

        cv2.waitKey(1)

def main():
    # Load the Haar cascades for eye and face detection
    eye_cascade = cv2.CascadeClassifier('/home/user/catkin_ws/src/unit3_exercises/haar_cascades/eye.xml')
    face_cascade = cv2.CascadeClassifier('/home/user/catkin_ws/src/unit3_exercises/haar_cascades/frontalface.xml')
    # Initialize the ShowingImage class with the loaded cascades
    face_detection_object = ShowingImage(eye_cascade, face_cascade)
    # Initialize the ROS node
    rospy.init_node('face_detection_node', anonymous=True)
    try:
        # Keep the program alive until interrupted
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    # Destroy all OpenCV windows
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
