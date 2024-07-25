#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class LoadImage(object):

    def __init__(self):
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.bridge_object = CvBridge()
       
    def camera_callback(self,data):
        
        try:
            # We select bgr8 because its the OpenCV encoding by default
            image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        img =cv2.imread('/home/user/catkin_ws/src/opencv_for_robotics_images/Unit_3/Course_images/test_e.jpg')

        # Lets initialize the HOG descriptor
        hog = cv2.HOGDescriptor()
        hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        #The image is pretty big so we will gibve it a resize
        imX = 720
        imY = 1080
        img = cv2.resize(img,(imX,imY))
        #We will define de 8x8 blocks in the winStride
        boxes, weights = hog.detectMultiScale(img, winStride=(8,8))
        boxes = np.array([[x, y, x + w, y + h] for (x, y, w, h) in boxes])

        for (xA, yA, xB, yB) in boxes:
            
            #Center in X 
            medX = xB - xA 
            xC = int(xA+(medX/2)) 

            #Center in Y
            medY = yB - yA 
            yC = int(yA+(medY/2)) 

            #Draw a circle in the center of the box 
            cv2.circle(img,(xC,yC), 1, (0,255,255), -1)

            # display the detected boxes in the original picture
            cv2.rectangle(img, (xA, yA), (xB, yB),
                                (255, 255, 0), 2)   
                
        cv2.imshow('image',img)
        cv2.waitKey(1)


def main():
    load_image_object = LoadImage()
    rospy.init_node('load_image_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
    #except rospy.ROSInterruptException:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
