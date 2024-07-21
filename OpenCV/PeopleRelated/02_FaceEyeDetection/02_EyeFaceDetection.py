#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


class ShowingImage(object):

    def __init__(self,eyeC, faceC):
        self.eyeClass = eyeC
        self.faceClass = faceC
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.bridge_object = CvBridge()

    def camera_callback(self,data):
        try:
            # We select bgr8 because its the OpenCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        #Get Images paht
        image_1 = '/home/user/catkin_ws/src/opencv_for_robotics_images/Unit_3/Course_images/chris.jpg'
        image_2 = '/home/user/catkin_ws/src/opencv_for_robotics_images/Unit_3/Course_images/chris.jpg'
        
        #Read Images using CV2
        img = cv2.imread(image_1)
        img2 = cv2.imread(image_2)
        
        #Resize images
        img = cv2.resize(img,(500,300))
        img2 = cv2.resize(img2,(500,300))

        #Images in Gray Scale needed to aply the filters
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray_2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

        ScaleFactor = 1.2
        minNeighbors = 3

        #Aply Classification to Gray Images, creates an array of positions for each face or eye found
        eyes = self.eyeClass.detectMultiScale(gray, ScaleFactor, minNeighbors)
        faces = self.faceClass.detectMultiScale(gray_2, ScaleFactor, minNeighbors)

        for (x,y,w,h) in eyes:
            
            cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)  
            roi = gray[y:y+h, x:x+w] #For each eye, creates a Region of Interes


        for (x,y,w,h) in faces:
            cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
            roi2 = gray_2[y:y+h, x:x+w]  
            
        cv2.imshow('Original',img)
        cv2.imshow('Face ROI',roi2)

        cv2.waitKey(1)



def main():
    
    eye_cascade = cv2.CascadeClassifier('/home/user/catkin_ws/src/unit3_exercises/haar_cascades/eye.xml')
    face_cascade = cv2.CascadeClassifier('/home/user/catkin_ws/src/unit3_exercises/haar_cascades/frontalface.xml')
    face_detection_object = ShowingImage(eye_cascade,face_cascade)
    rospy.init_node('face_detection_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
