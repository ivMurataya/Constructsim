#!/usr/bin/env python

import cv2
import numpy as np

image = cv2.imread('/home/user/catkin_ws/src/opencv_for_robotics_images/Unit_4/Course_images/corner_test_2.png')

gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

fast = cv2.FastFeatureDetector_create() 

#Keypoints using non Max Supression
"""FAST algorithm after non-maximal Supression. 
Many points were discarded, such as the repetitve points in the middle of the image (like a pattern), 
with only the external points remaining.
"""
Keypoints_1 = fast.detect(gray, None)

#Set non Max Supression disabled 
fast.setNonmaxSuppression(False)

#Keypoints without non max Suppression
"""
 there are many keypoints detected by the FAST algorithm, 
 ut many of these points are really close to each other. 
 What that means is that maybe a corner is being detect more than once. 
 Many corners have at least 3 detections because of the nature of the algorithm (analyzing pixel by pixel).
"""
Keypoints_2 = fast.detect(gray, None)

#Create  instance of the original image

image_without_nonmax = np.copy(image)
image_with_nonmax = np.copy(image)

# Draw keypoints on top of the input image

cv2.drawKeypoints(image, Keypoints_1, image_with_nonmax, color=(0,35,250))
cv2.drawKeypoints(image, Keypoints_2, image_without_nonmax, color=(0,35,250))



cv2.imshow('Without non max Supression',image_without_nonmax)
cv2.imshow('With non max Supression',image_with_nonmax)
cv2.imshow('Original',image)
cv2.waitKey(0)
cv2.destroyAllWindows()
