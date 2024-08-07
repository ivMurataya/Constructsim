#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from cv2 import aruco


class LoadPolygon(object):

    def __init__(self):
    
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.bridge_object = CvBridge()

    def camera_callback(self,data):
        try:
            # We select bgr8 because its the OpenCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        def order_coordinates(pts):
    
            #Initialize an empty array to save to next values 
            coordinates = np.zeros((4, 2), dtype="int")

            s = pts.sum(axis=1)
            coordinates[0] = pts[np.argmin(s)]
            coordinates[2] = pts[np.argmax(s)]

            diff = np.diff(pts, axis=1)
            coordinates[1] = pts[np.argmin(diff)]
            coordinates[3] = pts[np.argmax(diff)]

            return coordinates

        image = cv_image
        h,w = image.shape[:2]

        image = cv2.resize(image,(int(w*0.7), int(h*0.7)))
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        #Initialize the aruco Dictionary and its parameters 
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters =  aruco.DetectorParameters_create()

        #Detect the corners and id's in the examples 
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        #First we need to detect the markers itself, so we can later work with the coordinates we have for each.
        frame_markers = aruco.drawDetectedMarkers(image.copy(), corners, ids)

        #Show the markers detected
        cv2.imshow('markers',frame_markers)
        #Initialize an empty list for the coordinates 
        params = []

        try:

            for i in range(len(ids)):

                #Catch the corners of each tag
                c = corners[i][0]

                #Draw a circle in the center of each detection
                cv2.circle(image,(int(c[:, 0].mean()), int(c[:, 1].mean())), 3, (255,255,0), -1)
                
                #Save thhe center coordinates for each tag
                params.append((int(c[:, 0].mean()), int(c[:, 1].mean())))

            #Convert the coordinates list to an array
            params = np.array(params)

            #Draw a polygon with the coordinates
            cv2.drawContours(image,[params],-1 ,(255,0,150),-1)

            cv2.imshow('no_conversion',image) 
        except:
            pass               

        cv2.waitKey(1)



def main():
    load_polygon_object = LoadPolygon()
    rospy.init_node('load_polygon_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()


"""

The operation c[:, 0] extracts the first column of c. Here's how it works:

    : means "all rows".
    0 means "the first column".

    
# Example outputs:
corners = [
    np.array([[[100, 200], [150, 200], [150, 250], [100, 250]]], dtype=float),
    np.array([[[300, 400], [350, 400], [350, 450], [300, 450]]], dtype=float)
]

ids = np.array([[1], [2]], dtype=int)

rejectedImgPoints = [
    np.array([[[50, 60], [80, 60], [80, 90], [50, 90]]], dtype=float),
    np.array([[[200, 210], [230, 210], [230, 240], [200, 240]]], dtype=float)
]


"""
