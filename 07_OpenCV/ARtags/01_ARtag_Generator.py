import cv2 
import numpy as np 
from cv2 import aruco

'''
The first one is the dictionary we are going to use. 
This one was defined at the begining of the code (DICT_6X6_250). 
The second parameter is the id that will be assigned. 
In this case, we have a loop so we will create 4 markers with the ids 0,1,2, and 3, respectively. 
Finally, the Size makes reference to the size of the output image, in this case, it will be 700x700 pixels.
Once you have this code running, you have to press the spacebar as many times as
the number of artags you said to write. In this case, you will press it 4 times until the code stops.
'''
#Initialize the dictionary
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
for i in range (1, 5):

    size = 700
    img = aruco.drawMarker(aruco_dict, i, size)
    
    cv2.imwrite('/home/user/catkin_ws/src/unit5/Tags/image_'+str(i)+".jpg",img)
    
    cv2.imshow('artag',img)
    cv2.waitKey(0)
    cv2.destroyAllWindows
