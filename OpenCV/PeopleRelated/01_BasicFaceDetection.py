import numpy as np
import cv2

face_cascade = cv2.CascadeClassifier('/home/user/catkin_ws/src/opencv_for_robotics_images/Unit_3/haar_cascades/frontalface.xml')

img = cv2.imread('/home/user/catkin_ws/src/opencv_for_robotics_images/Unit_3/Course_images/face.jpg')
img_2 = cv2.imread('/home/user/catkin_ws/src/opencv_for_robotics_images/Unit_3/Course_images/many.jpg')

img = cv2.resize(img,(400,700))


gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
gray_2 = cv2.cvtColor(img_2, cv2.COLOR_BGR2GRAY)


ScaleFactor = 1.2

minNeighbors = 3
 
faces = face_cascade.detectMultiScale(gray, ScaleFactor, minNeighbors)
faces_2 = face_cascade.detectMultiScale(gray_2, ScaleFactor, minNeighbors)


for (x,y,w,h) in faces:
    
    cv2.rectangle(img,(x,y),(x+w,y+h),(255,255,0),2)  
    roi = img[y:y+h, x:x+w]


for (x,y,w,h) in faces_2:
    cv2.rectangle(img_2,(x,y),(x+w,y+h),(255,255,0),2)
    roi = img_2[y:y+h, x:x+w]  
    

cv2.imshow('Face',img)

cv2.imshow('Faces',img_2)

cv2.waitKey(0)
cv2.destroyAllWindows()
