import cv2
import numpy

image = cv2.imread('/home/pi/face.jpg')

a = cv2.ellipse(image, (320,240), (30,20), 0,0,360, (0,0,255), 3)
p
cv2.namedWindow('Main')
cv2.imshow('Main', image)
k = cv2.waitKey(0)

if k == 27:
    cv2.destroyAllWindows()
    
