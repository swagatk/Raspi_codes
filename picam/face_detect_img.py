# face detection with opencv

import cv2
import numpy


image = cv2.imread('/home/pi/face.jpg')

# Load a cascade file for detecting faces
face_cascade = cv2.CascadeClassifier(
    '/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml')

# convert to gray scale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Look for faces in the image using loaded cascade file
faces = face_cascade.detectMultiScale(gray, 1.1, 5)

print("Found "+str(len(faces))+" face(s)")

# Draw a rectangle around every found face
for (x,y,w,h) in faces:
    cv2.rectangle(image, (x,y), (x+w, y+h), (255,255,0), 2)

# Save the result image
cv2.imwrite('/home/pi/result.jpg', image)
