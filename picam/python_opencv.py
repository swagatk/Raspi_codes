# face detection with opencv

import io
import picamera
import cv2
import numpy


### Create a memory stream so photos don't need to be saved in a file
##stream = io.BytesIO()
##
### Get the picture with low resolution
##with picamera.PiCamera() as camera:
##    camera.resolution = (320, 240)
##    camera.capture(stream, format='jpeg')
##
##
### convert the picture into a binary array
##buff = numpy.fromstring(stream.getvalue(), dtype=numpy.uint8)
##
### Now create an OpenCV image
##image = cv2.imdecode(buff,1)
image = cv2.imread('/home/pi/face.jpg')

# Load a cascade file for detecting faces
face_cascade = cv2.CascadeClassifier('/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml')

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
