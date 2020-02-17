# OpenCV Face detection

import numpy as np
import cv2
from picamera import PiCamera
from picamera.array import PiRGBArray
import time

# Configure the PiCamera
camera = PiCamera()
camera.resolution = (640,480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640,480))

# Give time for the camera to stabilize
time.sleep(1)


# Load a cascade file for detecting faces
face_cascade = cv2.CascadeClassifier('/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml')


for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    # grab each frame
    image = frame.array

    
    # convert to gray scale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Look for faces in the image using loaded cascade file
    faces = face_cascade.detectMultiScale(gray, 1.1, 5)

    print("Found "+str(len(faces))+" face(s)")

    # Draw a rectangle around every found face
    for (x,y,w,h) in faces:
        cv2.rectangle(image, (x,y), (x+w, y+h), (255,255,0), 2)

    # show the frame
    cv2.imshow("Frame", image)
    key = cv2.waitKey(1) & 0xFF

    # clear the stream in preparation for next frame
    rawCapture.truncate(0)

    # quit if 'q' is pressed
    if key == ord("q") or key == 27:
        break
    elif key == ord("s"):
        cv2.imwrite('/home/pi/face_detect.jpg', image)
        break
        
cv2.destroyAllWindows()

    
                        
