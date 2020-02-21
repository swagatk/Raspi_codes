# Filters
import numpy as np
import cv2
from picamera import PiCamera
from picamera.array import PiRGBArray
import time

# Configure the PiCamera
camera = PiCamera()
camera.resolution = (320,240)
camera.rotation = 180
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(320,240))

# Give time for the camera to stabilize
time.sleep(1)


# Create a window
cv2.namedWindow('Input')
cv2.namedWindow('Blur')
cv2.namedWindow('Hist')
cv2.namedWindow('Canny')


# Process each frame from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    # grab each frame
    image = frame.array
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # define a kernel
    kernel = np.ones((5,5), np.float32)/25
    avg = cv2.filter2D(image, -1, kernel)
    blur = cv2.blur(image, (5,5))
    gblur = cv2.GaussianBlur(image, (5,5), 0)
    mblur = cv2.medianBlur(image, 5)
    canny = cv2.Canny(image, 100,200)
    

    cv2.imshow('Input', image)
    cv2.imshow('Blur', blur)
    cv2.imshow('Hist', hist)
    cv2.imshow('Canny', canny)               
                    

    # refresh video buffer
    rawCapture.truncate(0)

    ch = cv2.waitKey(5)
    if ch == 27:
        break

cv2.destroyAllWindows()
                
                
