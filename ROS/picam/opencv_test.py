#!/usr/bin/env python
import cv2
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
#cap = cv2.VideoCapture(f'v4l2src device=/dev/video0 io-mode=2 ! image/jpeg, width=(int)320, height=(int)240 ! nvjpegdec !1 video/x-raw, format=I420 ! appsink', cv2.CAP_GSTREAMER)
print(cap.isOpened())

# image attributes
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_FPS, 30)

if not cap.isOpened():
    print('Cannot open camera')

print("Press key 'q' to exit ...")
while True:
    ret, frame = cap.read()
    rframe = cv2.rotate(frame, cv2.ROTATE_180)
    if not ret:
        break
    cv2.imshow('frame', rframe)
    
    if cv2.waitKey(1) == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()