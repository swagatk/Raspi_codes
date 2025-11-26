# This is tested on Trixie
# with Python 3.11.9

import cv2
import mediapipe as mp

# Open the camera (Index 0 is usually the Pi Camera)
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

# Set resolution (optional, e.g., 640x480)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)



# CRITICAL: Libcamerify needs a moment to wake up the hardware
time.sleep(2)

if not cap.isOpened():
    print("ERROR: Could not open camera.")
    exit()



print("2. Camera Opened. Starting Loop...")

while True:
    ret, frame = cap.read()
    if not ret:
        print('no frames found')
        break

    # ... Run your MediaPipe code on 'frame' here ...

    cv2.imshow('MediaPipe', frame)
    if cv2.waitKey(5) & 0xFF == 27:
        break

cap.release()
