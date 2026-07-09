# Import the necessary packages and scripts for this software to run
import cv2
from collections import Counter
from pirobot.module import findnameoflandmark, findpostion
import math
import pygame
import sys

# Initialize the video capture
#cap = cv2.VideoCapture(cv2.CAP_V4L2)


tip = [8, 12, 16, 20]
tipname = [8, 12, 16, 20]
fingers = []
finger = []

def init():
    global cap
    cap = cv2.VideoCapture(cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    pygame.init()
    win = pygame.display.set_mode((100, 100))

# Function to fetch the current fingers direction
def getPosition(frame):
    # Below is used to determine the location of the joints of the fingers
    a = findpostion(frame)
    b = findnameoflandmark(frame)
    
    fingers = {"UP": 0, "DOWN": 0, "LEFT": 0, "RIGHT": 0, "EXIT": 0, "": 0}
    
    # Below is a series of If statements that will determine the direction of each finger
    if len(b and a) != 0:
        for id in range(0, 4):
            angle = math.degrees(math.atan2(a[tip[id] - 2][2] - a[tip[id]][2], a[tip[id] - 2][1] - a[tip[id]][1]))

            if angle < 0:
                angle += 360
            #print('angle = ', angle)
            direction = ""
            if 45 <= angle <= 135:
                direction = "UP"
            elif 260 < angle < 300:
                direction = "DOWN"
            elif 135 < angle < 225:
                direction = "LEFT"
            elif angle > 320 or angle < 20:
                direction = 'RIGHT'
            elif 230 < angle < 260: 
                direction = "EXIT"
            else:
                direction = ""
            fingers[direction] += 1
    
            max_direction = max(fingers, key=fingers.get) # Find the direction with the maximum count
    
        return max_direction
    else:
        return ("")  # Return empty if no fingers are detected

# Function to release resources when stopping
def stop():
    cap.release()
    cv2.destroyAllWindows()
    pygame.quit()
    sys.exit()

def fetch_command():
    ret, frame = cap.read() # Capture a frame

    if ret is False:
        print('Failed to open Camera')
        direction = 'EXIT'
    else:
        # Unedit the below line if your live feed is produced upside down
        #flipped = cv2.flip(frame, flipCode = -1)
    
        frame1 = cv2.resize(frame, (320, 240))  # Resize the frame
        direction = getPosition(frame) # Fetch the detected direction
    
        cv2.imshow("Hand Detection", frame) # Display the frame with hand landmarks
        cv2.waitKey(1) & 0xFF # Wait for a short period to allow the frame to be displayed
    return direction
