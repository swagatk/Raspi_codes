# CamJam Edukit 3  - Robotics
# Worksheet 3 - Motor Test Code


import RPi.GPIO as GPIO
import time

#set GPIO modes
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


# Set the GPIO Pin Mode
GPIO.setup(7, GPIO.OUT)
GPIO.setup(8, GPIO.OUT)
GPIO.setup(9, GPIO.OUT)
GPIO.setup(10, GPIO.OUT)

# Turn off all motors
GPIO.output(7, 0)
GPIO.output(8, 0)
GPIO.output(9, 0)
GPIO.output(10, 0)

print('Right motor forward')
# Turn the right motor forwards
GPIO.output(9, 1)
GPIO.output(10, 0)

input("Press Enter to Continue:")

GPIO.output(9,0)

print('Left motor forward')
# Turn the left motor forward
GPIO.output(7, 0)
GPIO.output(8, 1)

input('Press enter to stop')
GPIO.output(8, 0)

# Wait for 1 second


GPIO.cleanup()
