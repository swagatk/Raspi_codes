# camjam edukit - 3: robotics
# worksheet 5 - line detection

import RPi.GPIO as GPIO
import time

# set GPIO modes
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


# set variables for the GPIO Pins
pinLineFollower = 25

# set pin 25 as input
GPIO.setup(pinLineFollower, GPIO.IN)


try:
    while True:
        if GPIO.input(pinLineFollower) == 0:
            print('The sensor is seeing black surface')
        else:
            print('The sensor is seeing white surface')
        time.sleep(0.2)
except KeyboardInterrupt:
    GPIO.cleanup()
        
            
