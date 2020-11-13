# camjam edukit 3 - Robotics
# worksheet 6  - measuring distance

import RPi.GPIO as GPIO
import time
import numpy as np

#set GPIO modes
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# define GPIO pins to use

pinTrigger = 17
pinEcho = 18

print('Ultrasonic measurement')

#set up Pins
GPIO.setup(pinTrigger, GPIO.OUT)
GPIO.setup(pinEcho, GPIO.IN)

def measure():

    #  Average over 5 readings to reduce noise
    dist = []
    for i in range(5):

        #set trigger to False
        GPIO.output(pinTrigger, False)
        time.sleep(0.001)

        # send out a pulse at a freq of 10 micro hertz
        GPIO.output(pinTrigger, True)
        time.sleep(0.00001)
        GPIO.output(pinTrigger, False)

        # start the timer
        StartTime = time.time()

        # start time is reset until the Echo Pin is taken high
        while GPIO.input(pinEcho) == 0:
            StartTime = time.time()
            
        # stop when echo pin is no longer high - end time
        while GPIO.input(pinEcho) == 1:
            StopTime = time.time()
            interval = StopTime - StartTime
            #print('Interval: ', interval)
            # this block is really not required
            if interval >= 0.04:
                #print('Either you are too close or too far to me to see.')
                #StopTime = StartTime
                break
    
        # calculate pulse length
        ElapsedTime = StopTime - StartTime

        # Distance travelled by the pulse in that time in cm
        Distance = (ElapsedTime * 34326)/2.0

        dist.append(Distance)

    return np.mean(dist) 
        
    



try:
    while True:
        dist = measure()
        print('Distance in cm: ', dist)
        time.sleep(0.1)
except KeyboardInterrupt:
    GPIO.cleanup()
        
    
