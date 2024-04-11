#!/usr/bin/env python

# load the libraries
import RPi.GPIO as GPIO
import time
import numpy as np

#set GPIO modes
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


# set variables for GPIO  Pins
pinTrigger = 17     # ultrasonic sensor is required
pinEcho = 18

# configure pins to activate Ultrasonic Sensor
GPIO.setup(pinTrigger, GPIO.OUT)
GPIO.setup(pinEcho, GPIO.IN)



def measure():
    #  Average over 5 readings to reduce noise
    dist = []
    for i in range(5):

        #set trigger to False
        GPIO.output(pinTrigger, False)
        time.sleep(0.0001)

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
            if interval >= 0.02:
                #print('Either you are too close or too far to me to see.')
                #StopTime = StartTime
                break
    
        # calculate pulse length
        ElapsedTime = StopTime - StartTime

        # Distance travelled by the pulse in that time in cm
        Distance = (ElapsedTime * 34326)/2.0

        dist.append(Distance)

    return np.mean(dist) 
    
def isnearobstacle(localhownear):
    distance = measure()

    print('Is near obstacle: ' + str(distance))
    if distance < localhownear:
        return True
    else:
        return False

def avoidobstacle(left=False):
    backward() 
    time.sleep(reversetime)
    stopmotors()

    if left:
        turnright()
        left = False
    else:
        turnleft()
        left = True
    time.sleep(turntime)
    stopmotors()
    return left

def main():
    hownear = 40.0
    try:
        GPIO.output(pinTrigger, False)

        # Allow module to settle
        time.sleep(0.1)
        left = False
        while True:
            forward()
            time.sleep(0.1)
            if isnearobstacle(hownear):
                stopmotors()
                left = avoidobstacle()
    except KeyboardInterrupt:
        GPIO.cleanup()
    


############
if __name__ == '__main__':
    main_ao()
        
