# CamJam Edukit 3 - Robots
# Worksheet 9 - Obstacle Avoidance


# load the libraries
import RPi.GPIO as GPIO
import time
import numpy as np
import motor_control as mc



# set variables for GPIO  Pins
pinTrigger = 17
pinEcho = 18

# Distance variables
hownear = 40.0
reversetime = 0.5
turntime = 0.75

def initialize_sensors():
    #set GPIO modes
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    # Enable Ultrasonic sensors
    GPIO.setup(pinTrigger, GPIO.OUT)
    GPIO.setup(pinEcho, GPIO.IN)
    GPIO.output(pinTrigger, False)
    time.sleep(0.1)

# global variable needed for obstacle avoidance
left = False 

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
    
def nearobstacle(localhownear):
    distance = measure()
    #print('Is near obstacle: ' + str(distance))
    if distance < localhownear:
        return True
    else:
        return False

def avoidobstacle():
    global left
    mc.backward() 
    time.sleep(reversetime)
    mc.stopmotors()
    if left:
        mc.turnright()
        left = False
    else:
        mc.turnleft()
        left = True
    time.sleep(turntime)
    mc.stopmotors()



def halt():
    mc.stopmotors()

def initialize():
    mc.initialize_motors()
    initialize_sensors()

def cleanup():
    GPIO.cleanup()
    
def main():
    if nearobstacle(hownear):
        mc.stopmotors()
        avoidobstacle()
        msg = 'Turning to avoid obstacle'
        return msg
    else:
        mc.forward()
        time.sleep(0.1)
        msg = 'Moving forward'
        return msg


############
if __name__ == '__main__':
    try:
        GPIO.output(pinTrigger, False)
        # Allow module to settle
        time.sleep(0.1)
        left = False
        while True:
            forward()
            time.sleep(0.1)
            if nearobstacle(hownear):
                stopmotors()
                left = avoidobstacle(left)
    except KeyboardInterrupt:
        GPIO.cleanup()
    
        
