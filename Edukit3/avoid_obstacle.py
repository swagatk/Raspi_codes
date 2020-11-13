# CamJam Edukit 3 - Robots
# Worksheet 9 - Obstacle Avoidance


# load the libraries
import RPi.GPIO as GPIO
import time
import numpy as np

#set GPIO modes
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


# set variables for GPIO  Pins
pinMotorAForward = 9
pinMotorABackward = 10
pinMotorBForward = 7
pinMotorBBackward = 8
pinTrigger = 17
pinEcho = 18



# PWM parameters
Frequency = 20
DutyCycleA = 70
DutyCycleB = 70
Stop = 0

# Set the GPIO Pin Mode
GPIO.setup(pinMotorAForward, GPIO.OUT)
GPIO.setup(pinMotorABackward, GPIO.OUT)
GPIO.setup(pinMotorBForward, GPIO.OUT)
GPIO.setup(pinMotorBBackward, GPIO.OUT)
GPIO.setup(pinTrigger, GPIO.OUT)
GPIO.setup(pinEcho, GPIO.IN)

# Distance variables
hownear = 20.0
reversetime = 0.5
turntime = 0.75


# Set the GPIO to Software PWM at 'Frequency' Hertz
pwmMotorAForward = GPIO.PWM(pinMotorAForward, Frequency)
pwmMotorABackward = GPIO.PWM(pinMotorABackward, Frequency)
pwmMotorBForward = GPIO.PWM(pinMotorBForward, Frequency)
pwmMotorBBackward = GPIO.PWM(pinMotorBBackward, Frequency)

# set the duty cycle for software PWM - initially to 0
pwmMotorAForward.start(Stop)
pwmMotorABackward.start(Stop)
pwmMotorBForward.start(Stop)
pwmMotorBBackward.start(Stop)

def stopmotors():
    pwmMotorAForward.ChangeDutyCycle(Stop)
    pwmMotorABackward.ChangeDutyCycle(Stop)
    pwmMotorBForward.ChangeDutyCycle(Stop)
    pwmMotorBBackward.ChangeDutyCycle(Stop)

def forward():
    print('Moving Forward')
    pwmMotorAForward.ChangeDutyCycle(DutyCycleA)
    pwmMotorABackward.ChangeDutyCycle(Stop)
    pwmMotorBForward.ChangeDutyCycle(DutyCycleB)
    pwmMotorBBackward.ChangeDutyCycle(Stop)

def backward():
    print('Moving backward')
    pwmMotorAForward.ChangeDutyCycle(Stop)
    pwmMotorABackward.ChangeDutyCycle(DutyCycleA)
    pwmMotorBForward.ChangeDutyCycle(Stop)
    pwmMotorBBackward.ChangeDutyCycle(DutyCycleB)

def turnleft():
    print('Turning Left')
    pwmMotorAForward.ChangeDutyCycle(Stop)
    pwmMotorABackward.ChangeDutyCycle(DutyCycleA)
    pwmMotorBForward.ChangeDutyCycle(DutyCycleB)
    pwmMotorBBackward.ChangeDutyCycle(Stop)


def turnright():
    print('Turning Right')
    pwmMotorAForward.ChangeDutyCycle(DutyCycleA)
    pwmMotorABackward.ChangeDutyCycle(Stop)
    pwmMotorBForward.ChangeDutyCycle(Stop)
    pwmMotorBBackward.ChangeDutyCycle(DutyCycleB)


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

def avoidobstacle():
    backward() 
    time.sleep(reversetime)
    stopmotors()

    turnright()
    time.sleep(turntime)
    stopmotors()

############
if __name__ == '__main__':
    try:
        GPIO.output(pinTrigger, False)

        # Allow module to settle
        time.sleep(0.1)

        while True:
            forward()
            time.sleep(0.1)
            if isnearobstacle(hownear):
                stopmotors()
                avoidobstacle()
    except KeyboardInterrupt:
        GPIO.cleanup()
    
        
