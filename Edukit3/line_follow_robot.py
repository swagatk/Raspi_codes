# CamJam Edukit 3 - Robotics
# Worksheet 8 - Line Following Robot

# load the libraries
import RPi.GPIO as GPIO
import time

#set GPIO modes
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


# set variables for GPIO  Pins
pinMotorAForward = 10
pinMotorABackward = 9
pinMotorBForward = 8
pinMotorBBackward = 7
pinLineFollower = 25  




# PWM parameters
Frequency = 20
DutyCycleA = 30
DutyCycleB = 30
Stop = 0



# Set the GPIO Pin Mode
GPIO.setup(pinMotorAForward, GPIO.OUT)
GPIO.setup(pinMotorABackward, GPIO.OUT)
GPIO.setup(pinMotorBForward, GPIO.OUT)
GPIO.setup(pinMotorBBackward, GPIO.OUT)
GPIO.setup(pinLineFollower, GPIO.IN)


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
    pwmMotorAForward.ChangeDutyCycle(DutyCycleA)
    pwmMotorABackward.ChangeDutyCycle(Stop)
    pwmMotorBForward.ChangeDutyCycle(DutyCycleB)
    pwmMotorBBackward.ChangeDutyCycle(Stop)

def backward():
    pwmMotorAForward.ChangeDutyCycle(Stop)
    pwmMotorABackward.ChangeDutyCycle(DutyCycleA)
    pwmMotorBForward.ChangeDutyCycle(Stop)
    pwmMotorBBackward.ChangeDutyCycle(DutyCycleB)

def turnleft():
    pwmMotorAForward.ChangeDutyCycle(Stop)
    pwmMotorABackward.ChangeDutyCycle(DutyCycleA)
    pwmMotorBForward.ChangeDutyCycle(DutyCycleB)
    pwmMotorBBackward.ChangeDutyCycle(Stop)


def turnright():
    pwmMotorAForward.ChangeDutyCycle(DutyCycleA)
    pwmMotorABackward.ChangeDutyCycle(Stop)
    pwmMotorBForward.ChangeDutyCycle(Stop)
    pwmMotorBBackward.ChangeDutyCycle(DutyCycleB)
    

def isoverblack():
    if GPIO.input(pinLineFollower) == 0:
        return True
    else:
        return False


def seekline():
    ''' search for a black line'''

    # direct the robot will turn: True for left
    direction = True

    seeksize = 0.25
    seekcount = 1
    maxseekcount = 5

    # turn the robot left and right until it finds the line
    # or if we have looked enough

    while seekcount <= maxseekcount:

        #set the seek time
        seektime = seeksize * seekcount

        # start the motors turning in a direction
        if direction:
            print('Looking Left')
            turnleft()
        else:
            print('Looking Right')
            turnright()

        starttime = time.time()
        while time.time() - starttime <= seektime:
            if isoverblack():
                stopmotors()
                return True
        # robot has not found the black line yet, so stop
        stopmotors()

        # increase the seek count
        seekcount += 1

        # change direction
        direction = not direction

    # The line was not found, so return False
    return False

#################

try:
    print('Following the line')
    while True:
        if isoverblack():
            forward()
        else:
            stopmotors()
            if seekline():
                print('Following the line')
            else:
                stopmotors()
                print('Robot has lost the line')
                exit()
except KeyboardInterrupt:
    GPIO.cleanup()
        
