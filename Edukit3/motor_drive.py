# cam Jam Edukit 3 - Robotics
# Driving and turning
import RPi.GPIO as GPIO
import time

#set GPIO modes
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# set variables for GPIO motor Pins
pinMotorAForward = 9
pinMotorABackward = 10
pinMotorBForward = 8
pinMotorBBackward = 7

# Set the GPIO Pin Mode
GPIO.setup(pinMotorAForward, GPIO.OUT)
GPIO.setup(pinMotorABackward, GPIO.OUT)
GPIO.setup(pinMotorBForward, GPIO.OUT)
GPIO.setup(pinMotorBBackward, GPIO.OUT)

# Turn off all the motors
def stopmotors():
    GPIO.output(pinMotorAForward, 0)
    GPIO.output(pinMotorABackward, 0)
    GPIO.output(pinMotorBForward, 0)
    GPIO.output(pinMotorBBackward, 0)


def forward():
    GPIO.output(pinMotorAForward, 1)
    GPIO.output(pinMotorABackward, 0)
    GPIO.output(pinMotorBForward, 1)
    GPIO.output(pinMotorBBackward, 0)


def backward():
    GPIO.output(pinMotorAForward, 0)
    GPIO.output(pinMotorABackward, 1)
    GPIO.output(pinMotorBForward, 0)
    GPIO.output(pinMotorBBackward, 1)


def turnleft():
    GPIO.output(pinMotorAForward, 0)
    GPIO.output(pinMotorABackward, 1)
    GPIO.output(pinMotorBForward, 1)
    GPIO.output(pinMotorBBackward, 0)

def turnright():
    GPIO.output(pinMotorAForward, 1)
    GPIO.output(pinMotorABackward, 0)
    GPIO.output(pinMotorBForward, 0)
    GPIO.output(pinMotorBBackward, 1)


####
forward()
time.sleep(1)
backward()
time.sleep(1)
turnleft()
time.sleep(1)
turnright()
time.sleep(1)
stopmotors()

GPIO.cleanup()


