# motor control code


# load the libraries
import RPi.GPIO as GPIO
import time
import numpy as np
import sys



# set variables for GPIO  Pins
pinMotorAForward = 9
pinMotorABackward = 10
pinMotorBForward = 8
pinMotorBBackward = 7






def initialize_motors(frequency=50):
    #set GPIO modes
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # Set the GPIO Pin Mode
    GPIO.setup(pinMotorAForward, GPIO.OUT)
    GPIO.setup(pinMotorABackward, GPIO.OUT)
    GPIO.setup(pinMotorBForward, GPIO.OUT)
    GPIO.setup(pinMotorBBackward, GPIO.OUT)

    global pwmMotorAForward,\
       pwmMotorBForward,\
       pwmMotorABackward,\
       pwmMotorBBackward

    # Set the GPIO to Software PWM at 'Frequency' Hertz
    pwmMotorAForward = GPIO.PWM(pinMotorAForward, frequency)
    pwmMotorABackward = GPIO.PWM(pinMotorABackward, frequency)
    pwmMotorBForward = GPIO.PWM(pinMotorBForward, frequency)
    pwmMotorBBackward = GPIO.PWM(pinMotorBBackward, frequency)

    # set the duty cycle for software PWM - initially to 0
    pwmMotorAForward.start(0)
    pwmMotorABackward.start(0)
    pwmMotorBForward.start(0)
    pwmMotorBBackward.start(0)

def stopmotors():
    pwmMotorAForward.ChangeDutyCycle(0)
    pwmMotorABackward.ChangeDutyCycle(0)
    pwmMotorBForward.ChangeDutyCycle(0)
    pwmMotorBBackward.ChangeDutyCycle(0)

def forward(speed=50):
    print('Moving Forward')
    pwmMotorAForward.ChangeDutyCycle(speed)
    pwmMotorABackward.ChangeDutyCycle(0)
    pwmMotorBForward.ChangeDutyCycle(speed)
    pwmMotorBBackward.ChangeDutyCycle(0)

def backward(speed=50):
    print('Moving backward')
    pwmMotorAForward.ChangeDutyCycle(0)
    pwmMotorABackward.ChangeDutyCycle(speed)
    pwmMotorBForward.ChangeDutyCycle(0)
    pwmMotorBBackward.ChangeDutyCycle(speed)

def turnleft(speed=50):
    print('Turning Left')
    pwmMotorAForward.ChangeDutyCycle(0)
    pwmMotorABackward.ChangeDutyCycle(speed)
    pwmMotorBForward.ChangeDutyCycle(speed)
    pwmMotorBBackward.ChangeDutyCycle(0)


def turnright(speed=50):
    print('Turning Right')
    pwmMotorAForward.ChangeDutyCycle(speed)
    pwmMotorABackward.ChangeDutyCycle(0)
    pwmMotorBForward.ChangeDutyCycle(0)
    pwmMotorBBackward.ChangeDutyCycle(speed)

        
    

if __name__ == '__main__':
    initialize_motors()
    stopmotors()

    print('Controlling using keyboard')
    while True:
        try:
            forward()
        except KeyboardInterrupt:
            GPIO.cleanup()
    


