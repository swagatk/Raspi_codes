# motor control code


# load the libraries
import RPi.GPIO as GPIO
import time
import numpy as np
import key_press as kp

#set GPIO modes
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


# set variables for GPIO  Pins
pinMotorAForward = 9
pinMotorABackward = 10
pinMotorBForward = 8
pinMotorBBackward = 7



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


def key_control():

    if kp.getKey('LEFT'):
        turnleft()

    elif kp.getKey('RIGHT'):
        turnright()
        
    elif kp.getKey('UP'):
        forward()
        
    elif kp.getKey('DOWN'):
        backward()

    elif kp.getKey('ESCAPE'):
        stopmotors()
        kp.stop()
        GPIO.cleanup()

    else:
        stopmotors()
        

def exec_cmd(cmd_str):
    if cmd_str == 'LEFT':
        turnleft()
        print('Turning Left')
    elif cmd_str == 'RIGHT':
        turnright()
        print('Turning Right')
    elif cmd_str == 'UP':
        forward()
        print('Moving Forward')
    elif cmd_str == 'DOWN':
        backward()
        print('Moving Downward')
    else:
        stopmotors()
        
    

if __name__ == '__main__':
    stopmotors()
    kp.init()
    print('Controlling using keyboard')
    while True:
        try:
            key_control()
        except KeyboardInterrupt:
            GPIO.cleanup()
    


