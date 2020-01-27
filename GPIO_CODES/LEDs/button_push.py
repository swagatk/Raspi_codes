# CamJam EduKit 1 - Basics
# Worksheet 5 - Button


# import libraries
import os
import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


# Variables
ButtonPin = 25
LEDPin = 18

# Set as input / output
GPIO.setup(ButtonPin, GPIO.IN)
GPIO.setup(LEDPin, GPIO.OUT)

print('Button State:',GPIO.input(ButtonPin))

while True:
    if GPIO.input(ButtonPin) == False:
        print('Button Pressed')
        print('Button State:',GPIO.input(ButtonPin))
        GPIO.output(LEDPin, GPIO.HIGH)
        #time.sleep(1)
        
    else:
        GPIO.output(LEDPin, GPIO.LOW)
        os.system('clear')
        print('Waiting for you to press the button')

    time.sleep(0.2)

GPIO.cleanup()
