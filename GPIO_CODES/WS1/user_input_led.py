# CamJam EduKit 1 - Basics
# Worksheet 3 - Blinking LED

# Import libraries
import time
import os
import sys
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(18, GPIO.OUT)
GPIO.setup(23, GPIO.OUT)
GPIO.setup(24, GPIO.OUT)

# set up variables
LEDRed = 18
LEDYellow = 23
LEDGreen = 24

# set LED Pins to OUTPUT
GPIO.setup(LEDRed, GPIO.OUT)
GPIO.setup(LEDYellow, GPIO.OUT)
GPIO.setup(LEDGreen, GPIO.OUT)

# Initially set them to Low values
GPIO.output(LEDRed, GPIO.LOW)
GPIO.output(LEDYellow, GPIO.LOW)
GPIO.output(LEDGreen, GPIO.LOW)

# clear the terminal
os.system('clear')

# Ask Question
led_choice = input('Which LED to blink?(Red-1, Yellow-2, Green-3):')
led_choice = int(led_choice)
count = input('How many times to blink?')
count = int(count)

if led_choice == 1:
    LEDChoice = LEDRed
elif led_choice == 2:
    LEDChoice = LEDYellow
elif led_choice == 3:
    LEDChoice = LEDGreen
else:
    print('Invalid Choice for LED. Exiting ...')
    sys.exit()

if LEDChoice > 0:
    while count > 0:
        GPIO.output(LEDChoice, GPIO.HIGH)
        time.sleep(1)
        GPIO.output(LEDChoice, GPIO.LOW)
        time.sleep(1)
        print('Count: ', count)
        count = count - 1

GPIO.cleanup()
