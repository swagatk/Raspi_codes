# CamJam EduKit 1 - Basics
# Worksheet 3 - Blinking LED

# Import libraries
import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(18, GPIO.OUT)
GPIO.setup(23, GPIO.OUT)
GPIO.setup(24, GPIO.OUT)


count = 0

while count < 10: 

    GPIO.output(18, GPIO.HIGH)
    GPIO.output(23, GPIO.HIGH)
    GPIO.output(24, GPIO.HIGH)

    time.sleep(1)


    GPIO.output(18, GPIO.LOW)
    GPIO.output(23, GPIO.LOW)
    GPIO.output(24, GPIO.LOW)

    time.sleep(1)

    count += 1

    print("count: ", count)

GPIO.cleanup()
