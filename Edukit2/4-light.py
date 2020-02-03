# Edukit 2
# worksheet 4 - Light Sensor


import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

pinldr = 27

def readldr():
    ldrcount = 0
    GPIO.setup(pinldr, GPIO.OUT)
    GPIO.output(pinldr, GPIO.LOW)
    time.sleep(0.1)

    GPIO.setup(pinldr, GPIO.IN)
    while (GPIO.input(pinldr) == GPIO.LOW):
        ldrcount += 1
    return ldrcount

while True:
    print(readldr())
    time.sleep(1)
