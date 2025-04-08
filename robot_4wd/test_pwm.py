import PiMotor
import RPi.GPIO as GPIO                        #Import GPIO library
import time
from time import sleep
GPIO.setmode(GPIO.BOARD)                       #Set GPIO pin numbering

GPIO.setwarnings(False)


for i in range(3):
    GPIO.setmode(GPIO.BOARD)                       #Set GPIO pin numbering
    GPIO.setwarnings(False)
    print('iteration:', i)    
    m1 = PiMotor.Motor("MOTOR1", 2)
    m1.forward(100)
    time.sleep(1)
    m1.stop()
    m1.stop_pwm()
    del m1
    GPIO.cleanup()
    time.sleep(1)
