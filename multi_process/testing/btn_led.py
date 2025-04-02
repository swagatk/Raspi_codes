import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


pin_led = 14

pin_btn = 14

def read_btn():
    GPIO.setup(pin_btn, GPIO.IN)
    time.sleep(0.1)
    if GPIO.input(pin_btn) == GPIO.LOW:
        print('Button pressed')
        time.sleep(0.1)
    

def blink_led():
    GPIO.setup(pin_led, GPIO.OUT)
    GPIO.output(pin_led, GPIO.LOW)
    time.sleep(0.5)
    GPIO.output(pin_led, GPIO.HIGH)
    time.sleep(0.5)

if __name__ == '__main__':
    
    while True:
        try:
            read_btn()
            blink_led()
        except KeyboardInterrupt:
            print('Exiting')
            break
    # exit loop
    GPIO.cleanup()
