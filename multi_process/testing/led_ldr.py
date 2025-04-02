import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


pin_led = 14

pin_ldr = 14

def read_ldr():
    ldrcount = 0
    GPIO.setup(pin_ldr, GPIO.OUT)
    GPIO.output(pin_ldr, GPIO.LOW)
    time.sleep(0.1)
    GPIO.setup(pin_ldr, GPIO.IN)
    while (GPIO.input(pin_ldr) == GPIO.LOW):
        ldrcount += 1
    return ldrcount

def blink_led():
    GPIO.setup(pin_led, GPIO.OUT)
    GPIO.output(pin_led, GPIO.LOW)
    time.sleep(0.5)
    GPIO.output(pin_led, GPIO.HIGH)
    time.sleep(0.5)

if __name__ == '__main__':
    
    while True:
        try:
            ldr_value = read_ldr()
            print('ldr_value:', ldr_value)
            blink_led()
        except KeyboardInterrupt:
            print('Exiting')
            break
    # exit loop
    GPIO.cleanup()
