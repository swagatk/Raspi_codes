"""
In this code we are trying to access the same GPIO pin in two parallel processes.
The resource conflict is avoided by using shared variable "start".
Other, it gives "GPIO busy" error when the same resource is used by the two processes at the same time.
This is not very useful though but a good work around to allow to processes access the same GPIO pin,
but not simultaneously. 
"""
import RPi.GPIO as GPIO
import time
from multiprocessing import Process, Value, Lock
import sys

shared_pin = 14

def output_process(pin, running, lock, start):
    """ Process to handle output"""
    try:
        print('Output process starts ...')
        GPIO.setmode(GPIO.BCM)    
        while running.value:
            if start.value: # true
                with lock:
                    GPIO.setup(pin, GPIO.OUT)
                    GPIO.output(pin, GPIO.HIGH)
                time.sleep(0.5)
                with lock:
                    GPIO.setup(pin, GPIO.OUT)
                    GPIO.output(pin, GPIO.LOW)
                    if GPIO.getmode() is not None: # this is necessary
                        GPIO.cleanup(pin)
                time.sleep(0.5)
                start.value = False
    except Exception as e:
        print(f"Output process error: {e}")
    finally:
        print("Output process terminated ...")
        with lock:
            if GPIO.getmode() is not None:
                GPIO.cleanup(pin)


def input_process(pin, running, lock, start):
    print('Input process starts ....')
    try:
        time.sleep(0.1)
        print("Press Ctrl+C to exit ...")
        while running.value:
            if not start.value: # false
                with lock:
                    GPIO.setup(pin, GPIO.IN)               
                    btn_status = GPIO.input(pin)
                    #print('Button status: ', btn_status)
                    if btn_status == GPIO.HIGH:
                        print('Button Pressed')
                        time.sleep(0.01)
                    if GPIO.getmode() is not None:
                        GPIO.cleanup(pin)
                start.value = not start.value
            time.sleep(0.01)
    except Exception as e:
        print(f"Input process error: {e}")
    finally:
        print("Input process terminated ..")
        with lock:
            if GPIO.getmode() is not None:
                GPIO.cleanup(pin)

def main():
    running = Value('b', True)
    gpio_lock = Lock()
    start = Value('b', True)
    try:
        # initialize GPIO
        GPIO.setmode(GPIO.BCM)
        
        out_proc = Process(target=output_process, args=(shared_pin, running, gpio_lock, start))
        inp_proc = Process(target=input_process, args=(shared_pin, running, gpio_lock, start))

        out_proc.start()
        inp_proc.start()
        
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print('\nShutting Down ...')
        running.value = False

        # wait for processes to finish
        out_proc.join()
        inp_proc.join()

    finally:
        if GPIO.getmode() is not None:
            GPIO.cleanup()

if __name__ == '__main__':
    main()
