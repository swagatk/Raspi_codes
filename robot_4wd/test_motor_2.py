import RPi.GPIO as gpio
import motor_control_4wd as mc
import time

gpio.setmode(gpio.BOARD)
gpio.setwarnings(False)


if __name__ == '__main__':
    mc.initialize_motors()

    try:
        while True:
            mc.move_forward()
            time.sleep(1)
            mc.move_backward()
            time.sleep(1)
            mc.stop()
    except KeyboardInterrupt:
        gpio.cleanup()
            
