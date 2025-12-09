import RPi.GPIO as gpio
import motor_control_4wd as mc
import time

gpio.setmode(gpio.BOARD)
gpio.setwarnings(False)


if __name__ == '__main__':
    mc.initialize_motors()

    try:
        while True:
            mc.turn_left()
            time.sleep(1)
            
            #mc.stop()
    except KeyboardInterrupt:
        print('stopped by user')
    finally:
        mc.stop()
        mc.release_pins()
        
            
