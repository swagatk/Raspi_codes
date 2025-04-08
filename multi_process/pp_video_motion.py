import RPi.GPIO as GPIO
import time
import cv2
from picamera2 import Picamera2
from multiprocessing import Process, Value, Lock
import importlib.util
import sys
import avoid_obstacle as ao

def view_camera():
    print('Camera Process starts')
    try:
        picam = Picamera2()
        config = picam.create_preview_configuration(
            main = {"format":"XRGB8888",
                    "size":(320,240)})
        picam.configure(config)
        picam.start()
        while True:
            img = picam.capture_array()
            cv2.imshow("Camera", img)
            if cv2.waitKey(1) == ord('q'):
                break
    except Exception as e:
        print(f'view camera errors:{e}')
    finally:
        picam.stop()
        cv2.destroyAllWindows()



def robot_motion():
    print('Robot Motion Process starts ...')
    try:
        ao.initialize()
        
        while True:
            ao.main()

    except Exception as e:
        print(f'motion errors: {e}')
        ao.halt()
    finally:
        GPIO.cleanup()
            

if __name__ == '__main__':
    print('Main process')

    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        cam_proc = Process(target=view_camera, args=(""))
        mot_proc = Process(target=robot_motion, args=(""))

        cam_proc.start()
        mot_proc.start()
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print('Keyboard Interrupt .. stopping')
        # wait for processes to join
        cam_proc.join()
        mot_proc.join()
    finally:
        GPIO.cleanup()
        
        
    
