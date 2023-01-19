from picamera import PiCamera
from time import sleep

camera = PiCamera()
camera.rotation = 180
camera.resolution = (320, 320)
camera.framerate = 25
camera.awb_mode = 'auto'
camera.exposure_mode = 'night'
camera.start_preview()
sleep(5)
camera.stop_preview()