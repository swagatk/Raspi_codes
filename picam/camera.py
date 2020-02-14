# never save the file as "picamera.py"

from picamera import PiCamera, Color
from time import sleep

# camera configuration
camera = PiCamera()
camera.rotation = 180
camera.resolution = (320,240)
camera.framerate = 15
#camera.brightness = 50   # default is 50, max is 100

### save single image
##camera.start_preview()
##sleep(5)
##camera.capture('/home/pi/smallimg.jpg')
##camera.stop_preview()
##
### see through images
##camera.start_preview(alpha=200) # see through video (0-255)
##sleep(5)
##camera.stop_preview()
##
### save multiple images
##camera.start_preview()
##for i in range(5):
##    sleep(5)
##    camera.capture('/home/pi/image%s.jpg' % i)
##camera.stop_preview()
##
##
### save video
##camera.start_preview()
##camera.start_recording('/home/pi/video.h264')
##    sleep(5)
##    camera.stop_recording()
##camera.stop_preview()

##
###Annotate text
##camera.start_preview()
##camera.annotate_text_size = 18
##camera.annotate_background = Color('blue')
##camera.annotate_foreground = Color('yellow')
##camera.annotate_text = 'Swagat\'s Camera'
##sleep(5)
##camera.capture('/home/pi/imgtxt.jpg')
##camera.stop_preview()
##
##
### Changing camera brightness
##camera.start_preview()
##for i in range(100):
##    camera.annotate_text = "Brightness: %s" %i
##    camera.brightness = i
##    sleep(0.1)
##camera.stop_preview()

##
### Changing camera Contrast
##camera.start_preview()
##for i in range(100):
##    camera.annotate_text = "Contrast: %s" %i
##    camera.contrast = i
##    sleep(0.1)
##camera.stop_preview()

# Image effects
camera.start_preview()
camera.image_effect = 'colorswap'
sleep(5)
camera.capture('/home/pi/colorswap.jpg')
camera.stop_preview()

# loop through all image effects
camera.start_preview()
for effect in camera.IMAGE_EFFECTS:
    camera.image_effect = effect
    camera.annotate_text = "Effect: %s" % effect
    sleep(2)
camera.stop_preview()
sleep(5)
camera.capture('/home/pi/colorswap.jpg')
camera.stop_preview()


# loop through all exposures
camera.start_preview()
for mode in camera.EXPOSURE_MODES:
    camera.exposure_mode = mode
    camera.annotate_text = "Mode: %s" % mode
    sleep(2)
camera.stop_preview()
sleep(5)
#camera.capture('/home/pi/colorswap.jpg')
camera.stop_preview()

# white balance
camera.start_preview()
for mode in camera.AWB_MODES:
    camera.awb_mode = mode
    camera.annotate_text_size = 12
    camera.annotate_text = "AWB Mode: %s" % mode
    sleep(2)
camera.stop_preview()
sleep(5)
#camera.capture('/home/pi/colorswap.jpg')
camera.stop_preview()
