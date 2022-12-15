# never save the file as "picamera.py"

from picamera import PiCamera, Color
from time import sleep

# choices (only make one true)
SINGLE_IMG = True   # save single image
MULTI_IMG = False   # save multiple images
TRP_VID = False     # Transparent video
SAVE_VID = False    # save video
ANNOT_TXT = False   # text annotations
CHG_BRGHT = False   # change brightness
CHG_CNTRST = False  # change contrast
IMG_EFF = False     # image effects
IMG_EXP = False     # image exposure
WHT_BAL = False     # white balance

# camera configuration
camera = PiCamera()
camera.rotation = 180
camera.resolution = (320,240)
camera.framerate = 15
#camera.brightness = 50   # default is 50, max is 100

if SINGLE_IMG:
    # save single image
    camera.start_preview()
    sleep(15)
    camera.capture('/home/pi/smallimg.jpg')
    camera.stop_preview()

if TRP_VID:
    # see through images
    camera.start_preview(alpha=200) # see through video (0-255)
    sleep(5)
    camera.stop_preview()
##
if MULTI_IMG:
    # save multiple images
    camera.start_preview()
    for i in range(5):
        sleep(5)
        camera.capture('/home/pi/image%s.jpg' % i)
    camera.stop_preview()

if SAVE_VID:
    # save video
    camera.start_preview()
    camera.start_recording('/home/pi/video.h264')
    sleep(5)
    camera.stop_recording()
    camera.stop_preview()

if ANNOT_TXT:
    #Annotate text
    camera.start_preview()
    camera.annotate_text_size = 18
    camera.annotate_background = Color('blue')
    camera.annotate_foreground = Color('yellow')
    camera.annotate_text = 'Swagat\'s Camera'
    sleep(5)
    camera.capture('/home/pi/imgtxt.jpg')
    camera.stop_preview()

if CHG_BRGHT:
    # Changing camera brightness
    camera.start_preview()
    for i in range(100):
        camera.annotate_text = "Brightness: %s" %i
        camera.brightness = i
        sleep(0.1)
    camera.stop_preview()

if CHG_CNTRST:
    # Changing camera Contrast
    camera.start_preview()
    for i in range(100):
        camera.annotate_text = "Contrast: %s" %i
        camera.contrast = i
        sleep(0.1)
    camera.stop_preview()

# Image effects
##camera.start_preview()
##camera.image_effect = 'colorswap'
##sleep(5)
##camera.capture('/home/pi/colorswap.jpg')
##camera.stop_preview()

if IMG_EFF:
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

if IMG_EXP:
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

if WHT_BAL:
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
