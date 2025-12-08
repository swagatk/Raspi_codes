from picamera2 import Picamera2, Preview
import time
from libcamera import controls
from pprint import * 
picam2 = Picamera2()
camera_config = picam2.create_preview_configuration()
picam2.configure(camera_config)
# pprint(picam2.camera_controls)
# pprint(camera_config)
# pprint(picam2.camera_properties)
picam2.start_preview(Preview.QTGL)

picam2.start()

time.sleep(5)
# capture an image
picam2.capture_file("test.jpg")

picam2.stop_preview()

picam2.stop()
picam2.close()

print("Image captured and camera resources released.")
