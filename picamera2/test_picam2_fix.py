from picamera2 import Picamera2, Preview
import time
from libcamera import controls
from pprint import * 
picam2 = Picamera2()
camera_config = picam2.create_preview_configuration()
picam2.configure(camera_config)
##pprint(picam2.camera_controls)
##pprint(camera_config)
##pprint(picam2.camera_properties)
picam2.start_preview(Preview.QTGL)

picam2.start()
size = picam2.capture_metadata()['ScalerCrop'][2:]
full_res = picam2.camera_properties['PixelArraySize']

for _ in range(20):
    # This syncs us to the arrival of a new camera frame:
    picam2.capture_metadata()
    size = [int(s * 2) for s in size]
    offset = [(r - s) // 2 for r, s in zip(full_res, size)]
    picam2.set_controls({"ScalerCrop": offset + size})
    time.sleep(2)
    
picam2.stop_preview()
picam2.capture_file("test.jpg")
picam2.stop(Preview.QTGL)
picam2.close()
