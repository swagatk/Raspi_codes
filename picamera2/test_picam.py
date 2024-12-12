from picamera2 import Picamera2, Preview
import time
picam2 = Picamera2()
camera_config = picam2.create_preview_configuration()
picam2.configure(camera_config)
picam2.camera_properties
##picam2.start_preview(Preview.QT)
##picam2.start()
##time.sleep(2)
##picam2.capture_file("test.jpg")
##picam2.start_and_capture_file("test.jpg")
###picam2.start_and_record_video("test.mp4", duration=5)
