import cv2
from picamera2 import Picamera2
import time
from libcamera import Transform


picam2 = Picamera2()
picam2.preview_configuration.size=(320,240)
picam2.preview_configuration.format = "RGB888"
picam2.preview_configuration.transform = Transform(hflip=1, vflip=0)
picam2.start()


while True:
    img = picam2.capture_array()
    cv2.imshow("preview", img)
    if cv2.waitKey(1)==ord('q'):
        break
picam2.stop()
cv2.destroyAllWindows()
picam2.close()
