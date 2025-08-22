"""
Detecting faces using media pipe

"""
import numpy as np
import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from utils import visualize
import time

# Choose between Picam or USB Camera
USBCAM = True


# Step 1: Initialize camera

if USBCAM: 
    capture = cv2.VideoCapture(cv2.CAP_V4L2)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
else:
    from picamera2 import Picamera2
    from libcamera import Transform

    picam2 = Picamera2()
    picam2.preview_configuration.size=(640,480)
    picam2.preview_configuration.format = "RGB888"
    picam2.preview_configuration.transform = Transform(hflip=1, vflip=0)
    picam2.start()

# Step 2: Create an Object Detector 
BaseOptions = mp.tasks.BaseOptions
ObjectDetector = mp.tasks.vision.ObjectDetector
ObjectDetectorOptions = mp.tasks.vision.ObjectDetectorOptions
VisionRunningMode = mp.tasks.vision.RunningMode

options = ObjectDetectorOptions(
    base_options=BaseOptions(model_asset_path='./object_detector.tflite'),
    max_results=5,
    running_mode=VisionRunningMode.VIDEO)

with ObjectDetector.create_from_options(options) as detector:
    # The detector is initialized. Use it here.
    while True:
        if USBCAM:
            result, image = capture.read() # read frames from camera
            if result is False:
                break
        else: # picam2
            image = picam2.capture_array()

                # convert image from BGR to RGB
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # convert into mediapipe compatible format
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)

        # get current time in milliseconds
        frame_timestamp_ms = int(time.time() * 1000)

        # Run the detector
        detection_result = detector.detect_for_video(mp_image, frame_timestamp_ms)

        # process the result
        image_copy = np.copy(mp_image.numpy_view())
        annotated_image = visualize(image_copy, detection_result)
        rgb_annotated_image = cv2.cvtColor(annotated_image, cv2.COLOR_BGR2RGB)

        # visualize image
        cv2.imshow("Object Detection", rgb_annotated_image)

        # press q to exist
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

if USBCAM:
    capture.release()
else:
    picam2.stop()
cv2.destroyAllWindows()

        
