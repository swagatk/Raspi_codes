import cv2
from picamera2 import Picamera2
import numpy as np
import time
import sys

from pupil_apriltags import Detector

# --- AprilTag Detector setup ---
# Ensure "tagStandard41h12" matches your physical tags
detector = Detector(families='tagStandard41h12')

# ------- Camera Setup -----
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
picam = Picamera2()

config = picam.create_preview_configuration(
    main={"size": (IMAGE_WIDTH, IMAGE_HEIGHT), "format": "RGB888"}
)
picam.configure(config)
picam.start()

print("Starting AprilTag detection... Press 'q' to quit.")

try:
    prev_time = time.time()
    while True:
        frame = picam.capture_array()
        gray_image = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        
        detections = detector.detect(gray_image)
        
        # --- Visualization ---
        for detection in detections:
            # 1. Get Corners
            corners = detection.corners.astype(int)
            
            # 2. Get Tag ID
            tag_id = str(detection.tag_id)

            # 3. Get Center
            center = detection.center.astype(int)

            # --- DRAWING ---
            # Draw the bounding box
            cv2.polylines(frame, [corners], isClosed=True, color=(0, 255, 0), thickness=2)

            # Draw the Center point
            cv2.circle(frame, tuple(center), 5, (0, 0, 255), -1)

            # Draw the Tag ID above the first corner
            cv2.putText(frame, f"ID: {tag_id}", (corners[0][0], corners[0][1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        
        # Display the frame
        # --- FPS Calculation ---
        curr_time = time.time()
        fps = 1 / (curr_time - prev_time)
        prev_time = curr_time
        cv2.putText(frame, f"FPS: {int(fps)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        
        # --- Display ---
        cv2.imshow("AprilTag Detection", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except Exception as e:
    print(f'View camera errors: {e}')

finally:
    picam.stop()
    cv2.destroyAllWindows()
