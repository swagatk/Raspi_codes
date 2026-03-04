import cv2
from picamera2 import Picamera2
from picamera2 import Preview
from libcamera import Transform
from ultralytics import YOLO

# Load YOLO model
model = YOLO("/home/pi/yolo_project/orange_ball.pt")

# Initialize camera
picam2 = Picamera2()

# Configure preview stream
picam2.preview_configuration.main.size = (320, 240)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.transform = Transform(hflip=1, vflip=0)

picam2.configure("preview")
picam2.start()

print("YOLO Object Detection started. Press 'q' to quit.")

while True:
    # Capture frame
    frame = picam2.capture_array()

    # PiCamera v3 with libcamera outputs BGR despite "RGB888" format
    # No conversion needed - frame is already in BGR format for OpenCV

    # Run YOLO
    results = model(frame, verbose=False)

    # Draw detections
    annotated_frame = results[0].plot()

    # Show output
    cv2.imshow("YOLO Detection", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()