import cv2
from picamera2 import Picamera2
from ultralytics import YOLO
import time

# --- Camera and Model Setup ---

# Initialize Picamera2
picam2 = Picamera2()

# Configure the camera for preview
# We use a 640x480 resolution for a good balance of speed and accuracy
# We set the format to "RGB888" which is what OpenCV and YOLO expect
config = picam2.create_preview_configuration(
    main={"size": (320, 240), "format": "RGB888"}
)
picam2.configure(config)

# Start the camera
picam2.start()

# Load the YOLOv8 "nano" model (yolov8n.pt)
# This is the fastest model, ideal for Raspberry Pi
model = YOLO("yolov8n.pt")

# --- Real-time Detection Loop ---

print("Starting real-time object detection... Press 'q' to quit.")
prev_time = time.time()
while True:
    # 1. Capture a frame from the camera
    # This frame is a NumPy array in RGB format
    frame = picam2.capture_array()

    # 2. Run YOLOv8 inference on the frame
    # verbose=False reduces console spam
    results = model.predict(frame, imgsz=320, verbose=False)

    # 3. Get the annotated frame
    # results[0].plot() draws the bounding boxes, labels, and confidences
    # on a copy of the original frame
    annotated_frame = results[0].plot()

    # --- 4. Calculate and Display FPS ---
    current_time = time.time()
    fps = 1 / (current_time - prev_time)
    prev_time = current_time

    # Draw the FPS text on the frame
    cv2.putText(annotated_frame, f"FPS: {int(fps)}", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    # 5. Display the annotated frame
    # We use cv2.imshow to display the window
    # Note: YOLO's .plot() returns an RGB image, but cv2.imshow expects BGR.
    # We must convert the color format before displaying.
    cv2.imshow("YOLOv8 Live Detection", cv2.cvtColor(annotated_frame, cv2.COLOR_RGB2BGR))

    # 5. Check for 'q' key press to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# --- Cleanup ---
print("Cleaning up and exiting...")
cv2.destroyAllWindows()
picam2.stop()
