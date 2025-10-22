import cv2
from picamera2 import Picamera2
from ultralytics import YOLO
import time
import math

# ------- configuration ----
SKIP_FRAMES = 3  # Run detection 1 out of every 3 frames
INFERENCE_SIZE = 320 # Resolution for YOLO inference
WEBCAM_WIDTH = 320   # Webcam capture width
WEBCAM_HEIGHT = 240  # Webcam capture height

# --- Camera and Model Setup --- 

# Initialize Picamera2
picam2 = Picamera2()

# Configure the camera for preview
# We use a 640x480 resolution for a good balance of speed and accuracy
# We set the format to "RGB888" which is what OpenCV and YOLO expect
config = picam2.create_preview_configuration(
    main={"size": (WEBCAM_WIDTH, WEBCAM_HEIGHT), "format": "RGB888"}
)
picam2.configure(config)

# Start the camera
picam2.start()

# Load the YOLOv8 "nano" model (yolov8n.pt)
# This is the fastest model, ideal for Raspberry Pi
model = YOLO("yolov8n.pt")

# --- Variables for Loop ---
prev_time = 0
frame_counter = 0
last_results = None  # Store the last detection results


# --- Real-time Detection Loop ---

print("Starting real-time object detection... Press 'q' to quit.")

while True:
    # 1. Capture a frame from the camera
    # This frame is a NumPy array in RGB format
    frame = picam2.capture_array()

    # Create a copy of the frame to draw on
    annotated_frame = frame.copy()
    frame_counter += 1

    # 2. Run YOLOv8 inference on the frame
    # verbose=False reduces console spam
    if frame_counter % SKIP_FRAMES == 0:
        # RUN Yolov8 inference
        if INFERENCE_SIZE is None:
            results = model.predict(frame, verbose=False)
        else:
            results = model.predict(frame, imgsz=INFERENCE_SIZE, verbose=False)
        last_results = results

    # --- Drawing Logic ---
    # We must *always* draw, even on skipped frames.
    # We use the 'last_results' if they exist.
    if last_results:
        # Loop through each result object
        for r in last_results:
            boxes = r.boxes  # Get the Boxes object
            
            # Loop through each bounding box
            for box in boxes:
                # Get coordinates
                b = box.xyxy[0]  # [x1, y1, x2, y2]
                x1, y1, x2, y2 = map(math.floor, b) # Convert to int

                # Get class and label
                c = box.cls
                label = model.names[int(c)]
                
                # Get confidence
                conf = math.floor(box.conf[0] * 100) / 100
                label_with_conf = f"{label} {conf}"

                # Draw rectangle and label
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(annotated_frame, label_with_conf, (x1, y1 - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # --- Calculate and Display FPS ---
    current_time = time.time()
    fps = 1 / (current_time - prev_time)
    prev_time = current_time

    cv2.putText(annotated_frame, f"FPS: {int(fps)}", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)        

  
    
    # 4. Display the annotated frame
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
