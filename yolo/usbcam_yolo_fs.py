import cv2
from ultralytics import YOLO
import time
import math  # Needed for rounding box coordinates

# --- Configuration ---
SKIP_FRAMES = 3  # Run detection 1 out of every 3 frames
INFERENCE_SIZE = 320 # Resolution for YOLO inference
WEBCAM_WIDTH = 640   # Webcam capture width
WEBCAM_HEIGHT = 480  # Webcam capture height

# --- Model and Camera SetuP

# Load the YOLOv8 "nano" model
model = YOLO("yolov8n.pt")

# Open the default USB camera (usually index 0)
cap = cv2.VideoCapture(2)

# Set the desired resolution
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WEBCAM_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, WEBCAM_HEIGHT)

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open video device.")
    exit()

# --- Variables for Loop ---
prev_time = 0
frame_counter = 0
last_results = None  # Store the last detection results

print("Starting USB camera feed... Press 'q' to quit.")

# --- Real-time Detection Loop ---

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Can't receive frame. Exiting ...")
        break

    # Create a copy of the frame to draw on
    annotated_frame = frame.copy()
    frame_counter += 1

    # --- Frame Skipping Logic ---
    # Only run prediction if it's the right frame
    if frame_counter % SKIP_FRAMES == 0:
        # Run YOLOv8 inference
        if INFERENCE_SIZE is not None:
            results = model.predict(frame, imgsz=INFERENCE_SIZE, verbose=False)
        else:
            results = model.predict(frame, verbose=False)
        last_results = results  # Store the new results
        
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

    # --- Display the annotated frame ---
    cv2.imshow('YOLOv8 Live USB Cam', annotated_frame)

    # Check for 'q' key press to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# --- Cleanup ---
print("Cleaning up and exiting...")
cap.release()
cv2.destroyAllWindows()
