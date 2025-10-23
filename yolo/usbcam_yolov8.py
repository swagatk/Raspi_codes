import cv2
from ultralytics import YOLO
import time  # <--- 1. Import time library

# --- Model and Camera Setup ---

# Load the YOLOv8 "nano" model
#model = YOLO("yolov8n.pt")
model = YOLO("yolo11n.pt")


# Open the default USB camera (usually index 0)
cap = cv2.VideoCapture(0)

# Set the desired resolution
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open video device.")
    exit()

prev_time = time.time()  # <--- 2. Variable to store the time of the previous frame

print("Starting USB camera feed... Press 'q' to quit.")

# --- Real-time Detection Loop ---

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Can't receive frame. Exiting ...")
        break

    # Run YOLOv8 inference
    results = model.predict(frame, imgsz=320, verbose=False)

    # Get the annotated frame
    annotated_frame = results[0].plot()

    # --- 3. Calculate and Display FPS ---
    current_time = time.time()
    fps = 1 / (current_time - prev_time)
    prev_time = current_time

    # Draw the FPS text on the frame
    cv2.putText(annotated_frame, f"FPS: {int(fps)}", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    # --- End of FPS Code ---

    # 4. Display the annotated frame
    cv2.imshow('YOLOv8 Live USB Cam', annotated_frame)

    # Check for 'q' key press to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# --- Cleanup ---
print("Cleaning up and exiting...")
cap.release()
cv2.destroyAllWindows()
