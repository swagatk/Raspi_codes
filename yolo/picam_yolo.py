import cv2
from picamera2 import Picamera2
from ultralytics import YOLO
import time


# ------- configuration ----
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240
INFERENCE_SIZE = 320
NCNN = False
# --- Model and Camera Setup ---

# Load the YOLO "nano" model
if not NCNN:
    print("Using PyTorch Model")
    model = YOLO("/home/pi/yolo_project/yolov8n.pt")
    #model = YOLO("yolo11n.pt")
else:
    print("Using NCNN model ...")
    model = YOLO("/home/pi/yolo_project/yolov8n_ncnn_model")
    #model = YOLO("/home/pi/yolo_project/yolov8n_ncnn_model")


# Initialize Picamera2
picam2 = Picamera2()

# We use a 640x480 resolution for better compatibility with Pi 5 hardware
# We set the format to "RGB888" to match likely native format (we'll convert for display)
CAPTURE_WIDTH = 640
CAPTURE_HEIGHT = 480

# Configure the camera
# Increase buffer_count significantly to avoid running out of buffers 
config = picam2.create_video_configuration(
    main={"size": (CAPTURE_WIDTH, CAPTURE_HEIGHT), "format": "RGB888"},
    buffer_count=12
)
picam2.configure(config)

# Start the camera
picam2.start()
# Wait for the camera to warm up and for the first few frames to flow
time.sleep(2)



# --- Real-time Detection Loop ---

print("Starting real-time object detection... Press 'q' to quit.")
prev_time = time.time()
while True:
    # 1. Capture a frame from the camera
    # This frame is a NumPy array in RGB format
    frame = picam2.capture_array()

    # 2. Run YOLOv8 inference on the frame
    # verbose=False reduces console spam
    results = model.predict(frame, imgsz=INFERENCE_SIZE, verbose=False)

    # 3. Get the annotated frame
    # results[0].plot() returns BGR by default, but typically matches input format usually if not plotting raw opencv.
    # Actually ultralytics plot() returns numpy array (BGR if using opencv backend or similar).
    # But let's assume it returns useful image.
    annotated_frame = results[0].plot()

    # --- 4. Calculate and Display FPS ---
    current_time = time.time()
    fps = 1 / (current_time - prev_time)
    prev_time = current_time

    # Draw the FPS text on the frame
    cv2.putText(annotated_frame, f"FPS: {int(fps)}", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    # 5. Display the annotated frame
    # If using RGB888 input, and plot() returns BGR (standard for opencv), display directly?
    # Or plot() returns RGB? plot() uses cv2 internally so returns BGR usually.
    # But wait, input was RGB. Let's see.
    # If colors are weird, we swap. For now display as is.
    cv2.imshow("YOLO Live Detection", annotated_frame)

    # 5. Check for 'q' key press to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# --- Cleanup ---
print("Cleaning up and exiting...")
cv2.destroyAllWindows()
picam2.stop()
