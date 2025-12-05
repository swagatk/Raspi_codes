import cv2
from picamera2 import Picamera2
from ultralytics import YOLO
import time


# ------- configuration ----
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240
INFERENCE_SIZE = 320
NCNN = True
# --- Model and Camera Setup ---

# Load the YOLO "nano" model
if not NCNN:
    print("Using PyTorch Model")
    model = YOLO("yolov8n.pt")
    #model = YOLO("yolo11n.pt")
else:
    print("Using NCNN model ...")
    model = YOLO("/home/pi/yolo_project/yolov8n_ncnn_model")
    #model = YOLO("/home/pi/yolo_project/yolov8n_ncnn_model")


# Initialize Picamera2
picam2 = Picamera2()

# Configure the camera for preview
# We use a 640x480 resolution for a good balance of speed and accuracy
# We set the format to "RGB888" which is what OpenCV and YOLO expect
config = picam2.create_preview_configuration(
    main={"size": (IMAGE_WIDTH, IMAGE_HEIGHT), "format": "RGB888"}
)
picam2.configure(config)

# Start the camera
picam2.start()



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
    cv2.imshow("YOLO Live Detection", cv2.cvtColor(annotated_frame, cv2.COLOR_RGB2BGR))

    # 5. Check for 'q' key press to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# --- Cleanup ---
print("Cleaning up and exiting...")
cv2.destroyAllWindows()
picam2.stop()
