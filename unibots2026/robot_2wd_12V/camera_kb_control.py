import cv2
import time
import serial
import sys
import os
import numpy as np
import threading
from picamera2 import Picamera2
from libcamera import Transform
from ultralytics import YOLO

# Add apriltag library path
sys.path.append('/usr/local/lib/python3.11/site-packages')
try:
    from apriltag import apriltag
except ImportError:
    print("Warning: apriltag library not found! Please check the path.")

# --- CONFIGURATION ---
SERIAL_PORT = '/dev/ttyACM0'  # Adjust if needed
YOLO_MODEL_PATH = "/home/pi/Raspi_codes/unibots2026/robot_2wd_12V/ball_detection/best.pt"
NCNN_MODEL = "/home/pi/yolo_project/ball_detect_ncnn_model"
APRILTAG_FAMILY = "tagStandard41h12"
WIDTH = 320
HEIGHT = 240

# Select the best model available
if os.path.exists(NCNN_MODEL):
    YOLO_MODEL_PATH = NCNN_MODEL

# --- SERIAL SETUP ---
try:
    ser = serial.Serial(SERIAL_PORT, 9600, timeout=1)
    ser.reset_input_buffer()
    print(f"Connected to Arduino on {SERIAL_PORT}!")
    time.sleep(2)
except Exception as e:
    print(f"Error connecting to Serial: {e}")
    ser = None

def send_cmd(cmd):
    if ser:
        ser.write(cmd.encode() if isinstance(cmd, str) else cmd)

# --- MODELS SETUP ---
print(f"Loading YOLO model: {YOLO_MODEL_PATH}")
model = YOLO(YOLO_MODEL_PATH)

print("Initializing AprilTag detector...")
try:
    detector = apriltag(APRILTAG_FAMILY)
except NameError:
    detector = None

# --- CAMERA SETUP ---
print("Starting Camera...")
picam2 = Picamera2()
config = picam2.create_video_configuration(
    main={"size": (WIDTH, HEIGHT), "format": "RGB888"}
)
# Uncomment the following line if your camera needs flipping:
# picam2.preview_configuration.transform = Transform(hflip=1, vflip=0)
picam2.configure(config)
picam2.start()

print("\n---------------------------------------")
print("   CAMERA & KEYBOARD CONTROL ACTIVE! ")
print("---------------------------------------")
print("- Make sure the OpenCV Video window is focused.")
print("- Controls: W (Forward), S (Backward), A (Left), D (Right)")
print("- SPACE: Stop | 1, 2, 3: Set Speed")
print("- Toggles: B (Ball Detection) | T (AprilTag Detection)")
print("- Q: Quit")
print("---------------------------------------\n")

# --- THREADING DEFS ---
running = True
latest_frame = None
detect_ball = False
detect_apriltag = False

def camera_processing_thread():
    global running, latest_frame, detect_ball, detect_apriltag
    while running:
        try:
            # Capture frame
            frame = picam2.capture_array()
            
            # Convert for display (OpenCV uses BGR)
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            annotated_frame = frame_bgr.copy()
            
            # 1. YOLO Ball Detection
            if detect_ball:
                results = model.predict(frame_bgr, imgsz=WIDTH, verbose=False, conf=0.4)
                # This returns a BGR image with YOLO boxes drawn on it
                annotated_frame = results[0].plot()
            
            # 2. AprilTag Detection
            if detect_apriltag and detector:
                # Convert for AprilTag detection (Grayscale)
                frame_gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
                detections = detector.detect(frame_gray)
                
                for detection in detections:
                    if detection and 'lb-rb-rt-lt' in detection:
                        corners = np.array(detection['lb-rb-rt-lt']).astype(int)
                        tag_id = str(detection['id'])
                        center = np.array(detection['center']).astype(int)

                        # Draw bounding box
                        cv2.polylines(annotated_frame, [corners], isClosed=True, color=(0, 255, 0), thickness=2)
                        # Draw center point
                        cv2.circle(annotated_frame, tuple(center), 5, (0, 0, 255), -1)
                        # Draw Tag ID
                        cv2.putText(annotated_frame, f"ID: {tag_id}", (corners[0][0], corners[0][1] - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Lock the update
            latest_frame = annotated_frame
        except Exception as e:
            time.sleep(0.1)

# Start background camera thread
cam_thread = threading.Thread(target=camera_processing_thread)
cam_thread.daemon = True
cam_thread.start()

try:
    while True:
        if latest_frame is not None:
             # Show Combined Output
            cv2.imshow("Robot View - Controls (WASD)", latest_frame)
        
        # Handle Keyboard Inputs (Must have OpenCV window focused)
        # Using a very quick waitKey polling to stay highly responsive
        key = cv2.waitKey(10) & 0xFF
        
        if key == ord('w'):
            send_cmd('F')
            print("Action: Forward")
        elif key == ord('s'):
            send_cmd('B')
            print("Action: Backward")
        elif key == ord('a'):
            send_cmd('L')
            print("Action: Turn Left")
        elif key == ord('d'):
            send_cmd('R')
            print("Action: Turn Right")
        elif key == ord(' '):
            send_cmd('S')
            print("Action: Stop")
        elif key == ord('1'):
            send_cmd('1')
            print("Speed: Low")
        elif key == ord('2'):
            send_cmd('2')
            print("Speed: Medium")
        elif key == ord('3'):
            send_cmd('3')
            print("Speed: High")
        elif key == ord('b'):
            detect_ball = not detect_ball
            print(f"Ball Detection: {'ON' if detect_ball else 'OFF'}")
        elif key == ord('t'):
            detect_apriltag = not detect_apriltag
            print(f"AprilTag Detection: {'ON' if detect_apriltag else 'OFF'}")
        elif key == ord('q'):
            send_cmd('S')
            print("Quitting...")
            running = False
            break

except Exception as e:
    print(f"Loop interrupted: {e}")
    
finally:
    if ser:
        ser.write(b'S')
    picam2.stop()
    cv2.destroyAllWindows()
