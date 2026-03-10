import cv2
import time
import serial
import sys
import os
import numpy as np
import threading
from queue import Queue
from picamera2 import Picamera2
from libcamera import Transform
from ultralytics import YOLO

# Import pupil_apriltags for AprilTag detection and pose estimation
try:
    from pupil_apriltags import Detector as PoseDetector
    APRILTAG_AVAILABLE = True
except ImportError:
    print("Warning: pupil_apriltags not found! AprilTag detection disabled.")
    APRILTAG_AVAILABLE = False

# --- CONFIGURATION ---
SERIAL_PORT = '/dev/ttyACM0'  # Adjust if needed
YOLO_MODEL_PATH = "/home/pi/yolo_project/orange_ball.pt"
NCNN_MODEL = "/home/pi/yolo_project/orange_ball_ncnn_model"
APRILTAG_FAMILY = "tagStandard41h12"
WIDTH = 640 
HEIGHT = 480
SCALE = 3.0 # Scale factor for AprilTag distance estimation - adjust based on your camera setup and testing

# AprilTag Pose Detection Parameters (from camera calibration)
CAMERA_PARAMS = (954.4949188171072, 955.5979729485147, 332.0798756650343, 245.67451277016548)
TAG_SIZE = 0.10  # 10 centimeters = 0.10 meters

# Select the best model available
if os.path.exists(NCNN_MODEL):
    YOLO_MODEL_PATH = NCNN_MODEL

# --- SERIAL SETUP ---
try:
    ser = serial.Serial(SERIAL_PORT, 115200, timeout=0)  # Zero timeout for non-blocking
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    print(f"Connected to Arduino on {SERIAL_PORT}!")
    time.sleep(2)
except Exception as e:
    print(f"Error connecting to Serial: {e}")
    ser = None

# Command queue for non-blocking serial writes
cmd_queue = Queue(maxsize=5)

def send_cmd(cmd):
    """Queue command for immediate transmission"""
    if ser:
        # Drop old commands if queue is full - only latest matters
        while not cmd_queue.empty():
            try:
                cmd_queue.get_nowait()
            except:
                pass
        cmd_queue.put(cmd)

def serial_writer_thread():
    """Dedicated thread for serial writes - lowest latency"""
    global running
    while running:
        try:
            cmd = cmd_queue.get(timeout=0.005)  # 5ms timeout
            if ser:
                ser.write(cmd.encode() if isinstance(cmd, str) else cmd)
                ser.flush()
        except:
            pass

# --- MODELS SETUP ---
print(f"Loading YOLO model: {YOLO_MODEL_PATH}")
model = YOLO(YOLO_MODEL_PATH)

print("Initializing AprilTag detector...")
if APRILTAG_AVAILABLE:
    apriltag_detector = PoseDetector(families='tagStandard41h12', quad_decimate=2.0)
else:
    apriltag_detector = None

# --- CAMERA SETUP ---
print("Starting Camera...")
picam2 = Picamera2()
config = picam2.create_video_configuration(
    main={"size": (WIDTH, HEIGHT), "format": "RGB888"},
    controls={"FrameRate": 30},
    buffer_count=2  # Minimize frame buffer to reduce latency
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
print("- Toggles: B (Ball Detection) | T (AprilTag Detection) | P (Pose Detection) | C (Camera Stream)")
print("- Q: Quit")
print("---------------------------------------\n")

# --- THREADING DEFS ---
running = True
raw_frame = None  # Holds the absolute latest frame from the camera sensor
latest_frame = None  # Holds the annotated frame for UI
detect_ball = False
detect_apriltag = False
detect_pose = False
stream_camera = True
frame_lock = threading.Lock()

def camera_capture_thread():
    """ Dedicated thread purely to empty the camera buffer as fast as possible. """
    global running, raw_frame
    while running:
        try:
            # Constantly reading prevents libcamera's internal FIFO buffer from filling up
            # ensuring we always have the freshest possible frame with 0 lag.
            frame = picam2.capture_array()
            raw_frame = frame
        except Exception as e:
            time.sleep(0.01)

def camera_processing_thread():
    global running, latest_frame, raw_frame, detect_ball, detect_apriltag, detect_pose, stream_camera
    while running:
        try:
            if not stream_camera and not detect_ball and not detect_apriltag and not detect_pose:
                # Still need a latest_frame to show the paused text, but we stop capturing
                blank = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
                cv2.putText(blank, "Stream Paused", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.putText(blank, "Press 'c' to resume", (30, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                with frame_lock:
                    latest_frame = blank
                time.sleep(0.1)
                continue
            
            # Get latest frame pulled by capture thread
            frame = raw_frame
            if frame is None:
                time.sleep(0.01)
                continue
            
            # Convert for display (OpenCV uses BGR)
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            annotated_frame = frame_bgr  # Avoid copy unless needed
            
            # 1. YOLO Ball Detection
            if detect_ball:
                results = model.predict(frame_bgr, imgsz=WIDTH, verbose=False, conf=0.4)
                # This returns a BGR image with YOLO boxes drawn on it
                annotated_frame = results[0].plot()
            
            # 2. AprilTag Detection (basic - no pose)
            # Pre-compute grayscale if needed for AprilTag detection
            frame_gray = None
            if (detect_apriltag or detect_pose) and apriltag_detector:
                frame_gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            
            if detect_apriltag and apriltag_detector:
                detections = apriltag_detector.detect(frame_gray)
                
                for tag in detections:
                    corners = tag.corners.astype(int)
                    tag_id = str(tag.tag_id)
                    center = (int(tag.center[0]), int(tag.center[1]))

                    # Draw bounding box
                    cv2.polylines(annotated_frame, [corners], isClosed=True, color=(0, 255, 0), thickness=2)
                    # Draw center point
                    cv2.circle(annotated_frame, center, 5, (0, 0, 255), -1)
                    # Draw Tag ID
                    cv2.putText(annotated_frame, f"ID: {tag_id}", (corners[0][0], corners[0][1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # 3. AprilTag Pose Detection (with distance/offset estimation)
            if detect_pose and apriltag_detector:
                tags = apriltag_detector.detect(frame_gray, estimate_tag_pose=True, 
                                           camera_params=CAMERA_PARAMS, tag_size=TAG_SIZE)
                
                for tag in tags:
                    tag_id = tag.tag_id
                    
                    # Translation vector (X, Y, Z distance from camera in meters)
                    x_dist = tag.pose_t[0][0]  # Left/Right offset
                    y_dist = tag.pose_t[1][0]  # Up/Down offset
                    z_dist = tag.pose_t[2][0] / SCALE # Forward distance to tag
                    
                    # Print for debugging
                    print(f"Tag ID: {tag_id} | Distance: {z_dist:.2f}m | X-Offset: {x_dist:.2f}m")
                    
                    # Draw bounding box
                    cv2.polylines(annotated_frame, [tag.corners.astype(int)], True, (0, 255, 0), 2)
                    
                    # Draw tag ID and pose info
                    center_x, center_y = int(tag.center[0]), int(tag.center[1])
                    cv2.putText(annotated_frame, f"ID:{tag_id}", (center_x - 20, center_y - 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    cv2.putText(annotated_frame, f"D:{z_dist:.2f}m", (center_x - 30, center_y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
                    cv2.putText(annotated_frame, f"X:{x_dist:.2f}m", (center_x - 30, center_y + 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
            
            # Thread-safe frame update
            with frame_lock:
                latest_frame = annotated_frame
        except Exception as e:
            time.sleep(0.01)

# Start background threads
capture_thread = threading.Thread(target=camera_capture_thread)
capture_thread.daemon = True
capture_thread.start()

process_thread = threading.Thread(target=camera_processing_thread)
process_thread.daemon = True
process_thread.start()

serial_thread = threading.Thread(target=serial_writer_thread)
serial_thread.daemon = True
serial_thread.start()

# Movement state tracking for auto-stop
last_movement_time = 0
current_movement = None
STOP_DELAY = 0.15  # 150ms - handle keyboard repeat delay to prevent stuttering

# Pre-create blank frame for paused state
blank_frame = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
cv2.putText(blank_frame, "Stream Paused", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
cv2.putText(blank_frame, "Press 'c' to resume", (30, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

last_frame_displayed = None

try:
    while True:
        # Handle Keyboard FIRST - highest priority for lowest latency
        key = cv2.waitKey(10) & 0xFF
        
        movement_key_pressed = False
        
        if key == ord('w'):
            if current_movement != 'F':
                send_cmd('F')
                current_movement = 'F'
            last_movement_time = time.time()
            movement_key_pressed = True
        elif key == ord('s'):
            if current_movement != 'B':
                send_cmd('B')
                current_movement = 'B'
            last_movement_time = time.time()
            movement_key_pressed = True
        elif key == ord('a'):
            if current_movement != 'L':
                send_cmd('L')
                current_movement = 'L'
            last_movement_time = time.time()
            movement_key_pressed = True
        elif key == ord('d'):
            if current_movement != 'R':
                send_cmd('R')
                current_movement = 'R'
            last_movement_time = time.time()
            movement_key_pressed = True
        elif key == ord(' '):
            send_cmd('S')
            current_movement = None
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
            if APRILTAG_AVAILABLE:
                detect_apriltag = not detect_apriltag
                print(f"AprilTag Detection: {'ON' if detect_apriltag else 'OFF'}")
            else:
                print("AprilTag detection not available (pupil_apriltags not installed)")
        elif key == ord('p'):
            if APRILTAG_AVAILABLE:
                detect_pose = not detect_pose
                print(f"AprilTag Pose Detection: {'ON' if detect_pose else 'OFF'}")
            else:
                print("Pose detection not available (pupil_apriltags not installed)")
        elif key == ord('c'):
            stream_camera = not stream_camera
            print(f"Camera Stream: {'ON' if stream_camera else 'OFF'}")
        elif key == ord('q'):
            send_cmd('S')
            print("Quitting...")
            running = False
            break
        
        # Auto-stop when no movement key is pressed for STOP_DELAY seconds
        if not movement_key_pressed and current_movement is not None:
            if time.time() - last_movement_time > STOP_DELAY:
                send_cmd('S')
                current_movement = None
        
        # Update display only when a new frame is available or stream is toggled
        display_frame = None
        with frame_lock:
            if latest_frame is not last_frame_displayed:
                display_frame = latest_frame
                last_frame_displayed = latest_frame
        
        if display_frame is not None:
            if stream_camera:
                cv2.imshow("Robot View - Controls (WASD)", display_frame)
            else:
                cv2.imshow("Robot View - Controls (WASD)", blank_frame)

except Exception as e:
    print(f"Loop interrupted: {e}")
    
finally:
    if ser:
        ser.write(b'S')
    picam2.stop()
    cv2.destroyAllWindows()
