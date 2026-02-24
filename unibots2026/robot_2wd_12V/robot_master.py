import cv2
from picamera2 import Picamera2
from ultralytics import YOLO
import time
import serial
import threading
import sys
import datetime
from gpiozero import Button

# --- CONFIGURATION ---
# Robot
BUTTON_PIN = 25  # The GPIO pin for your button
STOP_DIST = 35
SIDE_DIST = 30
SERIAL_PORT = '/dev/ttyACM0'

# Camera / YOLO
# We use a 320x240 resolution for better compatibility with Pi 5 hardware
CAPTURE_WIDTH = 320
CAPTURE_HEIGHT = 240
INFERENCE_SIZE = 320

# Use NCNN model for faster CPU inference
# If this fails, it will fallback to PyTorch
import os
NCNN_MODEL = "/home/pi/yolo_project/yolov8n_ncnn_model"
PT_MODEL = "/home/pi/yolo_project/yolov8n.pt"

if os.path.exists(NCNN_MODEL):
    print(f"Using NCNN model for speed: {NCNN_MODEL}")
    MODEL_PATH = NCNN_MODEL
else:
    print(f"NCNN model not found, using PyTorch (slower): {PT_MODEL}")
    MODEL_PATH = PT_MODEL

# --- LOGGING FUNCTIONS ---
def log_camera_detection(class_name, conf):
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    log_entry = f"{timestamp} | Object: {class_name} | Confidence: {conf:.2f}\n"
    with open("camera_log.txt", "a") as f:
        f.write(log_entry)

def log_motion_action(action_desc, l, c, r):
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    log_entry = f"{timestamp} | Action: {action_desc} | L:{l} C:{c} R:{r}\n"
    with open("motion_log.txt", "a") as f:
        f.write(log_entry)

# --- GLOBAL VARIABLES ---
# Sensor Data (shared)
latest_L = 999
latest_C = 999
latest_R = 999

# State Flags
running = True        # Master flag to keep threads alive
robot_active = False  # Toggled by button to enable/disable motion

# Serial Connection
try:
    ser = serial.Serial(SERIAL_PORT, 9600, timeout=1)
    ser.reset_input_buffer()
    print("Connected to Robot via Serial.")
    time.sleep(2)
except Exception as e:
    print(f"Error connecting to Serial: {e}")
    ser = None

# --- BUTTON TOGGLE FUNCTION ---
def toggle_mode():
    global robot_active
    robot_active = not robot_active 
    
    if robot_active:
        print("\n>>> GO! Robot Started. <<<")
        if ser: ser.write(b'3') 
    else:
        print("\n>>> PAUSE! Robot Stopped. <<<")
        if ser: ser.write(b'S') 

# Setup Button
run_button = Button(BUTTON_PIN, bounce_time=0.2)
run_button.when_pressed = toggle_mode


# --- THREAD 1: SENSOR READER ---
# Reads serial data from Arduino continuously to update global distances
def update_sensors():
    global latest_L, latest_C, latest_R, running
    while running:
        if ser and ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8').rstrip()
                if line.startswith("D,"):
                    parts = line.split(",")
                    latest_L = int(parts[1])
                    latest_C = int(parts[2])
                    latest_R = int(parts[3])
            except:
                pass
        time.sleep(0.01)

# --- THREAD 2: ROBOT CONTROL LOGIC ---
# Handles the "Brain" of the movement: Avoidance logic
def robot_control_loop():
    global running, robot_active, latest_L, latest_C, latest_R
    while running:
        if robot_active and ser:
            # === AUTONOMOUS LOGIC ===
            
            # CASE A: OBSTACLE DEAD AHEAD
            if latest_C < STOP_DIST:
                log_motion_action(f"BLOCKED ({latest_C}cm)", latest_L, latest_C, latest_R)
                print(f"Blocked ({latest_C}cm)! Avoiding...")
                ser.write(b'S')
                time.sleep(0.2)
                
                ser.write(b'B') # Reverse
                time.sleep(0.3)
                
                # Turn to clearer side
                if latest_L > latest_R:
                    print("Turning LEFT")
                    log_motion_action("Turn LEFT", latest_L, latest_C, latest_R)
                    ser.write(b'L') 
                else:
                    print("Turning RIGHT")
                    log_motion_action("Turn RIGHT", latest_L, latest_C, latest_R)
                    ser.write(b'R')
                    
                time.sleep(0.5) 
                ser.write(b'S')
                time.sleep(0.2)
                
            # CASE B: TOO CLOSE TO LEFT WALL
            elif latest_L < SIDE_DIST:
                print("Nudging Right")
                log_motion_action("Nudge RIGHT", latest_L, latest_C, latest_R)
                ser.write(b'R')
                time.sleep(0.1)
                ser.write(b'F') # Resume Forward
                
            # CASE C: TOO CLOSE TO RIGHT WALL
            elif latest_R < SIDE_DIST:
                print("Nudging Left")
                log_motion_action("Nudge LEFT", latest_L, latest_C, latest_R)
                ser.write(b'L')
                time.sleep(0.1)
                ser.write(b'F')

            # CASE D: PATH CLEAR
            else:
                # Optional: Log 'Forward' periodically to avoid massive log file
                # For now, we won't log straight forward to save disk space
                ser.write(b'F')
                
        else:
            # Paused
            time.sleep(0.1)

        time.sleep(0.05)


# --- THREAD 3: CAMERA & YOLO ---
# Captures images and runs detection. 
def camera_yolo_loop():
    global running
    
    # Init Camera
    try:
        picam2 = Picamera2()
        config = picam2.create_video_configuration(
            main={"size": (CAPTURE_WIDTH, CAPTURE_HEIGHT), "format": "RGB888"},
            buffer_count=12
        )
        picam2.configure(config)
        picam2.start()
        time.sleep(1) # Warmup
        print("Camera Started.")
    except Exception as e:
        print(f"Camera Error: {e}")
        return

    # Init YOLO
    try:
        print("Loading YOLO model...")
        model = YOLO(MODEL_PATH)
        print("Model Loaded.")
    except Exception as e:
        print(f"YOLO Error: {e}")
        picam2.stop()
        return

    # Skipped frames Logic
    frame_count = 0
    SKIP_FRAMES = 3 # Process 1 out of every 3 frames
    last_annotated_frame = None
    prev_time = time.time()

    print("Starting Camera Loop...")
    while running:
        try:
            # 1. Capture
            frame = picam2.capture_array()
            
            # Increment frame counter
            frame_count += 1

            # 2. Predict (Only every SKIP_FRAMES)
            if frame_count % SKIP_FRAMES == 0 or last_annotated_frame is None:
                results = model.predict(frame, imgsz=INFERENCE_SIZE, verbose=False, conf=0.4)
                
                # --- LOGGING DETECTIONS ---
                # results is a list [Result(), Result(), ...]
                if len(results) > 0:
                    for r in results:
                        # r.boxes contains the bounding boxes
                        for box in r.boxes:
                            # box.cls is tensor([class_id])
                            class_id = int(box.cls[0])
                            # box.conf is tensor([confidence])
                            confidence = float(box.conf[0])
                            # Get class name
                            class_name = model.names[class_id]
                            
                            # Log to file
                            log_camera_detection(class_name, confidence)
                
                # 3. Annotate
                last_annotated_frame = results[0].plot()
                
                # 4. FPS Calculation (Update only on processed frames)
                current_time = time.time()
                fps = 1 / (current_time - prev_time)
                prev_time = current_time
                cv2.putText(last_annotated_frame, f"FPS: {int(fps)}", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                display_frame = last_annotated_frame
            else:
                # Just display the last known good frame to save processing
                # Or display current raw frame? 
                # Displaying last *annotated* frame makes valid detections persist 
                # but video looks 1/3fps. 
                # Better: Display current frame? No, we want to see boxes. 
                # We will stick to displaying the last annotated frame for simplicity/speed
                # This makes the video 'choppy' but the robot reaction is what matters.
                display_frame = last_annotated_frame
            
            # 5. Display
            cv2.imshow("Robot Vision", display_frame)
            
            # Exit on Q
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Q pressed. Exiting...")
                running = False
                break
        except Exception as e:
            print(f"Camera Loop Error: {e}")
            break

    # Cleanup Camera stuff
    try:
        picam2.stop()
    except:
        pass
    cv2.destroyAllWindows()


# --- START STARTUP ---
if __name__ == "__main__":
    print(f"--------------------------------")
    print(f"ROBOT MASTER STARTED")
    print(f"Press Button on GPIO {BUTTON_PIN} to Start/Stop Movement.")
    print(f"Press 'q' in Camera Window to Exit Program.")
    print(f"--------------------------------")

    # Start Sensor Thread
    t_sensors = threading.Thread(target=update_sensors)
    t_sensors.daemon = True
    t_sensors.start()

    # Start Robot Control Thread
    t_robot = threading.Thread(target=robot_control_loop)
    t_robot.daemon = True
    t_robot.start()

    # Run Camera in Main Thread (It acts as parallel to the others)
    camera_yolo_loop()

    # When camera loop ends (user pressed q):
    running = False
    
    # Stop robot
    if ser:
        ser.write(b'S')
    
    print("Main thread exiting.")
