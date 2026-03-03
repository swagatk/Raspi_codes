import cv2
import time
import serial
import threading
import datetime
import os
from gpiozero import Button
from picamera2 import Picamera2
from libcamera import Transform
from ultralytics import YOLO

# --- CONFIGURATION ---
SERIAL_PORT = '/dev/ttyACM0'  # Adjust if needed
BUTTON_PIN = 25

# Model Selection
NCNN_MODEL = "/home/pi/yolo_project/ball_detect_ncnn_model"
PT_MODEL = "/home/pi/yolo_project/ball_detect.pt"

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

# Movement Thresholds
STOP_DIST = 35        # cm
SIDE_DIST = 30        # cm
CENTER_TOLERANCE = 80 # pixels from center to consider "centered"
BOTTOM_TOLERANCE = 210 # If ball_y is greater than this, stop (max is CAPTURE_HEIGHT, e.g., 240)

# Camera Settings
CAPTURE_WIDTH = 320
CAPTURE_HEIGHT = 240
CENTER_X = CAPTURE_WIDTH // 2

# --- GLOBAL VARIABLES ---
# Thread Control
running = True
robot_active = False

# Sensor Readings (Shared)
latest_L = 999
latest_C = 999
latest_R = 999

# Ball Detection (Shared)
ball_visible = False
ball_x = 0
ball_y = 0
ball_w = 0
ball_h = 0

# Serial Connection
try:
    ser = serial.Serial(SERIAL_PORT, 9600, timeout=1)
    ser.reset_input_buffer()
    print(f"Connected to Serial: {SERIAL_PORT}")
    time.sleep(2) # Allow Arduino to reset
except Exception as e:
    print(f"Error connecting to Serial: {e}")
    ser = None

# --- SERIAL HELPERS ---
def robot_stop():
    if ser: ser.write(b'S')

def set_speed(level):
    # level can be 1 (slow), 2 (medium), 3 (fast)
    if ser: ser.write(str(level).encode())

def robot_forward():
    if ser: 
        set_speed(2) # Normal speed for forward
        ser.write(b'F')

def robot_backward():
    if ser: 
        set_speed(2)
        ser.write(b'B')

def robot_left():
    if ser: 
        set_speed(1) # Minimum speed for rotating
        ser.write(b'L')

def robot_right():
    if ser: 
        set_speed(1) # Minimum speed for rotating
        ser.write(b'R')

def robot_spin_search():
    # Spin slowly to find ball.
    if ser:
        set_speed(1) # Minimum speed for rotating
        ser.write(b'R')

# --- BUTTON HANDLER ---
def toggle_mode():
    global robot_active
    robot_active = not robot_active
    if robot_active:
        print("\n>>> ACTIVE: Robot Started. Searching for ball... <<<")
    else:
        print("\n>>> PAUSED: Robot Stopped. <<<")
        robot_stop()

run_button = Button(BUTTON_PIN, bounce_time=0.2)
run_button.when_pressed = toggle_mode


# --- THREAD 1: SENSOR READER ---
def update_sensors():
    global latest_L, latest_C, latest_R, running
    while running:
        if ser and ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8').rstrip()
                # Expected format: "D,Left,Center,Right" e.g "D,120,45,120"
                if line.startswith("D,"):
                    parts = line.split(",")
                    if len(parts) == 4:
                        latest_L = int(parts[1])
                        latest_C = int(parts[2])
                        latest_R = int(parts[3])
            except Exception:
                pass
        time.sleep(0.01)


# --- THREAD 2: CAMERA & YOLO ---
def camera_yolo_loop():
    global running, ball_visible, ball_x, ball_y, ball_w, ball_h
    
    # 1. Initialize Camera
    try:
        picam2 = Picamera2()
        config = picam2.create_video_configuration(
            main={"size": (CAPTURE_WIDTH, CAPTURE_HEIGHT), "format": "RGB888"},
            buffer_count=12
        )
        picam2.configure(config)
        # Flip if needed (based on ball_detection.py which had hflip=1, vflip=0)
        # picam2.camera.Controls.Transform = Transform(hflip=1, vflip=0)
        # Note: Picamera2 configuration method varies. 
        # ball_detection.py used: picam2.preview_configuration.transform = Transform(hflip=1, vflip=0)
        # We will try to rely on default or apply similar if needed.
        # However, for simplicity let's stick to standard config first. 
        # If orientation is wrong, we can adjust.
        picam2.start()
        print("Camera Started.")
    except Exception as e:
        print(f"Camera Error: {e}")
        return

    # 2. Load Model
    try:
        print(f"Loading YOLO model: {MODEL_PATH}")
        model = YOLO(MODEL_PATH)
        print("Model Loaded.")
    except Exception as e:
        print(f"YOLO Error: {e}")
        picam2.stop()
        return

    print("Starting Detection Loop...")
    
    while running:
        try:
            # Capture
            frame = picam2.capture_array()
            
            # Predict
            # Using raw frame (RGB). YOLO can handle it, or convert to BGR for OpenCV display.
            # OpenCV expects BGR. Ultralytics handles RGB/BGR but consistency is good.
            # Let's convert to BGR for display purposes later.
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            # Run inference
            results = model.predict(frame_bgr, imgsz=320, verbose=False, conf=0.4)
            
            # Process Detections
            # We want to find the largest ball (closest)
            largest_box = None
            max_area = 0
            
            if len(results) > 0:
                for r in results:
                    boxes = r.boxes
                    for box in boxes:
                        # Log detection
                        class_id = int(box.cls[0])
                        confidence = float(box.conf[0])
                        class_name = model.names[class_id]
                        log_camera_detection(class_name, confidence)

                        # Check for largest ball tracking
                        # box.xywh returns center_x, center_y, width, height
                        x, y, w, h = box.xywh[0]
                        area = float(w) * float(h)
                        if area > max_area:
                            max_area = area
                            largest_box = (float(x), float(y), float(w), float(h))
            
            # Update Shared State
            if largest_box:
                ball_visible = True
                ball_x, ball_y, ball_w, ball_h = largest_box
            else:
                ball_visible = False
            
            # Visualization
            annotated_frame = results[0].plot()
            cv2.imshow("Robot Vision", annotated_frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                running = False
                break
                
        except Exception as e:
            print(f"Detection Loop Error: {e}")
            time.sleep(0.1)

    picam2.stop()
    cv2.destroyAllWindows()


# --- THREAD 3: ROBOT CONTROL ---
def robot_control_loop():
    global running, robot_active
    global latest_L, latest_C, latest_R
    global ball_visible, ball_x
    
    print("Robot Control Loop Started.")
    
    while running:
        if robot_active and ser:
            # ----------------------------------
            # PRIORITY 1: OBSTACLE AVOIDANCE
            # ----------------------------------
            if latest_C < STOP_DIST:
                log_motion_action(f"BLOCKED ({latest_C}cm)", latest_L, latest_C, latest_R)
                print(f"OBSTACLE AHEAD ({latest_C}cm)! Avoiding...")
                robot_stop()
                time.sleep(0.1)
                robot_backward()
                time.sleep(0.3)
                
                # Turn away from obstacle
                if latest_L > latest_R:
                    log_motion_action("Turn LEFT (Avoid)", latest_L, latest_C, latest_R)
                    robot_left()
                else:
                    log_motion_action("Turn RIGHT (Avoid)", latest_L, latest_C, latest_R)
                    robot_right()
                time.sleep(0.5)
                robot_stop()
                continue # Skip rest of loop to re-evaluate sensors

            elif latest_L < SIDE_DIST:
                log_motion_action("Nudge RIGHT", latest_L, latest_C, latest_R)
                print("Too close Left - Nudging Right")
                robot_right()
                time.sleep(0.1)
                robot_forward()
                time.sleep(0.1)
                continue

            elif latest_R < SIDE_DIST:
                log_motion_action("Nudge LEFT", latest_L, latest_C, latest_R)
                print("Too close Right - Nudging Left")
                robot_left()
                time.sleep(0.1)
                robot_forward()
                time.sleep(0.1)
                continue

            # ----------------------------------
            # PRIORITY 2: BALL TRACKING
            # ----------------------------------
            if ball_visible:
                # Calculate error from center
                # ball_x is 0..320. Center is 160.
                error = ball_x - CENTER_X
                
                # Debug
                # print(f"Tracking Ball: x={ball_x:.1f}, error={error:.1f}, y={ball_y:.1f}")
                
                if ball_y > BOTTOM_TOLERANCE:
                    # Ball is very close (near bottom of frame), stop!
                    print("Ball Reached! Stopping.")
                    log_motion_action("Ball Reached (Stop)", latest_L, latest_C, latest_R)
                    robot_stop()
                    # Wait slightly so we don't rapid-fire commands once arrived
                    time.sleep(0.5)
                elif abs(error) < CENTER_TOLERANCE:
                    # Ball is roughly centered, move towards it
                    # log_motion_action("Forward (Track)", latest_L, latest_C, latest_R) # Optional: reduce log spam
                    robot_forward()
                elif error < 0:
                    # Ball is to the Left (x < 160)
                    log_motion_action("Turn LEFT (Track)", latest_L, latest_C, latest_R)
                    robot_left()
                    time.sleep(0.01)  # turn very slightly
                    robot_stop()
                    time.sleep(0.3)   # wait for camera to update frame
                else:
                    # Ball is to the Right (x > 160)
                    log_motion_action("Turn RIGHT (Track)", latest_L, latest_C, latest_R)
                    robot_right()
                    time.sleep(0.01)  # turn very slightly
                    robot_stop()
                    time.sleep(0.3)   # wait for camera to update frame
                
                # Short delay to prevent flooding serial
                time.sleep(0.05)

            # ----------------------------------
            # PRIORITY 3: SEARCHING (NO BALL)
            # ----------------------------------
            else:
                # No ball seen? Rotate to find one.
                # "Step 1... rotate find ping-pong balls"
                # print("Searching...")
                # log_motion_action("Spin Search", latest_L, latest_C, latest_R) # Optional: reduce log spam
                robot_spin_search()
                time.sleep(0.05)  # pulse spin to rotate slowly
                robot_stop()
                time.sleep(0.15)  # give camera time to catch the ball

        else:
            # If not active, do nothing
            time.sleep(0.1)
    
    # Cleanup
    robot_stop()


# --- MAIN ENTRY ---
if __name__ == "__main__":
    print("---------------------------------------")
    print("      ROBOT BALL CHASER ACTIVATED      ")
    print(f"      Button Pin: {BUTTON_PIN}")
    print("---------------------------------------")

    # Start Sensors Thread
    t_sensors = threading.Thread(target=update_sensors)
    t_sensors.daemon = True
    t_sensors.start()

    # Start Robot Control Thread
    t_control = threading.Thread(target=robot_control_loop)
    t_control.daemon = True
    t_control.start()

    # Start Camera/YOLO (Run in main thread as it needs GUI)
    camera_yolo_loop()

    # Ensure clean exit
    running = False
    robot_stop()
    print("Exiting...")
