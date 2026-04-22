import cv2
import time
import serial
import serial.tools.list_ports
import threading
import datetime
import os
from gpiozero import Button
from picamera2 import Picamera2
from libcamera import Transform
from ultralytics import YOLO

# --- CONFIGURATION ---
def find_arduino_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if 'ttyACM' in port.device or 'ttyUSB' in port.device:
            return port.device
    return None

SERIAL_PORT = find_arduino_port()
BUTTON_PIN = 25

# Model Selection
NCNN_MODEL = "/home/pi/yolo_project/orange_ball_ncnn_model"
PT_MODEL = "/home/pi/yolo_project/orange_ball.pt"

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

# Camera Settings & Distance Calculation
CAPTURE_WIDTH = 320
CAPTURE_HEIGHT = 240
CENTER_X = CAPTURE_WIDTH // 2
SKIP_FRAMES = 3  # Run detection 1 out of every N frames (higher = faster FPS)

# Camera Calibration Parameters (from go_to_home.py, tuned for 640x480)
CAMERA_PARAMS = (907.462397724348, 908.550833315007, 358.40056240558073, 246.47297678800183)
ORIGINAL_CALIB_WIDTH = 640

# Scale the focal length (fx) to match the current capture resolution (320x240)
FOCAL_LENGTH_X = CAMERA_PARAMS[0] * (CAPTURE_WIDTH / ORIGINAL_CALIB_WIDTH)
BALL_DIAMETER_CM = 4.0 # Standard ping-pong ball size (~4.0 cm). Adjust if needed.
TARGET_REACHED_CM = 30.0 # Distance to stop in front of the ball


# --- GLOBAL VARIABLES ---
# Thread Control
running = True
robot_active = True

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
ball_distance_cm = 999.0 # Store calculated distance

# Serial Connection
try:
    if SERIAL_PORT:
        ser = serial.Serial(SERIAL_PORT, 9600, timeout=1)
        ser.reset_input_buffer()
        print(f"Connected to Serial: {SERIAL_PORT}")
        time.sleep(2) # Allow Arduino to reset
    else:
        print("No Arduino port found (ttyACM/ttyUSB).")
        ser = None
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
    global running, ball_visible, ball_x, ball_y, ball_w, ball_h, ball_distance_cm
    
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
    prev_time = time.time()
    frame_counter = 0
    last_results = None
    
    while running:
        try:
            # Capture
            frame = picam2.capture_array()
            
            # PiCamera v3 with libcamera outputs BGR despite "RGB888" format
            # No conversion needed - frame is already in BGR format for OpenCV
            frame_bgr = frame
            frame_counter += 1
            
            # Run inference only every SKIP_FRAMES frames
            if frame_counter % SKIP_FRAMES == 0:
                results = model.predict(frame_bgr, imgsz=320, verbose=False, conf=0.4)
                last_results = results
            
            # Process Detections using last_results (allows tracking even on skipped frames)
            # We want to find the largest ball (closest)
            largest_box = None
            max_area = 0
            
            if last_results and len(last_results) > 0:
                for r in last_results:
                    boxes = r.boxes
                    for box in boxes:
                        # Log detection (only on detection frames)
                        if frame_counter % SKIP_FRAMES == 0:
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
                # Calculate distance using pinhole camera model: Distance = (Real_Size * Focal_Length) / Pixel_Size
                ball_distance_cm = (BALL_DIAMETER_CM * FOCAL_LENGTH_X) / ball_w
            else:
                ball_visible = False
                ball_distance_cm = 999.0
            
            # Calculate FPS
            current_time = time.time()
            fps = 1 / (current_time - prev_time)
            prev_time = current_time
            
            # Visualization
            if last_results:
                annotated_frame = last_results[0].plot()
            else:
                annotated_frame = frame_bgr.copy()
            cv2.putText(annotated_frame, f"FPS: {int(fps)} | Dist: {ball_distance_cm:.1f}cm", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
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
    global ball_visible, ball_x, ball_distance_cm
    
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
                
                # print(f"Tracking Ball: x={ball_x:.1f}, dist={ball_distance_cm:.1f}cm")
                
                if ball_distance_cm <= TARGET_REACHED_CM:
                    # Ball is very close, stop!
                    print(f"Ball Reached! Distance: {ball_distance_cm:.1f}cm. Stopping.")
                    log_motion_action(f"Ball Reached ({ball_distance_cm:.1f}cm)", latest_L, latest_C, latest_R)
                    robot_stop()
                    # Wait slightly so we don't rapid-fire commands once arrived
                    time.sleep(0.5)
                elif abs(error) < CENTER_TOLERANCE:
                    # Ball is roughly centered, move towards it
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
