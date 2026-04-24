import cv2
import time
import serial
import serial.tools.list_ports
import threading
import datetime
import os
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

# Camera Configuration
CAMERA_TYPE = "usb" # Options: "picamera" or "usb"
USB_CAMERA_PATH = "/dev/video2"  # Pass the explicit device path

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
TARGET_REACHED_CM = 60.0 # Distance to stop in front of the ball


# --- GLOBAL VARIABLES ---
# Thread Control
running = True
robot_active = False # Start paused, wait for button press
execute_ball_catch = False
arm_is_up = True     # Track arm state (True=UP, False=DOWN)

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
        ser = serial.Serial(SERIAL_PORT, 115200, timeout=0)
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
    if ser: ser.write(b'S\n')

def set_speed(level):
    # level can be 1 (slow), 2 (medium), 3 (fast)
    if ser: ser.write(f"{level}\n".encode())

def robot_forward():
    if ser: 
        set_speed(2) # Normal speed for forward
        ser.write(b'F\n')

def robot_backward():
    if ser: 
        set_speed(2)
        ser.write(b'B\n')

def robot_left():
    if ser: 
        set_speed(1) # Slower speed (1) for precise tracking
        ser.write(b'L\n')

def robot_right():
    if ser: 
        set_speed(1) # Slower speed (1) for precise tracking
        ser.write(b'R\n')

def robot_spin_search():
    # Spin slowly to find ball.
    if ser:
        set_speed(1) # Slower speed (1) to systematically scan for balls
        ser.write(b'R\n')

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
    global running, robot_active, execute_ball_catch, arm_is_up, ball_visible, ball_x, ball_y, ball_w, ball_h, ball_distance_cm
    
    show_grid = False

    # 1. Initialize Camera
    cap = None
    picam2 = None
    try:
        if CAMERA_TYPE == "picamera":
            picam2 = Picamera2()
            config = picam2.create_video_configuration(
                main={"size": (CAPTURE_WIDTH, CAPTURE_HEIGHT), "format": "RGB888"},
                transform=Transform(hflip=1, vflip=1),
                buffer_count=12
            )
            picam2.configure(config)
            picam2.start()
            print("PiCamera Started.")
        elif CAMERA_TYPE == "usb":
            cap = cv2.VideoCapture(USB_CAMERA_PATH, cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAPTURE_WIDTH)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_HEIGHT)
            if not cap.isOpened():
                raise Exception(f"Cannot open USB camera at {USB_CAMERA_PATH}")
            print(f"USB Camera Started ({USB_CAMERA_PATH}).")
        else:
            raise Exception("Invalid CAMERA_TYPE specified.")
    except Exception as e:
        print(f"Camera Initialize Error: {e}")
        return

    # 2. Load Model
    try:
        print(f"Loading YOLO model: {MODEL_PATH}")
        model = YOLO(MODEL_PATH)
        print("Model Loaded.")
    except Exception as e:
        print(f"YOLO Error: {e}")
        if picam2: picam2.stop()
        if cap: cap.release()
        return

    print("Starting Detection Loop...")
    prev_time = time.time()
    frame_counter = 0
    last_results = None
    
    while running:
        try:
            # Capture
            if CAMERA_TYPE == "picamera":
                frame_bgr = picam2.capture_array()
                # PiCamera v3 with libcamera outputs BGR despite "RGB888" format
                # No conversion needed - frame is already in BGR format for OpenCV
            else:
                ret, frame = cap.read()
                if not ret:
                    print("Failed to grab frame from USB camera. Reconnecting...")
                    time.sleep(0.5)
                    continue
                # Optional: Flip USB camera if it's mounted upside down
                # frame = cv2.flip(frame, -1) # -1 is both axes
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
                        w_f, h_f = float(w), float(h)
                        aspect_ratio = w_f / h_f if h_f > 0 else 0
                        
                        area = w_f * h_f
                        # Filter by aspect ratio (approx 1:1 for a sphere) to avoid merging multiple balls
                        if 0.7 <= aspect_ratio <= 1.3:
                            if area > max_area:
                                max_area = area
                                largest_box = (float(x), float(y), w_f, h_f)
            
            # Update Shared State
            if largest_box:
                ball_visible = True
                ball_x, ball_y, ball_w, ball_h = largest_box
                
                # Calculate distance based on bounding box Area
                ball_area = ball_w * ball_h
                
                # TODO: Replace these placeholder areas with actual bounding box areas at these distances!
                # Example: If area is 12000 => 15cm, area is 3000 => 30cm, etc.
                area_calibration = {
                    7000.0: 15.0,
                    2000.0: 30.0,
                    850.0: 45.0,
                    450.0: 60.0,
                    290.0: 75.0,
                    180.0: 90.0,
                    70.0: 150.0
                }
                
                # Simple linear interpolation for the given ball_area
                sorted_a = sorted(area_calibration.keys())
                if ball_area <= sorted_a[0]:
                    ball_distance_cm = area_calibration[sorted_a[0]]
                elif ball_area >= sorted_a[-1]:
                    ball_distance_cm = area_calibration[sorted_a[-1]]
                else:
                    # Find which two points ball_area falls between
                    for i in range(len(sorted_a) - 1):
                        a1, a2 = sorted_a[i], sorted_a[i+1]
                        if a1 <= ball_area <= a2:
                            d1, d2 = area_calibration[a1], area_calibration[a2]
                            # Linear interpolate the distance
                            ball_distance_cm = d1 + (d2 - d1) * ((ball_area - a1) / (a2 - a1))
                            break
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
            
            if ball_visible:
                # Calculate bounding box coordinates for the tracked target
                tl_x = int(ball_x - ball_w / 2)
                tl_y = int(ball_y - ball_h / 2)
                br_x = int(ball_x + ball_w / 2)
                br_y = int(ball_y + ball_h / 2)
                ball_area = ball_w * ball_h
                
                # Draw a distinct cyan bounding box and text overlay for the targeted ball
                cv2.rectangle(annotated_frame, (tl_x, tl_y), (br_x, br_y), (255, 255, 0), 3)
                cv2.putText(annotated_frame, f"TARGET: {ball_distance_cm:.1f}cm", (max(0, tl_x), max(20, tl_y - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                
                top_text = f"FPS: {int(fps)} | Dist: {ball_distance_cm:.1f}cm"
                area_text = f"Area: {int(ball_area)}"
            else:
                top_text = f"FPS: {int(fps)} | Dist: No Target"
                area_text = ""

            cv2.putText(annotated_frame, top_text, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            if area_text:
                cv2.putText(annotated_frame, area_text, (10, 55),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            if show_grid:
                # Draw horizontal grid lines
                for grid_y in [120, 150, 180, 200, 240]:
                    cv2.line(annotated_frame, (0, grid_y), (CAPTURE_WIDTH, grid_y), (255, 255, 255), 1)
                    cv2.putText(annotated_frame, f"y={grid_y}", (5, grid_y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

            cv2.imshow("Robot Vision", annotated_frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('g') or key == ord('G'):
                show_grid = not show_grid
                print(f"Grid toggled: {show_grid}")
            elif key == ord('q'):
                if ser:
                    print("Q pressed! Executing ARM DOWN and Gripper OPEN...")
                    ser.write(b'a\n')
                    time.sleep(1)
                    ser.write(b'O\n')
                    time.sleep(1)
                running = False
                break
            elif key == ord('y') and not robot_active:
                if not arm_is_up and ser:
                    print("\n>>> Arm is DOWN. Moving Arm UP before starting... <<<")
                    ser.write(b'A\n')
                    arm_is_up = True
                    time.sleep(1.0)  # Wait for the arm to raise
                robot_active = True
                print("\n>>> ACTIVE: Robot Started by 'y' key. Searching for ball... <<<")
            elif key == ord('b') or key == ord('B'):
                execute_ball_catch = True
                print("\n>>> Ball Catch Manoeuvre Initialized by 'B' key. <<<")
            elif key == ord('u') or key == ord('U'):
                if ser:
                    ser.write(b'A\n')
                    arm_is_up = True
                    print("\n>>> Arm UP (Obstacle Avoidance ENABLED) <<<")
            elif key == ord('j') or key == ord('J'):
                if ser:
                    ser.write(b'a\n')
                    arm_is_up = False
                    print("\n>>> Arm DOWN (Obstacle Avoidance DISABLED) <<<")
                
        except Exception as e:
            print(f"Detection Loop Error: {e}")
            time.sleep(0.1)

    if picam2: picam2.stop()
    if cap: cap.release()
    cv2.destroyAllWindows()


# --- THREAD 3: ROBOT CONTROL ---
def robot_control_loop():
    global running, robot_active, execute_ball_catch, arm_is_up
    global latest_L, latest_C, latest_R
    global ball_visible, ball_x, ball_distance_cm
    
    print("Robot Control Loop Started.")
    
    while running:
        if execute_ball_catch and ser:
            execute_ball_catch = False
            was_active = robot_active
            robot_active = False # Pause normal control
            robot_stop()
            time.sleep(0.1)

            print("\n[BALL CATCH] Checking path...")
            if latest_C > 30: # Assuming 30cm is the threshold
                print(f"[BALL CATCH] Path clear ({latest_C}cm). Moving forward ~30cm.")
                robot_forward()
                time.sleep(1.0) # Approx time to move 30cm
                robot_stop()
                time.sleep(0.2)
            else:
                print(f"[BALL CATCH] Path blocked ({latest_C}cm). Skipping initial forward move.")

            print("\n[BALL CATCH] Executing Manoeuvre...")
            ser.write(b'S\n') # Ensure stopped first
            time.sleep(0.1)
            ser.write(b'a\n') # Arm DOWN
            arm_is_up = False # Disable obstacle avoidance
            time.sleep(2.0)
            ser.write(b'O\n') # Gripper OPEN
            time.sleep(1.0)
            
            ser.write(b'F\n') # Move forward
            time.sleep(1.5) # Short interval
            ser.write(b'C\n') # Close gripper
            time.sleep(1.0) # Wait for close
            ser.write(b'S\n') # Stop
            time.sleep(0.5)
            
            for _ in range(3):
                ser.write(b'F\n') # Move forwards
                time.sleep(1.0)
                ser.write(b'S\n') # Stop to scoop
                time.sleep(0.2)
                ser.write(b'O\n') # Open gripper
                time.sleep(1.0)
                ser.write(b'C\n') # Close gripper
                time.sleep(1.0)
            
            robot_active = was_active # Restore previous state
            print("\n[BALL CATCH] Manoeuvre Complete.")
            continue

        if robot_active and ser:
            # ----------------------------------
            # PRIORITY 1: OBSTACLE AVOIDANCE (Only if Arm is UP)
            # ----------------------------------
            if arm_is_up:
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
                    robot_active = False
                    # Wait slightly so we don't rapid-fire commands once arrived
                    time.sleep(0.5)
                elif abs(error) < CENTER_TOLERANCE:
                    # Ball is roughly centered, move towards it
                    robot_forward()
                elif error < 0:
                    # Ball is to the Left (x < 160)
                    log_motion_action("Turn LEFT (Track)", latest_L, latest_C, latest_R)
                    robot_left()
                else:
                    # Ball is to the Right (x > 160)
                    log_motion_action("Turn RIGHT (Track)", latest_L, latest_C, latest_R)
                    robot_right()
                
                # Short delay to prevent flooding serial, motors run continuously
                time.sleep(0.1)

            # ----------------------------------
            # PRIORITY 3: SEARCHING (NO BALL)
            # ----------------------------------
            else:
                # No ball seen? Rotate to find one.
                robot_spin_search()
                time.sleep(0.1)  # Run motors continuously while searching

        else:
            # If not active, do nothing
            time.sleep(0.1)
    
    # Cleanup
    robot_stop()


# --- MAIN ENTRY ---
if __name__ == "__main__":
    print("---------------------------------------")
    print("      ROBOT BALL CHASER ACTIVATED      ")
    print("Waiting for 'y' input to start movement...")
    print("---------------------------------------")

    # Execute ARM UP before moving
    if ser:
        print("Executing ARM UP before moving...")
        ser.write(b'A\n')
        time.sleep(1)

    # Start Sensors Thread
    t_sensors = threading.Thread(target=update_sensors)
    t_sensors.daemon = True
    t_sensors.start()

    # Start Robot Control Thread
    t_control = threading.Thread(target=robot_control_loop)
    t_control.daemon = True
    t_control.start()

    try:
        # Start Camera/YOLO (Run in main thread as it needs GUI)
        camera_yolo_loop()
    except KeyboardInterrupt:
        print("\nCtrl+C pressed!")
        if ser:
            print("Executing ARM DOWN and Gripper OPEN...")
            ser.write(b'a\n')
            time.sleep(1)
            ser.write(b'O\n')
            time.sleep(1)
    finally:
        # Ensure clean exit
        running = False
        robot_stop()
        print("Exiting...")
