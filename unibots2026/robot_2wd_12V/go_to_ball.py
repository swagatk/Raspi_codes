import cv2
import time
import serial
import serial.tools.list_ports
import threading
import datetime
import os
import numpy as np
from picamera2 import Picamera2
from libcamera import Transform
from ultralytics import YOLO
try:
    from pupil_apriltags import Detector as PoseDetector
except ImportError:
    print("Warning: pupil_apriltags not found! Tag detection will fail.")


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
STOP_DIST = 30        # cm
SIDE_DIST = 40        # cm
CENTER_TOLERANCE = 40 # pixels from center to consider "centered"

# Camera Configuration
CAMERA_TYPE = "usb" # Options: "picamera" or "usb"

def find_usb_camera():
    import glob
    video_devices = sorted(glob.glob('/dev/video*'))
    for dev in video_devices:
        try:
            # Skip codec/isp devices (usually video10 and above)
            num = int(dev.replace('/dev/video', ''))
            if num >= 10:
                continue
        except ValueError:
            continue
            
        print(f"Testing {dev}...")
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if cap.isOpened():
            ret, _ = cap.read()
            cap.release()
            if ret:
                print(f"Found working camera at {dev}")
                return dev
    return "/dev/video2" # Fallback

USB_CAMERA_PATH = find_usb_camera()

# Camera Settings & Distance Calculation
CAPTURE_WIDTH = 320
CAPTURE_HEIGHT = 240
CENTER_X = CAPTURE_WIDTH // 2
SKIP_FRAMES = 3  # Run detection 1 out of every N frames (higher = faster FPS)

# Camera Calibration Parameters (from go_to_home.py, tuned for 640x480)
if CAMERA_TYPE == "picamera":
    CAMERA_PARAMS = (907.462397724348, 908.550833315007, 358.40056240558073, 246.47297678800183)
else:
    CAMERA_PARAMS = (742.843247995633, 743.2228374107693, 322.3205884283167, 234.06623771807327)
ORIGINAL_CALIB_WIDTH = 640

# Scale the focal length (fx) to match the current capture resolution (320x240)
FOCAL_LENGTH_X = CAMERA_PARAMS[0] * (CAPTURE_WIDTH / ORIGINAL_CALIB_WIDTH)
BALL_DIAMETER_CM = 4.0 # Standard ping-pong ball size (~4.0 cm). Adjust if needed.
TARGET_REACHED_CM = 30.0 # Distance to stop in front of the ball
CAPTURE_DISTANCE_CM = 15.0 # Target forward travel distance during the ball catch manoeuver
CAPTURE_SPEED_CM_S = 10.6 # Approximate cm/sec speed of robot using motor speed level '2'
CAPTURE_GRIPPER_CYCLES = 4 # Number of open-close gripper operations during forward movement

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

# Tag Detection (Shared)
tag_visible = False
tag_distance_cm = 999.0
tag_yaw_deg = 0.0
top_camera_frame = None  # To share the frame with the main loop
top_camera_tags = []     # Store tag data for rendering

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
        print(f"Moving FORWARD | Sensors - L:{latest_L} C:{latest_C} R:{latest_R}")
        set_speed(2) # Normal speed for forward
        ser.write(b'F\n')

def robot_backward():
    if ser: 
        print(f"Moving BACKWARD | Sensors - L:{latest_L} C:{latest_C} R:{latest_R}")
        set_speed(2)
        ser.write(b'B\n')

def robot_left():
    if ser: 
        print(f"Moving LEFT | Sensors - L:{latest_L} C:{latest_C} R:{latest_R}")
        set_speed(1) # Slower speed (1) for precise tracking
        ser.write(b'L\n')

def robot_right():
    if ser: 
        print(f"Moving RIGHT | Sensors - L:{latest_L} C:{latest_C} R:{latest_R}")
        set_speed(1) # Slower speed (1) for precise tracking
        ser.write(b'R\n')

def robot_forward_slow():
    if ser:
        print(f"Moving FORWARD (Slow) | Sensors - L:{latest_L} C:{latest_C} R:{latest_R}")
        set_speed(1) # Slower speed (1) for searching safely
        ser.write(b'F\n')

def robot_spin_search():
    # Spin slowly to find ball.
    if ser:
        print(f"Spin SEARCHING | Sensors - L:{latest_L} C:{latest_C} R:{latest_R}")
        set_speed(2) # Increased to speed(2) to ensure motors have enough torque to rotate
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
        model = YOLO(MODEL_PATH, task='detect')
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
    
    # Target stabilization tracking
    consecutive_detections = 0
    consecutive_misses = 0
    CONFIRM_FRAMES = 3   # Frames required to confirm target
    MISS_FRAMES_TOLERANCE = 5 # Frames to keep target despite missing
    current_target_box = None
    
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
            # We want to find the largest ball (closest) that matches previous target if it exists
            largest_box = None
            max_area = 0
            candidate_boxes = []

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
                        
                        # Filter by wildly relaxed aspect ratio (handle wide/tall boxes from grouped balls)
                        # Two balls side-by-side easily exceeds a 2.0 aspect ratio due to bounding box margins
                        if 0.2 <= aspect_ratio <= 5.0:
                            candidate_boxes.append((float(x), float(y), w_f, h_f, area))

            # Advanced target stabilization and debouncing
            best_match = None
            if candidate_boxes:
                # Find absolutely largest candidate in the current frame
                largest_candidate = max(candidate_boxes, key=lambda b: b[4])

                # Find candidate closest to existing target
                closest_candidate = None
                if current_target_box:
                    cx, cy = current_target_box[0], current_target_box[1]
                    best_dist = 80 # Max search radius in pixels
                    for b in candidate_boxes:
                        dist = ((b[0] - cx)**2 + (b[1] - cy)**2)**0.5
                        if dist < best_dist:
                            best_dist = dist
                            closest_candidate = b
                            
                if current_target_box and closest_candidate:
                    # Prioritize nearer objects: If the largest candidate in frame is significantly 
                    # larger (e.g., >1.5x area) than the currently tracked ball, SWITCH to it!
                    if largest_candidate[4] > closest_candidate[4] * 1.5:
                        best_match = largest_candidate
                        consecutive_detections = 1 # Reset confidence count for the new target
                    else:
                        best_match = closest_candidate
                else:
                    # No target lock, or target lost -> pick the largest ball available
                    best_match = largest_candidate

            if best_match:
                consecutive_misses = 0
                consecutive_detections += 1
                
                # Apply Exponential Moving Average (EMA) for smoother box tracking
                if current_target_box is None:
                    current_target_box = (best_match[0], best_match[1], best_match[2], best_match[3])
                else:
                    alpha = 0.5 # Smoothing factor
                    current_target_box = (
                        alpha * best_match[0] + (1 - alpha) * current_target_box[0],
                        alpha * best_match[1] + (1 - alpha) * current_target_box[1],
                        alpha * best_match[2] + (1 - alpha) * current_target_box[2],
                        alpha * best_match[3] + (1 - alpha) * current_target_box[3]
                    )

                # Only output visible ball if we've seen it for enough frames
                if consecutive_detections >= CONFIRM_FRAMES:
                    largest_box = current_target_box
            else:
                consecutive_misses += 1
                if consecutive_misses > MISS_FRAMES_TOLERANCE:
                    # Give up on the target
                    current_target_box = None
                    consecutive_detections = 0
                    largest_box = None
                else:
                    # Tolerate the miss and keep using the last known target box
                    if consecutive_detections >= CONFIRM_FRAMES:
                        largest_box = current_target_box

            # Update Shared State
            if largest_box:
                ball_visible = True
                ball_x, ball_y, ball_w, ball_h = largest_box
                
                # Calculate distance reliably using focal length instead of bounding box area
                # Distance = (Real Diameter * Focal Length) / Pixel Diameter
                # CRITICAL FIX: We use min(ball_w, ball_h) instead of max().
                # If YOLO merges 2 balls side-by-side, the bounding box width doubles, 
                # but the bounding box height remains the exact diameter of a single ball!
                # min() ensures we always grab the true 4cm diameter edge.
                pixel_diameter = min(ball_w, ball_h)
                if pixel_diameter > 0:
                    ball_distance_cm = (BALL_DIAMETER_CM * FOCAL_LENGTH_X) / pixel_diameter
                else:
                    ball_distance_cm = 999.0
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

            # Show top camera feed if arm is DOWN
            if not arm_is_up and top_camera_frame is not None:
                arm_view = top_camera_frame.copy()
                
                # Annotate C sensor reading
                cv2.putText(arm_view, f"C Sensor: {latest_C} cm", (20, 40), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
                
                # Annotate tag data
                if top_camera_tags:
                    for tag in top_camera_tags:
                        dist_cm = (tag.pose_t[2][0] / 3.0) * 100.0
                        corners = tag.corners.astype(int)
                        # Draw bounding box around tag
                        for i in range(4):
                            cv2.line(arm_view, tuple(corners[i]), tuple(corners[(i+1)%4]), (0, 255, 0), 2)
                        
                        center = tuple(tag.center.astype(int))
                        cv2.circle(arm_view, center, 4, (0, 0, 255), -1)
                        cv2.putText(arm_view, f"ID: {tag.tag_id} ({dist_cm:.1f}cm)", 
                                    (corners[0][0], corners[0][1] - 10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                            
                cv2.imshow("Top Camera (Arm DOWN)", arm_view)
            else:
                try:
                    if cv2.getWindowProperty("Top Camera (Arm DOWN)", cv2.WND_PROP_VISIBLE) >= 0:
                        cv2.destroyWindow("Top Camera (Arm DOWN)")
                except cv2.error:
                    pass
            
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
                if latest_L < 50 or latest_C < 50 or latest_R < 50:
                    print(f"Too close to an obstacle, can not lower the arm (L:{latest_L} C:{latest_C} R:{latest_R})")
                elif ser:
                    ser.write(b'a\n')
                    arm_is_up = False
                    print("\n>>> Arm DOWN (Obstacle Avoidance DISABLED) <<<")
                
        except Exception as e:
            print(f"Detection Loop Error: {e}")
            time.sleep(0.1)

    if picam2: picam2.stop()
    if cap: cap.release()
    cv2.destroyAllWindows()


# --- THREAD 3: PICAMERA & TAGS ---
def picamera_tag_loop():
    global running, tag_visible, tag_distance_cm, tag_yaw_deg, top_camera_frame, top_camera_tags
    
    try:
        picam2 = Picamera2()
        config = picam2.create_video_configuration(
            main={"size": (320, 240), "format": "RGB888"},
            transform=Transform(hflip=1, vflip=1),
            controls={"FrameRate": 30},
            buffer_count=2
        )
        picam2.configure(config)
        picam2.start()
        print("PiCamera for Tags Started (320x240).")
    except Exception as e:
        print(f"PiCamera Tag Initialize Error: {e}")
        return

    try:
        apriltag_detector = PoseDetector(families='tagStandard41h12', quad_decimate=1.0) # Down from 2.0 because image is already smaller
    except Exception as e:
        print(f"Failed to initialize PoseDetector: {e}")
        if picam2: picam2.stop()
        return

    # Scaled down by half since resolution is 320x240 instead of 640x480
    orig_params = (954.4949188171072, 955.5979729485147, 332.0798756650343, 245.67451277016548)
    CAMERA_PARAMS = (orig_params[0] / 2.0, orig_params[1] / 2.0, orig_params[2] / 2.0, orig_params[3] / 2.0)
    TAG_SIZE = 0.10
    SCALE = 3.0

    frame_skip = 3
    frame_count = 0

    while running:
        try:
            frame_bgr = picam2.capture_array()
            frame_count += 1
            if frame_count % frame_skip != 0:
                continue

            gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
            tags = apriltag_detector.detect(gray, estimate_tag_pose=True,
                                            camera_params=CAMERA_PARAMS, tag_size=TAG_SIZE)
            
            if tags:
                closest_dist_m = 999.0
                closest_yaw = 0.0
                
                for tag in tags:
                    dist_m = tag.pose_t[2][0] / SCALE
                    if dist_m < closest_dist_m:
                        closest_dist_m = dist_m
                        R = tag.pose_R
                        yaw = np.arctan2(-R[2][0], np.sqrt(R[2][1]**2 + R[2][2]**2))
                        closest_yaw = np.degrees(yaw)
                
                tag_distance_cm = closest_dist_m * 100.0
                tag_yaw_deg = closest_yaw
                tag_visible = True
            else:
                tag_visible = False
                tag_distance_cm = 999.0
            
            top_camera_tags = tags
            top_camera_frame = frame_bgr.copy()
            
        except Exception as e:
            # print(f"Tag Loop Error: {e}")
            time.sleep(0.01)
            
        time.sleep(0.01)

    if picam2: picam2.stop()


# --- THREAD 4: ROBOT CONTROL ---
def robot_control_loop():
    global running, robot_active, execute_ball_catch, arm_is_up
    global latest_L, latest_C, latest_R
    global ball_visible, ball_x, ball_distance_cm
    global tag_visible, tag_distance_cm, tag_yaw_deg
    
    print("Robot Control Loop Started.")
    obstacle_streak = 0
    
    while running:
        if execute_ball_catch and ser:
            execute_ball_catch = False
            
            if latest_L < 50 or latest_C < 50 or latest_R < 50:
                print(f"Too close to an obstacle, can not lower the arm (L:{latest_L} C:{latest_C} R:{latest_R})")
                continue
                
            was_active = robot_active
            robot_active = False # Pause normal control
            robot_stop()
            time.sleep(0.1)

            print("\n[BALL CATCH] Executing Manoeuvre...")
            ser.write(b'S\n') # Ensure stopped first
            time.sleep(0.1)
            ser.write(b'a\n') # Arm DOWN
            arm_is_up = False # Disable obstacle avoidance
            time.sleep(2.0)
            
            ser.write(b'O\n') # Gripper OPEN initially
            time.sleep(1.0)

            print(f"[BALL CATCH] Moving forward {CAPTURE_DISTANCE_CM}cm while opening/closing gripper...")
            # Move forward slowly (speed '2') for specified distance while opening and closing
            set_speed(2)
            ser.write(b'F\n')
            
            time_to_move = CAPTURE_DISTANCE_CM / CAPTURE_SPEED_CM_S
            
            # Calculate toggle interval based on desired number of cycles (2 toggles per cycle)
            if CAPTURE_GRIPPER_CYCLES > 0:
                toggle_interval = time_to_move / (CAPTURE_GRIPPER_CYCLES * 2)
            else:
                toggle_interval = time_to_move + 1 # Never toggle
                
            start_time = time.time()
            last_toggle = start_time
            gripper_state = b'O\n'
            
            while (time.time() - start_time) < time_to_move:
                current_time = time.time()
                # Toggle gripper based on calculated interval
                if current_time - last_toggle >= toggle_interval:
                    if gripper_state == b'O\n':
                        ser.write(b'C\n')
                        gripper_state = b'C\n'
                    else:
                        ser.write(b'O\n')
                        gripper_state = b'O\n'
                    last_toggle = current_time
                time.sleep(0.02)
            
            ser.write(b'C\n') # Final state: closed
            time.sleep(0.5)
            ser.write(b'S\n') # Stop robot
            
            time.sleep(0.5)
            ser.write(b'A\n') # Arm UP
            arm_is_up = True
            time.sleep(2.0)
            
            robot_active = was_active # Restore previous state
            print("\n[BALL CATCH] Manoeuvre Complete.")
            continue

        if robot_active and ser:
            # ----------------------------------
            # PRIORITY 1: OBSTACLE AVOIDANCE (Only if Arm is UP)
            # ----------------------------------
            if arm_is_up:
                if latest_C < STOP_DIST or latest_L < SIDE_DIST or latest_R < SIDE_DIST:
                    obstacle_streak += 1
                    log_motion_action(f"OBSTACLE (L:{latest_L} C:{latest_C} R:{latest_R})", latest_L, latest_C, latest_R)
                    print(f"OBSTACLE DETECTED! (L:{latest_L}, C:{latest_C}, R:{latest_R}) Avoiding... (Attempt {obstacle_streak})")
                    robot_stop()
                    time.sleep(0.1)
                    
                    # Corner Escape Maneuver
                    if obstacle_streak >= 3:
                        print("Stuck in corner! Backing up and rotating to escape...")
                        log_motion_action("CORNER ESCAPE", latest_L, latest_C, latest_R)
                        
                        # Strong backup
                        set_speed(2)
                        if ser: ser.write(b'B\n')
                        time.sleep(0.7)
                        
                        # ~90 degree rotation (approx 1.2s on speed 2)
                        set_speed(2)
                        if latest_L < latest_R:
                            if ser: ser.write(b'R\n')
                        else:
                            if ser: ser.write(b'L\n')
                        time.sleep(1.2)
                        
                        robot_stop()
                        time.sleep(0.5)
                        obstacle_streak = 0
                        continue
                    
                    if latest_C < STOP_DIST:
                        robot_backward()
                        time.sleep(0.3)
                    
                    # Turn slowly away from the closest obstacle
                    if latest_L < latest_R:
                        log_motion_action("Turn RIGHT (Avoid)", latest_L, latest_C, latest_R)
                        robot_right()
                    else:
                        log_motion_action("Turn LEFT (Avoid)", latest_L, latest_C, latest_R)
                        robot_left()
                        
                    time.sleep(0.6) # Turn slowly to avoid obstacle
                    robot_stop()
                    time.sleep(0.5) # Allow camera and sensors to stabilize before searching again
                    continue # Skip rest of loop to re-evaluate sensors
                else:
                    # Clear streak if path is clear
                    obstacle_streak = 0

            else:
                # ----------------------------------
                # OBSTACLE AVOIDANCE (Arm is DOWN)
                # ----------------------------------
                # Ignore L and R sensors. Use Center US and Tag Detection.
                if latest_C < 50 or (tag_visible and tag_distance_cm < 50):
                    log_motion_action(f"BLOCKED ARM DOWN (C:{latest_C}cm, Tag:{tag_distance_cm:.1f}cm)", latest_L, latest_C, latest_R)
                    print(f"OBSTACLE AHEAD (Arm DOWN)! Avoiding... C:{latest_C}cm Tag:{tag_distance_cm:.1f}cm")
                    robot_stop()
                    time.sleep(0.1)
                    robot_backward()
                    time.sleep(0.3)
                    
                    if tag_visible and tag_distance_cm < 50:
                        # Turn away from tag wall
                        if tag_yaw_deg > 0:
                            log_motion_action(f"Turn LEFT (Avoid Tag Yaw {tag_yaw_deg:.1f})", latest_L, latest_C, latest_R)
                            robot_left()
                        else:
                            log_motion_action(f"Turn RIGHT (Avoid Tag Yaw {tag_yaw_deg:.1f})", latest_L, latest_C, latest_R)
                            robot_right()
                    else:
                        # Default turn when blind/no tag
                        log_motion_action("Turn RIGHT (Blind Avoid)", latest_L, latest_C, latest_R)
                        robot_right()
                        
                    time.sleep(0.5)
                    robot_stop()
                    continue

            # ----------------------------------
            # PRIORITY 2: BALL TRACKING
            # ----------------------------------
            if ball_visible:
                # Calculate error from center
                # ball_x is 0..320. Center is 160.
                error = ball_x - CENTER_X
                
                # print(f"Tracking Ball: x={ball_x:.1f}, dist={ball_distance_cm:.1f}cm")
                
                if abs(error) >= CENTER_TOLERANCE:
                    # Ball is NOT centered, turn to center it first (even if it's close)
                    if error < 0:
                        # Ball is to the Left (x < 160)
                        log_motion_action("Turn LEFT (Track)", latest_L, latest_C, latest_R)
                        robot_left()         # Turn towards the ball
                        time.sleep(0.06)     # Short nudge to adjust angle
                        robot_stop()         # Stop immediately to re-evaluate center natively
                        time.sleep(0.2)      # Longer wait for camera stabilization to avoid overshooting
                    else:
                        # Ball is to the Right (x > 160)
                        log_motion_action("Turn RIGHT (Track)", latest_L, latest_C, latest_R)
                        robot_right()        # Turn towards the ball
                        time.sleep(0.06)     # Short nudge to adjust angle
                        robot_stop()         # Stop immediately to re-evaluate center natively
                        time.sleep(0.2)      # Longer wait for camera stabilization to avoid overshooting
                elif ball_distance_cm <= TARGET_REACHED_CM:
                    # Ball is roughly centered AND very close, stop!
                    print(f"Ball Reached! Distance: {ball_distance_cm:.1f}cm. Stopping.")
                    log_motion_action(f"Ball Reached ({ball_distance_cm:.1f}cm)", latest_L, latest_C, latest_R)
                    robot_stop()
                    robot_active = False
                    # Wait slightly so we don't rapid-fire commands once arrived
                    time.sleep(0.5)
                else:
                    # Ball is roughly centered but still far, move towards it straight
                    log_motion_action("Forward (Track)", latest_L, latest_C, latest_R)
                    robot_forward_slow()
                    time.sleep(0.5)      # Move forward longer to close distance
                    robot_stop()         # Stop to re-evaluate
                    time.sleep(0.1)      # Brief stabilization

            # ----------------------------------
            # PRIORITY 3: SEARCHING (NO BALL)
            # ----------------------------------
            else:
                # No ball seen? Stop and go back to searching mode.
                print("Target lost. Stopping and waiting to re-acquire...")
                robot_stop()
                robot_active = False

        else:
            # If not active, but we don't see a ball, spin and search slowly
            if not execute_ball_catch:
                if not ball_visible:
                    # Use slightly larger rotation so it doesn't get stuck barely moving
                    robot_spin_search()
                    time.sleep(0.3)
                    robot_stop()
                    time.sleep(0.6) # Stop and let camera frame update
                else:
                    # Ball is visible but robot is not active yet. Just wait.
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

    # Start PiCamera Tag Thread
    t_tag = threading.Thread(target=picamera_tag_loop)
    t_tag.daemon = True
    t_tag.start()

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
