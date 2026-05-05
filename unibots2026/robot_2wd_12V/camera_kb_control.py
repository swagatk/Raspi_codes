import cv2
import time
import serial
import serial.tools.list_ports
import sys
import termios
import tty
import threading
import select
import os
import numpy as np
import glob
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
CAPTURE_WIDTH = 320 
CAPTURE_HEIGHT = 240
SKIP_FRAMES = 3
CAPTURE_GRIPPER_CYCLES = 3

YOLO_MODEL_PATH = "/home/pi/yolo_project/orange_ball.pt"
NCNN_MODEL = "/home/pi/yolo_project/orange_ball_ncnn_model"
APRILTAG_FAMILY = "tagStandard41h12"
SCALE = 3.0 # Scale factor for AprilTag distance estimation

# Calibration parameters for 640x480
orig_params = (954.4949188171072, 955.5979729485147, 332.0798756650343, 245.67451277016548)
scale_x = CAPTURE_WIDTH / 640.0
scale_y = CAPTURE_HEIGHT / 480.0
CAMERA_PARAMS = (orig_params[0] * scale_x, orig_params[1] * scale_y, orig_params[2] * scale_x, orig_params[3] * scale_y)
TAG_SIZE = 0.10

if os.path.exists(NCNN_MODEL):
    YOLO_MODEL_PATH = NCNN_MODEL

# --- CAMERA AUTO DETECT ---
def find_usb_camera():
    video_devices = sorted(glob.glob('/dev/video*'))
    for dev in video_devices:
        try:
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

# --- CONNECT TO ARDUINO ---
def find_arduino_port():
    print("Scanning for available serial ports...")
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if 'ttyACM' in port.device or 'ttyUSB' in port.device:
            return port.device
    return None

serial_port = find_arduino_port()
try:
    if not serial_port:
        print("Error: Could not find any connected Arduino.")
        sys.exit(1)
        
    ser = serial.Serial(serial_port, 115200, timeout=1)
    ser.reset_input_buffer()
    print(f"Connected to Robot on {serial_port}!")
    time.sleep(2)
except Exception as e:
    print(f"Error: {e}")
    sys.exit()

# --- GLOBALS ---
running = True 
current_mode = None  # None, 'T', 'P', 'D'
latest_L, latest_C, latest_R = 999, 999, 999

# --- BACKGROUND THREAD: READ SENSORS ---
def listen_to_arduino():
    global latest_L, latest_C, latest_R
    while running:
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8').rstrip()
                if line.startswith("D,"):
                    parts = line.split(",")
                    latest_L = int(parts[1])
                    latest_C = int(parts[2])
                    latest_R = int(parts[3])
                    sys.stdout.write(f"\rSensors -> L:{latest_L} C:{latest_C} R:{latest_R}   ")
                    sys.stdout.flush()
            except:
                pass
        time.sleep(0.05)

# --- KEYBOARD FUNCTION (Non-Blocking) ---
def get_key(timeout=0.1):
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        r, w, e = select.select([sys.stdin], [], [], timeout)
        if r:
            ch = sys.stdin.read(1)
        else:
            ch = None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# --- CAMERA PROCESSES ---
def run_vision_loop():
    global running, current_mode
    
    print(f"Loading YOLO model: {YOLO_MODEL_PATH}")
    model = YOLO(YOLO_MODEL_PATH)
    
    apriltag_detector = None
    if APRILTAG_AVAILABLE:
        apriltag_detector = PoseDetector(families=APRILTAG_FAMILY, quad_decimate=1.0)

    # Init PiCamera
    try:
        picam2 = Picamera2()
        config = picam2.create_video_configuration(
            main={"size": (CAPTURE_WIDTH, CAPTURE_HEIGHT), "format": "RGB888"},
            transform=Transform(hflip=1, vflip=1),
            controls={"FrameRate": 30},
            buffer_count=2
        )
        picam2.configure(config)
        picam2.start()
    except Exception as e:
        print(f"PiCamera Error: {e}")
        picam2 = None

    usb_cam_path = find_usb_camera()
    cap = cv2.VideoCapture(usb_cam_path, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAPTURE_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_HEIGHT)

    frame_counter = 0
    
    while running:
        try:
            if current_mode in ['T', 'P'] and picam2:
                frame = picam2.capture_array()
            elif current_mode == 'D' and cap.isOpened():
                ret, frame = cap.read()
                if not ret: continue
            else:
                time.sleep(0.1)
                try: cv2.destroyAllWindows()
                except: pass
                continue

            frame_counter += 1
            if frame_counter % SKIP_FRAMES != 0:
                continue

            annotated = frame.copy()
            
            if current_mode == 'D':
                # Ball detection with YOLO (USB Camera)
                results = model.predict(frame, imgsz=320, verbose=False, conf=0.4)
                if results and len(results) > 0:
                    annotated = results[0].plot()

            elif current_mode in ['T', 'P'] and apriltag_detector:
                # Tag / Pose detection with PiCamera
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                tags = apriltag_detector.detect(gray, estimate_tag_pose=True,
                                                camera_params=CAMERA_PARAMS, tag_size=TAG_SIZE)
                for tag in tags:
                    corners = tag.corners.astype(int)
                    for i in range(4):
                        cv2.line(annotated, tuple(corners[i]), tuple(corners[(i+1)%4]), (0, 255, 0), 2)
                    center = tuple(tag.center.astype(int))
                    cv2.circle(annotated, center, 4, (0, 0, 255), -1)
                    
                    if current_mode == 'P':
                        dist_cm = (tag.pose_t[2][0] / SCALE) * 100.0
                        R = tag.pose_R
                        yaw = np.degrees(np.arctan2(-R[2][0], np.sqrt(R[2][1]**2 + R[2][2]**2)))
                        cv2.putText(annotated, f"ID:{tag.tag_id} D:{dist_cm:.1f}cm Y:{yaw:.1f}", 
                                    (corners[0][0], corners[0][1] - 10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                    else:
                        cv2.putText(annotated, f"ID:{tag.tag_id}", 
                                    (corners[0][0], corners[0][1] - 10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

            cv2.imshow("Vision Stream", annotated)
            cv2.waitKey(1)
            
        except Exception as e:
            # print(f"Vision loop error: {e}")
            time.sleep(0.1)

    if picam2: picam2.stop()
    if cap: cap.release()
    cv2.destroyAllWindows()


# --- MAIN PROGRAM ---
print("\n--- CONTROLS ---")
print("WASD: Move | SPACE: Stop | 1-3: Speed")
print("U: Arm UP | J: Arm DOWN | H: Arm VERTICAL")
print("O: Gripper OPEN | C: Gripper CLOSE")
print("F: Elbow DROP | G: ARM DROP (+ OPEN)")
print("B: Ball Catch Manoeuvre")
print("T: Tag Detection (PiCamera) | P: Pose Detection (PiCamera) | K: Ball Detection (USB) | X: Stop Camera")
print("Q: Quit")
print("----------------")

sensor_thread = threading.Thread(target=listen_to_arduino)
sensor_thread.daemon = True
sensor_thread.start()

vision_thread = threading.Thread(target=run_vision_loop)
vision_thread.daemon = True
vision_thread.start()

current_cmd = b'S\n'

try:
    while True:
        key = get_key(0.55)
        cmd = None
        
        if key:
            key_lower = key.lower()
            if key_lower == 'w': cmd = b'F\n'
            elif key_lower == 's': cmd = b'B\n'
            elif key_lower == 'a': cmd = b'L\n'
            elif key_lower == 'd': cmd = b'R\n'
            elif key == ' ': cmd = b'S\n'
            elif key == '1': cmd = b'1\n'
            elif key == '2': cmd = b'2\n'
            elif key == '3': cmd = b'3\n'
            elif key_lower == 'u': cmd = b'A\n'
            elif key_lower == 'j': cmd = b'a\n'
            elif key_lower == 'h': cmd = b'H\n'
            elif key_lower == 'f': cmd = b'f\n'
            elif key_lower == 'g': cmd = b'g\n'
            elif key_lower == 'o': cmd = b'O\n'
            elif key_lower == 'c': cmd = b'C\n'
            elif key_lower == 't': 
                current_mode = 'T'
                print(f"\n[VISION] Switched to Tag Detection (PiCamera - {CAPTURE_WIDTH}x{CAPTURE_HEIGHT})")
                cmd = current_cmd
            elif key_lower == 'p': 
                current_mode = 'P'
                print(f"\n[VISION] Switched to Pose Detection (PiCamera - {CAPTURE_WIDTH}x{CAPTURE_HEIGHT})")
                cmd = current_cmd
            elif key_lower == 'k': 
                current_mode = 'D'
                print(f"\n[VISION] Switched to Ball Detection (USB Camera - {CAPTURE_WIDTH}x{CAPTURE_HEIGHT})")
                cmd = current_cmd
            elif key_lower == 'x':
                current_mode = None
                print("\n[VISION] Camera Stopped")
                cmd = current_cmd
            elif key_lower == 'b': 
                print("\r\n[BALL CATCH] Executing Manoeuvre...")
                ser.write(b'S\n')
                time.sleep(0.1)
                ser.write(b'a\n')
                time.sleep(2.0)
                
                ser.write(b'O\n')
                time.sleep(1.0)

                print("\r\n[BALL CATCH] Moving forward while opening/closing gripper...")
                ser.write(b'2\n')
                time.sleep(0.1)
                ser.write(b'F\n')
                
                time_to_move = 1.5
                if CAPTURE_GRIPPER_CYCLES > 0:
                    toggle_interval = time_to_move / (CAPTURE_GRIPPER_CYCLES * 2)
                else:
                    toggle_interval = time_to_move + 1
                    
                start_time = time.time()
                last_toggle = start_time
                gripper_state = b'O\n'
                
                while (time.time() - start_time) < time_to_move:
                    current_time = time.time()
                    if current_time - last_toggle >= toggle_interval:
                        if gripper_state == b'O\n':
                            ser.write(b'C\n')
                            gripper_state = b'C\n'
                        else:
                            ser.write(b'O\n')
                            gripper_state = b'O\n'
                        last_toggle = current_time
                    time.sleep(0.02)
                
                ser.write(b'C\n')
                time.sleep(0.5)
                ser.write(b'S\n')
                
                time.sleep(0.5)
                ser.write(b'A\n')
                time.sleep(2.0)
                
                cmd = b'S\n'
                current_cmd = b'S\n'
                
            elif key_lower == 'q': 
                ser.write(b'S\n')
                print("\r\nExecuting ARM DOWN (a) and Gripper OPEN (O)...")
                ser.write(b'a\n')
                time.sleep(2.0)
                ser.write(b'O\n')
                time.sleep(1.0)
                running = False
                break
        
        if key and cmd is not None:
            if cmd != current_cmd:
                sys.stdout.write(f"\r\n[KEY PRESSED] Sending: {cmd.decode('utf-8').strip()}\r\n")
                sys.stdout.flush()
                ser.write(cmd)
                current_cmd = cmd
        elif not key:
            if current_cmd in [b'F\n', b'B\n', b'L\n', b'R\n']:
                sys.stdout.write("\n[TIMEOUT OR NO KEY] Sending: S\n")
                sys.stdout.flush()
                ser.write(b'S\n')
                current_cmd = b'S\n'

except Exception as e:
    print(f"\nStopped! {e}")
    ser.write(b'S\n')
    ser.write(b'a\n')
    time.sleep(2.0)
    ser.write(b'O\n')
    time.sleep(1.0)
    running = False

finally:
    ser.write(b'S\n')
    time.sleep(0.1)
    if 'ser' in locals() and ser and hasattr(ser, 'close'):
        ser.close()
    print("\nSerial port closed. Exiting.")
