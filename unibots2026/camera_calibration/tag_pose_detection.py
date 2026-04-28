"""
AprilTag Pose Detection Example
This script demonstrates how to use the pupil_apriltags library to detect AprilTags and estimate 
their pose (position and orientation) relative to the camera. 
It assumes you have already calibrated your camera.
"""
import cv2
from pupil_apriltags import Detector
import time
import math
import glob

# --- CONFIGURATION ---
CAMERA_TYPE = "usb"  # Options: "picamera" or "usb"

def find_usb_camera():
    video_devices = sorted(glob.glob('/dev/video*'))
    for dev in video_devices:
        try:
            num = int(dev.replace('/dev/video', ''))
            if num >= 10:
                continue
        except ValueError:
            continue
            
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if cap.isOpened():
            ret, _ = cap.read()
            cap.release()
            if ret:
                return dev
    return "/dev/video2" # Fallback

USB_CAMERA_PATH = find_usb_camera()

try:
    if CAMERA_TYPE == "picamera":
        from picamera2 import Picamera2
except ImportError:
    if CAMERA_TYPE == "picamera":
        print("Picamera2 is not installed or not found. Please install proper packages (e.g. python3-picamera2).")
        exit(1)

# Replace with the 4 numbers from your calibration script!
if CAMERA_TYPE == "picamera":
    CAMERA_PARAMS = (907.462397724348, 908.550833315007, 358.40056240558073, 246.47297678800183)
elif CAMERA_TYPE == "usb":
    CAMERA_PARAMS = camera_params = (742.843247995633, 743.2228374107693, 322.3205884283167, 234.06623771807327)
TAG_SIZE = 0.10 # 10 centimeters = 0.10 meters
SCALE = 2.0

# Frame dimensions
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CENTER_X = FRAME_WIDTH // 2   # 320
CENTER_Y = FRAME_HEIGHT // 2  # 240

# --- SETUP HARDWARE ---
picam2 = None
cap = None

if CAMERA_TYPE == "picamera":
    picam2 = Picamera2()
    config = picam2.create_video_configuration(
        main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"},
        controls={"FrameRate": 30}
    )
    picam2.configure(config)
    picam2.start()
    print("PiCamera Started.")
elif CAMERA_TYPE == "usb":
    cap = cv2.VideoCapture(USB_CAMERA_PATH, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    if not cap.isOpened():
        raise Exception(f"Cannot open USB camera at {USB_CAMERA_PATH}")
    print(f"USB Camera Started ({USB_CAMERA_PATH}).")

# Initialize Detector (quad_decimate=2.0 speeds up detection by shrinking the image internally)
at_detector = Detector(families='tagStandard41h12', quad_decimate=2.0)

print("Vision System Online. Press 'Q' to quit.")

while True:
    if CAMERA_TYPE == "picamera":
        frame = picam2.capture_array()
    else:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame. Reconnecting...")
            time.sleep(0.5)
            continue
        
    # 1. Grayscale Conversion (Crucial for Speed)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # 2. Detect Balls (Example Placeholder)
    # Put your HSV color thresholding here. Use the 'frame' (color) variable for this!
    # e.g., mask = cv2.inRange(hsv, lower_orange, upper_orange)
    
    # 3. Detect AprilTags (Using the Grayscale image)
    tags = at_detector.detect(gray, estimate_tag_pose=True, camera_params=CAMERA_PARAMS, tag_size=TAG_SIZE)
    
    pixel_angles = []  # Store pixel angles for averaging
    
    for tag in tags:
        tag_id = tag.tag_id
        
        # Translation vector (X, Y, Z distance from camera in meters)
        x_dist = tag.pose_t[0][0] # Left/Right offset
        y_dist = tag.pose_t[1][0] # Up/Down offset
        z_dist = tag.pose_t[2][0] / SCALE # Forward distance to tag 
        
        # Calculate pixel offset from center of screen
        tag_center_x = int(tag.center[0])
        tag_center_y = int(tag.center[1])
        x_offset = tag_center_x - CENTER_X  # Positive = right of center
        y_offset = tag_center_y - CENTER_Y  # Positive = below center
        
        # Draw bounding box for visual feedback
        cv2.polylines(frame, [tag.corners.astype(int)], True, (0, 255, 0), 2)
        cv2.putText(frame, str(tag_id), (tag_center_x, tag_center_y - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        # Draw line from center of screen to tag center
        cv2.line(frame, (CENTER_X, CENTER_Y), (tag_center_x, tag_center_y), (255, 0, 255), 2)
        
        # Calculate pixel angle using pixel offsets for display
        pixel_angle_rad = math.atan2(x_offset, y_offset)
        pixel_angle_deg = math.degrees(pixel_angle_rad)
        
        # Calculate heading angle using actual distances in meters for console
        heading_rad = math.atan2(x_dist, z_dist)
        heading_deg = math.degrees(heading_rad)
        
        # Display pixel angle on the line (midpoint between center and tag)
        mid_x = (CENTER_X + tag_center_x) // 2
        mid_y = (CENTER_Y + tag_center_y) // 2
        angle_text = f"{pixel_angle_deg:.1f} deg"
        cv2.putText(frame, angle_text, (mid_x + 5, mid_y - 5), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
        
        # Display pixel offsets near the tag
        offset_text = f"X:{x_offset} Y:{y_offset}"
        cv2.putText(frame, offset_text, (tag_center_x - 40, tag_center_y + 20), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        
        # Store pixel angle for averaging
        pixel_angles.append(pixel_angle_rad)
        
        # Print heading angle (from actual distances) to console
        print(f"Tag ID: {tag_id} | Heading: {heading_deg:.1f} deg | Z: {z_dist:.2f}m | X: {x_dist:.2f}m")

    # Draw average angle line (orange) if tags detected
    if pixel_angles:
        avg_angle_rad = sum(pixel_angles) / len(pixel_angles)
        avg_angle_deg = math.degrees(avg_angle_rad)
        # Calculate endpoint for average line (length 200 pixels)
        line_length = 200
        end_x = int(CENTER_X + line_length * math.sin(avg_angle_rad))
        end_y = int(CENTER_Y + line_length * math.cos(avg_angle_rad))
        cv2.line(frame, (CENTER_X, CENTER_Y), (end_x, end_y), (0, 165, 255), 3)  # Orange in BGR
        # Display average angle text
        cv2.putText(frame, f"Avg: {avg_angle_deg:.1f} deg", (end_x + 5, end_y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)

    # Draw center crosshair lines
    cv2.line(frame, (CENTER_X, 0), (CENTER_X, FRAME_HEIGHT), (0, 255, 255), 1)  # Vertical line
    cv2.line(frame, (0, CENTER_Y), (FRAME_WIDTH, CENTER_Y), (0, 255, 255), 1)   # Horizontal line

    # Display window (Comment this out when running headless on the robot!)
    cv2.imshow("Robot Vision", frame)
    
    if cv2.waitKey(1) == ord('q'):
        break

if picam2: picam2.stop()
if cap: cap.release()
cv2.destroyAllWindows()
