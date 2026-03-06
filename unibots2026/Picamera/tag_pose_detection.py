"""
AprilTag Pose Detection Example
This script demonstrates how to use the pupil_apriltags library to detect AprilTags and estimate 
their pose (position and orientation) relative to the camera. 
It assumes you have already calibrated your camera.
"""
import cv2
from pupil_apriltags import Detector
import time
try:
    from picamera2 import Picamera2
except ImportError:
    print("Picamera2 is not installed or not found. Please install proper packages (e.g. python3-picamera2).")
    exit(1)
# --- CONFIGURATION ---
# Replace with the 4 numbers from your calibration script!
CAMERA_PARAMS = (954.4949188171072, 955.5979729485147, 332.0798756650343, 245.67451277016548)
TAG_SIZE = 0.10 # 10 centimeters = 0.10 meters

# --- SETUP HARDWARE ---
# Initialize Picamera2
picam2 = Picamera2()
config = picam2.create_video_configuration(
    main={"size": (640, 480), "format": "RGB888"},
    controls={"FrameRate": 30}
)
picam2.configure(config)
picam2.start()

# Initialize Detector (quad_decimate=2.0 speeds up detection by shrinking the image internally)
at_detector = Detector(families='tagStandard41h12', quad_decimate=2.0)

print("Vision System Online. Press 'Q' to quit.")

while True:
    frame = picam2.capture_array()
        
    # 1. Grayscale Conversion (Crucial for Speed)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # 2. Detect Balls (Example Placeholder)
    # Put your HSV color thresholding here. Use the 'frame' (color) variable for this!
    # e.g., mask = cv2.inRange(hsv, lower_orange, upper_orange)
    
    # 3. Detect AprilTags (Using the Grayscale image)
    tags = at_detector.detect(gray, estimate_tag_pose=True, camera_params=CAMERA_PARAMS, tag_size=TAG_SIZE)
    
    for tag in tags:
        tag_id = tag.tag_id
        
        # Translation vector (X, Y, Z distance from camera in meters)
        x_dist = tag.pose_t[0][0] # Left/Right offset
        y_dist = tag.pose_t[1][0] # Up/Down offset
        z_dist = tag.pose_t[2][0] # Forward distance to tag
        
        # Print for debugging
        print(f"Tag ID: {tag_id} | Distance: {z_dist:.2f}m | X-Offset: {x_dist:.2f}m")
        
        # Draw bounding box for visual feedback
        cv2.polylines(frame, [tag.corners.astype(int)], True, (0, 255, 0), 2)
        cv2.putText(frame, str(tag_id), (int(tag.center[0]), int(tag.center[1])), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # Display window (Comment this out when running headless on the robot!)
    cv2.imshow("Robot Vision", frame)
    
    if cv2.waitKey(1) == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()
