import os

# Hardware & Connections
CAMERA_TYPE = "usb"  # 'picamera' or 'usb'
CAPTURE_WIDTH = 320
CAPTURE_HEIGHT = 240
SKIP_FRAMES = 3

# Models
NCNN_MODEL = "/home/pi/yolo_project/orange_ball_ncnn_model"
PT_MODEL = "/home/pi/yolo_project/orange_ball.pt"
MODEL_PATH = NCNN_MODEL if os.path.exists(NCNN_MODEL) else PT_MODEL

# Navigation & Obstacle Avoidance
STOP_DIST = 35        # cm
SIDE_DIST = 30        # cm
CENTER_TOLERANCE = 40 # pixels 

# Ball parameters
BALL_DIAMETER_CM = 4.0
TARGET_REACHED_CM = 40.0
CAPTURE_DISTANCE_CM = 15.0
CAPTURE_SPEED_CM_S = 10.6
CAPTURE_GRIPPER_CYCLES = 4

# Mission Durations (seconds)
STEP1_DURATION = 10   # Home tag photo
STEP2_DURATION = 5    # Rotate 180
STEP3A_DURATION = 20  # Scan for balls
STEP3B_DURATION = 60  # Move to balls
STEP4_DURATION = 40   # Capture manoeuvre
STEP6_DURATION = 60   # Go to home
STEP7_DURATION = 20   # Ball drop

# Specify Home Tags
HOME_TAG_IDS = [14, 15]
