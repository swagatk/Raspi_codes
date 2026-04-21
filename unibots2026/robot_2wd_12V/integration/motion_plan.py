import time
import threading
import sys
import cv2
from apriltag_detection import * 

# from servo_test import move_arm_up, move_arm_drop, arm_rest_pose
# Configuration Variables (User Defined Durations in seconds)
# Maximum total time reduced to ~180s (3 minutes)
TIME_STEP_1_HOME_TAG_CAPTURE = 5
TIME_STEP_2_ROTATE_180 = 5
TIME_STEP_3A_SCAN_BALLS = 15
TIME_STEP_3B_GO_TO_BALL = 45
TIME_STEP_4_CAPTURE_MANOEUVRE = 30
TIME_STEP_5_SECURE_BALL = 5
TIME_STEP_6_GO_TO_HOME = 50
TIME_STEP_7_DROP_MANOEUVRE = 20
# TOTAL: ~175 seconds

# Settings
FRAME_WIDTH = 320
FRAME_HEIGHT = 240
FPS = 30
CONSECUTIVE_FRAMES_REQUIRED = 3

# State Management
is_running = False
current_step = 0
latest_frame = None
frame_lock = threading.Lock()
sequence_start_time = 0

# Simulated/Imported specific commands 
# You'll need to import the exact functions from your other modules here
# from servo_test import move_arm_up, move_arm_drop, arm_rest_pose, send_cmd
# from go_to_home import navigate_to_home, capture_home_tags
# from ball_detection import detect_orange_balls
# from go_to_ball import navigate_to_ball
# from keyboard_control import drive_forward, stop_robot, turn_robot

def print_progress_bar(step, step_name, elapsed, duration, bar_length=20):
    """Draws a progress bar in the terminal."""
    fraction = elapsed / duration if duration > 0 else 1
    fraction = min(max(fraction, 0), 1) # Clamp between 0-1
    filled_length = int(bar_length * fraction)
    bar = '=' * filled_length + '>' + ' ' * (bar_length - filled_length - 1)
    
    total_elapsed = time.time() - sequence_start_time
    minutes = int(total_elapsed // 60)
    seconds = int(total_elapsed % 60)
    
    sys.stdout.write(f"\r[{bar}] Step {step}: {step_name} | Sub-timer: {int(elapsed)}s/{duration}s | Total: {minutes}m{seconds}s  ")
    sys.stdout.flush()

def button_monitor_thread():
    """Monitor a physical button to start/stop the autonomous sequence."""
    global is_running, current_step, sequence_start_time
    
    print("Press Enter to start/stop...")
    while True:
        try:
            user_input = input("") 
            if user_input.strip() == "":
                is_running = not is_running
                if is_running:
                    print("\nButton pressed: Sequence STARTED.")
                    sequence_start_time = time.time()
                    current_step = 1
                else:
                    print("\nButton pressed: Sequence STOPPED.")
                    current_step = 0
                    execute_stop_safely()
                    
        except KeyboardInterrupt:
            break
        time.sleep(0.1)

def camera_capture_thread():
    """Capture video in a separate thread for optimal performance."""
    global latest_frame, is_running
    
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)
    
    while True:
        ret, frame = cap.read()
        if ret:
            with frame_lock:
                latest_frame = frame
        else:
            time.sleep(0.01)
            
        if not is_running and current_step == 0:
            time.sleep(0.1)

    cap.release()   

def execute_stop_safely():
    """Immediately stop all motors and safe the arm."""
    print("\nEMERGENCY STOP executed.")
    # stop_robot()
    # send_cmd(b'a\n') # Arm down/Rest

def step_1_home_capture():
    start_time = time.time()
    consecutive_frames = 0
    duration = TIME_STEP_1_HOME_TAG_CAPTURE
    detected_home_tags = []
    
    while is_running:
        elapsed = time.time() - start_time
        print_progress_bar(1, "Home Tag Capture", elapsed, duration)
        
        if elapsed >= duration:
            break
            
        with frame_lock:
            frame = latest_frame
            
        if frame is not None:
            tag_detections = capture_home_tag_function(frame)
            if len(tag_detections) >= 2: 
                consecutive_frames +=1
                # Save the tag IDs locally to return later
                detected_home_tags = [tag.tag_id for tag in tag_detections[:2]]
            
            if consecutive_frames >= CONSECUTIVE_FRAMES_REQUIRED:
                print("\nHome tags successfully captured.")
                return detected_home_tags 
        time.sleep(0.1)

    return detected_home_tags

def step_2_rotate_180():
    start_time = time.time()
    duration = TIME_STEP_2_ROTATE_180
    
    while is_running:
        elapsed = time.time() - start_time
        print_progress_bar(2, "Rotating 180 degrees", elapsed, duration)
        if elapsed >= duration:
            break
        time.sleep(0.1)
    print()

def step_3_scan_and_go_to_ball():
    start_time = time.time()
    duration = TIME_STEP_3A_SCAN_BALLS
    
    while is_running:
        elapsed = time.time() - start_time
        print_progress_bar("3A", "Scanning for balls", elapsed, duration)
        if elapsed >= duration:
            break
        # Mock detection logic
        time.sleep(0.05)
    print()
    
    if not is_running: return
        
    start_time = time.time()
    duration = TIME_STEP_3B_GO_TO_BALL
    while is_running:
        elapsed = time.time() - start_time
        print_progress_bar("3B", "Navigating to ball", elapsed, duration)
        if elapsed >= duration:
            break
        time.sleep(0.1)
    print()

def step_4_capture_manoeuvre():
    start_time = time.time()
    duration = TIME_STEP_4_CAPTURE_MANOEUVRE
    
    while is_running:
        elapsed = time.time() - start_time
        print_progress_bar(4, "Ball capture manouver", elapsed, duration)
        if elapsed >= duration:
            break
        time.sleep(0.1)
    print()
        
def step_5_secure_ball():
    start_time = time.time()
    duration = TIME_STEP_5_SECURE_BALL
    
    while is_running:
        elapsed = time.time() - start_time
        print_progress_bar(5, "Securing ball / Arm UP", elapsed, duration)
        if elapsed >= duration:
            break
        time.sleep(0.1)
    print()

def step_6_go_home():
    start_time = time.time()
    duration = TIME_STEP_6_GO_TO_HOME
    
    while is_running:
        elapsed = time.time() - start_time
        print_progress_bar(6, "Navigating to Home", elapsed, duration)
        if elapsed >= duration:
            break
        time.sleep(0.1)
    print()

def step_7_drop_manoeuvre():
    start_time = time.time()
    duration = TIME_STEP_7_DROP_MANOEUVRE
    
    while is_running:
        elapsed = time.time() - start_time
        print_progress_bar(7, "Executing Drop", elapsed, duration)
        if elapsed >= duration:
            break
        time.sleep(0.1)
    print()

def step_8_rest_pose():
    print("Step 8: Entering Rest Pose.")
    execute_stop_safely()

def autonomous_sequence():
    """Main state machine running the steps sequentially."""
    global current_step, is_running
    
    # Store the home tags globally once captured
    home_tags = []
    
    while True:
        if is_running:
            if current_step == 1:
                home_tags = step_1_home_capture()
                if home_tags:
                    print(f"Captured Home Tags: {home_tags}")
                if is_running: current_step = 2
            
            elif current_step == 2:
                step_2_rotate_180()
                if is_running: current_step = 3
                
            elif current_step == 3:
                step_3_scan_and_go_to_ball()
                if is_running: current_step = 4
                
            elif current_step == 4:
                step_4_capture_manoeuvre()
                if is_running: current_step = 5
                
            elif current_step == 5:
                step_5_secure_ball()
                if is_running: current_step = 6
                
            elif current_step == 6:
                step_6_go_home()
                if is_running: current_step = 7
                
            elif current_step == 7:
                step_7_drop_manoeuvre()
                if is_running: current_step = 8
                
            elif current_step == 8:
                step_8_rest_pose()
                print("Sequence Completed Successfully.")
                is_running = False
                current_step = 0
                
        else:
            time.sleep(0.1)

if __name__ == "__main__":
    cam_thread = threading.Thread(target=camera_capture_thread, daemon=True)
    cam_thread.start()
    
    btn_thread = threading.Thread(target=button_monitor_thread, daemon=True)
    btn_thread.start()
    
    print("Starting Main Autonomous Loop Handler...")
    
    try:
        autonomous_sequence()
    except KeyboardInterrupt:
        print("\nInterrupted by user, shutting down.")
        execute_stop_safely()

