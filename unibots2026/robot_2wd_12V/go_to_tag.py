import cv2
import time
import serial
import sys
import numpy as np
import threading
from picamera2 import Picamera2

try:
    from pupil_apriltags import Detector as PoseDetector
except ImportError:
    print("Error: pupil_apriltags not found! Please install it.")
    sys.exit()

# --- CONFIGURATION ---
TARGET_TAG_ID = 0          # The ID of the AprilTag to search for
TARGET_Z_DIST = 0.20       # Target distance to stop at (in meters)
X_OFFSET_TOL = 0.08        # Acceptable horizontal offset from center (meters)
Z_DIST_TOL = 0.05          # Acceptable distance tolerance (meters)
FRAME_SKIP = 3  # Only run detection every Nth frame

SERIAL_PORT = '/dev/ttyACM0'  
WIDTH = 640 
HEIGHT = 480
CAMERA_PARAMS = (954.4949188171072, 955.5979729485147, 332.0798756650343, 245.67451277016548)
TAG_SIZE = 0.10  # 10 cm tag
SCALE = 3.0      # Scale factor from previous calibration

# --- SERIAL SETUP ---
try:
    ser = serial.Serial(SERIAL_PORT, 115200, timeout=0)
    print(f"Connected to Arduino on {SERIAL_PORT}!")
    time.sleep(2)
except Exception as e:
    print(f"Error connecting to Serial: {e}")
    ser = None

current_cmd = b'S'
last_cmd_time = 0

def send_cmd(cmd, force=False):
    global current_cmd, last_cmd_time
    # Convert string to bytes
    if isinstance(cmd, str):
        cmd = cmd.encode()
        
    now = time.time()
    # Send if command changed, or if 0.3s has passed (to prevent Arduino auto-stop timeout)
    # or if force flag is manually passed to override all optimizations!
    if (force or cmd != current_cmd or (now - last_cmd_time) > 0.3) and ser:
        ser.write(cmd)
        ser.flush()  # Crucial! Force OS to empty the USB buffer immediately so sleeps are accurate
        current_cmd = cmd
        last_cmd_time = now

# --- CAMERA SETUP & THREADING ---
print("Starting Camera...")
picam2 = Picamera2()
config = picam2.create_video_configuration(
    main={"size": (WIDTH, HEIGHT), "format": "RGB888"},
    controls={"FrameRate": 30},
    buffer_count=2
)
picam2.configure(config)
picam2.start()

apriltag_detector = PoseDetector(families='tagStandard41h12', quad_decimate=2.0)

running = True
raw_frame = None

def camera_capture_thread():
    """ Dedicated thread purely to empty the camera buffer as fast as possible. """
    global running, raw_frame
    while running:
        try:
            frame = picam2.capture_array()
            raw_frame = frame
        except Exception as e:
            time.sleep(0.01)

capture_thread = threading.Thread(target=camera_capture_thread)
capture_thread.daemon = True
capture_thread.start()

# --- MAIN ROBOT LOGIC ---
print("\n--- ROBOT GO_TO_TAG ACTIVE ---")
print(f"Searching for Tag ID: {TARGET_TAG_ID}")
print("Press Ctrl+C to stop.")

last_seen_time = 0
last_known_x = 0.0    # Remember which side the tag was on to search smarter
SEARCH_TIMEOUT = 0.7  # If tag unseen for 0.7 seconds, start scanning for it

# Make sure we start at a slower speed for better control
send_cmd('1')

frame_skip_counter = 0


try:
    while True:
        frame = raw_frame
        if frame is None:
            time.sleep(0.01)
            continue
            
        frame_skip_counter += 1
        if frame_skip_counter < FRAME_SKIP:
            time.sleep(0.01)
            continue
            
        frame_skip_counter = 0
            
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        tags = apriltag_detector.detect(frame_gray, estimate_tag_pose=True, 
                                        camera_params=CAMERA_PARAMS, tag_size=TAG_SIZE)
        
        target_found = False
        target_z = 0.0
        target_x = 0.0

        for tag in tags:
            if tag.tag_id == TARGET_TAG_ID:
                target_found = True
                last_seen_time = time.time()
                
                # Coordinate frame mapping based on your previous config
                target_x = tag.pose_t[0][0]  # Left/Right offset
                target_z = tag.pose_t[2][0] / SCALE # Forward distance
                break

        if target_found:
            last_known_x = target_x
            
            # --- ALIGN & APPROACH LOGIC ---
            
            # Dynamic X tolerance (Funnel approach):
            # Allows more horizontal error when far away, tightens as it gets closer.
            # Prevents over-correcting tiny 2-degree offsets when 1.5+ meters away!
            current_x_tol = max(X_OFFSET_TOL, target_z * 0.15) # Relaxed to ~8.5 degrees

            # Fast Forward override! If the tag is far away and somewhat centralized, just charge!
            # It will naturally auto-correct its drifting rotation *while* driving forward later.
            if target_z > 1.0 and abs(target_x) < (target_z * 0.35): # Up to 20 degree error allowed for charging
                print(f"Tag {TARGET_TAG_ID} found FAR! Skipping alignment, charging FORWARD (Z: {target_z:.2f}m, X: {target_x:.2f}m)")
                send_cmd('3') 
                send_cmd('F')
                continue # Skip the rest of the alignment checks for this frame

            print(f"Tag {TARGET_TAG_ID} spotted! Z: {target_z:.2f}m, X: {target_x:.2f}m")

            # 1. First prioritize rotational alignment (X axis)
            if target_x > current_x_tol:
                # Ensure we are fully stopped before attempting micro-rotations!
                if current_cmd != b'S':
                    send_cmd('S', force=True)
                    time.sleep(0.1) # Let momentum settle
                    
                # Tag is to the right, turn right (micro twitch at speed 1)
                send_cmd('1', force=True)
                send_cmd('R', force=True)
                time.sleep(0.04)  # Back to 40ms to overcome static friction 
                send_cmd('S', force=True)
                
                time.sleep(0.1)   # 100ms OFF (let robot settle)
                print(f"-> Turning RIGHT slowly to align (X: {target_x:.2f} > {current_x_tol:.2f})")
                
                # Force immediate check of next frame to prevent rapid-fire twitching blindly
                frame_skip_counter = FRAME_SKIP
                
            elif target_x < -current_x_tol:
                if current_cmd != b'S':
                    send_cmd('S', force=True)
                    time.sleep(0.1) # Let momentum settle
                    
                # Tag is to the left, turn left (micro twitch at speed 1)
                send_cmd('1', force=True)
                send_cmd('L', force=True)
                time.sleep(0.04)  # Back to 40ms
                send_cmd('S', force=True)
                
                time.sleep(0.1)   # 100ms OFF 
                print(f"-> Turning LEFT slowly to align (X: {target_x:.2f} < {-current_x_tol:.2f})")
                
                # Force immediate check of next frame
                frame_skip_counter = FRAME_SKIP
                
            # 2. X is aligned, check distance (Z axis)
            else:
                if target_z > TARGET_Z_DIST + Z_DIST_TOL:
                    # Too far away, move forward
                    send_cmd('3') # High speed for forward motion!
                    send_cmd('F')
                    print("-> Moving FORWARD")
                elif target_z < TARGET_Z_DIST - Z_DIST_TOL:
                    # Too close, back up
                    send_cmd('1') # Low speed for backing up (safer)
                    send_cmd('B')
                    print("-> Moving BACKWARD")
                else:
                    # Within tolerance for both X and Z
                    send_cmd('S')
                    print(f"-> MISSION ACCOMPLISHED! ARRIVED at Target (Z: {target_z:.2f}m, X: {target_x:.2f}m)")
                    break # Exit the main loop to permanently stop the robot
                    
        else:
            # Tag not currently visible
            time_since_last_seen = time.time() - last_seen_time
            if time_since_last_seen > SEARCH_TIMEOUT:
                # Spin back towards the direction we last saw it in discrete steps
                search_dir = 'R' if last_known_x > 0 else 'L'
                dir_name = 'RIGHT' if search_dir == 'R' else 'LEFT'
                print(f"Tag {TARGET_TAG_ID} lost... Searching {dir_name} in steps...")
                
                # Use send_cmd with force=True so it records the state properly
                send_cmd('1', force=True) 
                send_cmd(search_dir, force=True)
                
                time.sleep(0.08) # 80ms ON: Shorter kick to prevent massive jumps
                
                send_cmd('S', force=True)
                
                time.sleep(0.2)  # 200ms OFF: completely stop and let the camera sensor clear motion blur
                
                # Force the next loop iteration to process a frame immediately!
                frame_skip_counter = FRAME_SKIP 
            else:
                # Briefly lost it, just wait
                send_cmd('S', force=True)
                
        # Small delay to keep loop from maxing out CPU needlessly
        # Capture thread still empties buffer independently
        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nEmergency Stop Triggered.")
finally:
    running = False
    send_cmd('S')
    time.sleep(0.1) # Wait for stop command to go through
    picam2.stop()
    print("Clean exit.")
