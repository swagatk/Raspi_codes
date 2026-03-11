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
TARGET_Z_DIST = 0.25       # Target distance to stop at (in meters)
X_OFFSET_TOL = 0.08        # Acceptable horizontal offset from center (meters)
Z_DIST_TOL = 0.05          # Acceptable distance tolerance (meters)
FRAME_SKIP = 3  # Only run detection every Nth frame

STOP_DIST = 35             # Front ultrasonic sensor stop distance (cm)
SIDE_DIST = 30             # Side ultrasonic sensor push away distance (cm)
ARRIVED_DIST = 20          # Distance at which ultrasonic confirms tag is close enough to stop (cm)
SENSOR_SMOOTHING_COUNT = 3 # Number of sensor readings to average to prevent false positives

SERIAL_PORT = '/dev/ttyACM0'  
WIDTH = 640 
HEIGHT = 480
CAMERA_PARAMS = (954.4949188171072, 955.5979729485147, 332.0798756650343, 245.67451277016548)
TAG_SIZE = 0.10  # 10 cm tag
SCALE = 3.0      # Scale factor from previous calibration

# --- GLOBAL VARIABLES FOR SENSORS ---
latest_L = 999
latest_C = 999
latest_R = 999

history_L = []
history_C = []
history_R = []

running = True

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
        ser.reset_output_buffer() # Clear older unsent commands
        ser.reset_input_buffer()  # Clear any stale incoming data
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

# --- BACKGROUND THREAD (Sensor Listener) ---
def update_sensors():
    global latest_L, latest_C, latest_R, history_L, history_C, history_R
    while running:
        if ser and ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8').rstrip()
                if line.startswith("D,"):
                    parts = line.split(",")
                    
                    # Read new raw values
                    new_L = int(parts[1])
                    new_C = int(parts[2])
                    new_R = int(parts[3])
                    
                    # Update histories
                    history_L.append(new_L)
                    history_C.append(new_C)
                    history_R.append(new_R)
                    
                    # Keep only the last N readings
                    if len(history_L) > SENSOR_SMOOTHING_COUNT:
                        history_L.pop(0)
                        history_C.pop(0)
                        history_R.pop(0)
                        
                    # Calculate averages
                    if len(history_L) > 0:
                        latest_L = sum(history_L) / len(history_L)
                        latest_C = sum(history_C) / len(history_C)
                        latest_R = sum(history_R) / len(history_R)
            except:
                pass
        time.sleep(0.01)

t = threading.Thread(target=update_sensors)
t.daemon = True
t.start()

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

        # --- SENSOR OVERRIDE / OBSTACLE AVOIDANCE ---
        OBSTACLE_DETECTED = False
        if latest_C < STOP_DIST:
            print(f"Blocked directly ahead ({latest_C}cm)! Avoiding...")
            send_cmd('S', force=True) # Stop
            time.sleep(0.2)
            
            send_cmd('1', force=True) 
            send_cmd('B', force=True) # Reverse briefly to un-stuck
            time.sleep(0.3)
            
            # Check which way is clearer
            if latest_L > latest_R:
                print("Turning LEFT (Left side has more space)")
                send_cmd('L', force=True) 
                last_known_x = 1.0  # Robot turned left, so target is now to our right
            else:
                print("Turning RIGHT (Right side has more space)")
                send_cmd('R', force=True)
                last_known_x = -1.0 # Robot turned right, so target is now to our left
                
            time.sleep(0.5) # Spin for 0.5 seconds
            send_cmd('S', force=True) # Stop spinning
            time.sleep(0.2) # Stabilize
            
            # Record it as lost so it falls into "search mode" naturally 
            last_seen_time = time.time() - SEARCH_TIMEOUT - 0.1 
            OBSTACLE_DETECTED = True

        elif latest_L < SIDE_DIST:
            print("Too close to LEFT -> Nudging Right")
            send_cmd('1', force=True) 
            send_cmd('R', force=True) # Spin Right
            last_known_x = -1.0 # Turned right, target is to the left
            time.sleep(0.1) # Short nudge
            OBSTACLE_DETECTED = True
            
        elif latest_R < SIDE_DIST:
            print("Too close to RIGHT -> Nudging Left")
            send_cmd('1', force=True) 
            send_cmd('L', force=True) # Spin Left
            last_known_x = 1.0 # Turned left, target is to the right
            time.sleep(0.1) # Short nudge
            OBSTACLE_DETECTED = True

        if OBSTACLE_DETECTED:
             # Skip tag processing for this frame to allow obstacle avoidance to finish
             continue

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
                # If we're getting close to target, use ultrasonic distance!
                # Note: target_z is in meters, ARRIVED_DIST is in cm
                if target_z < 0.6 and latest_C < ARRIVED_DIST:
                    send_cmd('S', force=True)
                    print(f"-> MISSION ACCOMPLISHED! ARRIVED at Target (Ultrasonic Front: {latest_C}cm, X: {target_x:.2f}m)")
                    break # Exit the main loop to permanently stop the robot

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
