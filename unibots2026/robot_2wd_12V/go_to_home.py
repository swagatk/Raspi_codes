import cv2
import numpy as np
from pupil_apriltags import Detector
import serial
import time
import math
import sys
import select
import threading
from picamera2 import Picamera2

# --- CONFIGURATION ---
CAMERA_PARAMS = (954.4949188171072, 955.5979729485147, 332.0798756650343, 245.67451277016548)
TAG_SIZE = 0.10 # 10 centimeters = 0.10 meters
SCALE = 3.0     # Scale factor for Picamera distance (Z) calibration

STOP_DIST = 20             # Front ultrasonic sensor stop distance (cm)
SIDE_DIST = 20             # Side ultrasonic sensor push away distance (cm)
SENSOR_SMOOTHING_COUNT = 3 # Number of sensor readings to average to prevent false positives
SEARCH_TIMEOUT = 0.7       # If tag unseen for 0.7 seconds, start scanning for it
FRAME_SKIP = 3             # Only run detection every Nth frame

# --- ABSOLUTE MAP (The "GPS" Coordinates) ---
# 'wall_base_angle' is the compass heading the robot would have if it 
# was staring perfectly straight at that specific wall.
# (0=East, 90=North, 180=West, 270=South)
gap1 = 0.15
gap2 = 0.3
gap3 = 0.5  
xmax = 2.0 
ymax = 2.0
ARENA_MAP = {
    
    # Right Wall (Y = ymax) - Staring at it means facing West (180 deg)
    12: {"x": xmax-gap1, "y": ymax, "wall_base_angle": 180},
    13: {"x": xmax-gap2, "y": ymax, "wall_base_angle": 180},
    14: {"x": xmax-gap2, "y": ymax, "wall_base_angle": 180},
    15: {"x": xmax-gap3, "y": ymax, "wall_base_angle": 180},
    16: {"x": xmax-gap2, "y": ymax, "wall_base_angle": 180},
    17: {"x": xmax-gap2, "y": ymax, "wall_base_angle": 180},
    # Top Wall (X = xmax) - Staring at it means facing North 90 deg)
    6: {"x": xmax, "y": gap1, "wall_base_angle": 90},
    7: {"x": xmax, "y": gap2, "wall_base_angle": 90},
    8: {"x": xmax, "y": gap2, "wall_base_angle": 90},
    9: {"x": xmax, "y": gap3, "wall_base_angle": 90},
    10: {"x": xmax, "y": gap2, "wall_base_angle": 90},
    11: {"x": xmax, "y": gap2, "wall_base_angle": 90},
    # Left Wall (Y = 0.0) - Staring at it means facing East (0 deg)
    0: {"x": gap1, "y": 0.0, "wall_base_angle": 0},
    1: {"x": gap2, "y": 0.0, "wall_base_angle": 0},
    2: {"x": gap2, "y": 0.0, "wall_base_angle": 0},
    3: {"x": gap3, "y": 0.0, "wall_base_angle": 0},
    4: {"x": gap2, "y": 0.0, "wall_base_angle": 0},
    5: {"x": gap2, "y": 0.0, "wall_base_angle": 0},
    # Bottom Wall (X = 0) - Staring at it means facing South (270 deg)
    18: {"x": 0, "y": ymax-gap1, "wall_base_angle": 270},
    19: {"x": 0, "y": ymax-gap2, "wall_base_angle": 270},
    20: {"x": 0, "y": ymax-gap2, "wall_base_angle": 270},
    21: {"x": 0, "y": ymax-gap3, "wall_base_angle": 270},
    22: {"x": 0, "y": ymax-gap2, "wall_base_angle": 270},
    23: {"x": 0, "y": ymax-gap2, "wall_base_angle": 270}
}

# --- HOME SETTINGS ---
# Define Home between two Tag IDs
HOME_TAG_1 = 15
HOME_TAG_2 = 16

tag1_info = ARENA_MAP[HOME_TAG_1]
tag2_info = ARENA_MAP[HOME_TAG_2]

# Calculate midpoint between the two tags
mid_x = (tag1_info["x"] + tag2_info["x"]) / 2.0
mid_y = (tag1_info["y"] + tag2_info["y"]) / 2.0

# Calculate 0.1m offset directly away from the wall
# wall_base_angle points into the wall, so we subtract the components to move away from it
wall_angle_rad = math.radians(tag1_info["wall_base_angle"])
HOME_X = mid_x - 0.1 * math.cos(wall_angle_rad)
HOME_Y = mid_y - 0.1 * math.sin(wall_angle_rad)

# --- GLOBAL VARIABLES FOR SENSORS ---
latest_L = 999
latest_C = 999
latest_R = 999
history_L = []
history_C = []
history_R = []
running = True

# --- SERIAL SETUP ---
SERIAL_PORT = '/dev/ttyACM0'  
try:
    ser = serial.Serial(SERIAL_PORT, 115200, timeout=0)
    print(f"Connected to Arduino on {SERIAL_PORT}!")
    time.sleep(2)
except Exception as e:
    print(f"Serial Error: {e}")
    ser = None

current_cmd = b'S'
last_cmd_time = 0

def send_cmd(cmd, force=False):
    global current_cmd, last_cmd_time
    if isinstance(cmd, str):
        cmd = cmd.encode()
        
    now = time.time()
    if (force or cmd != current_cmd or (now - last_cmd_time) > 0.3) and ser:
        ser.reset_output_buffer()
        ser.reset_input_buffer()
        ser.write(cmd)
        ser.flush()
        current_cmd = cmd
        last_cmd_time = now

# --- CAMERA SETUP & THREADING ---
print("Starting Camera...")
picam2 = Picamera2()
config = picam2.create_video_configuration(
    main={"size": (640, 480), "format": "RGB888"},
    controls={"FrameRate": 30},
    buffer_count=2
)
picam2.configure(config)
picam2.start()

detector = Detector(families='tagStandard41h12', quad_decimate=2.0)

raw_frame = None

def camera_capture_thread():
    global running, raw_frame
    while running:
        try:
            raw_frame = picam2.capture_array()
        except:
            time.sleep(0.01)

def update_sensors():
    global latest_L, latest_C, latest_R, history_L, history_C, history_R, running
    while running:
        if ser and ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8').rstrip()
                if line.startswith("D,"):
                    parts = line.split(",")
                    new_L = int(parts[1])
                    new_C = int(parts[2])
                    new_R = int(parts[3])
                    
                    history_L.append(new_L)
                    history_C.append(new_C)
                    history_R.append(new_R)
                    
                    if len(history_L) > SENSOR_SMOOTHING_COUNT:
                        history_L.pop(0)
                        history_C.pop(0)
                        history_R.pop(0)
                        
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

# --- OPTIMIZED VIDEO DISPLAY THREAD ---
show_vision = False
display_frame = None
start_motion = False

def video_display_thread():
    global display_frame, show_vision, running, start_motion
    cv2.namedWindow("Robot Vision", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Robot Vision", 320, 240)
    
    paused_frame = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.putText(paused_frame, "Vision Paused", (60, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    cv2.putText(paused_frame, "Press 'C' to toggle", (60, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    while running:
        if show_vision and display_frame is not None:
            try:
                small_frame = cv2.resize(display_frame, (320, 240))
                bgr = cv2.cvtColor(small_frame, cv2.COLOR_RGB2BGR)
                cv2.imshow("Robot Vision", bgr)
            except Exception as e:
                print(f"Vision error: {e}")
        else:
            cv2.imshow("Robot Vision", paused_frame)
            
        key = cv2.waitKey(30) & 0xFF
        if key == ord('q') or key == ord('Q'):
            running = False
        elif key == ord('s') or key == ord('S') or key == 13: # 13 is Enter
            start_motion = True
        elif key == ord('c') or key == ord('C'):
            show_vision = not show_vision
            print(f"\n[Camera View Toggled: {'ON' if show_vision else 'OFF'}]\n")

display_t = threading.Thread(target=video_display_thread)
display_t.daemon = True
display_t.start()

# --- MAIN NAVIGATION LOOP ---
state = "HUNTING" 
robot_x, robot_y, robot_heading = 0.0, 0.0, 0.0

print("--- MULTI-TAG SLAM SYSTEM ONLINE ---")

last_seen_time = time.time()
last_known_side = 1.0 # 1.0 for right, -1.0 for left
frame_skip_counter = 0

send_cmd('1') # Set a basic low speed just in case

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
        
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        tags = detector.detect(gray, estimate_tag_pose=True, camera_params=CAMERA_PARAMS, tag_size=TAG_SIZE)
        
        rx_list, ry_list, h_list = [], [], []
        is_localized = False
        detected_tag_ids = []
        
        # 1. MULTI-TAG TRIANGULATION
        if tags:
            print(f"\n--- Frame Data ---")
            
        draw_frame = frame.copy() if show_vision else None

        for tag in tags:
            detected_tag_ids.append(tag.tag_id)
            
            # Print raw tag pose directly from detector
            x_dist = tag.pose_t[0][0]
            y_dist = tag.pose_t[1][0]
            z_dist = tag.pose_t[2][0] / SCALE
            print(f"  > Tag {tag.tag_id}: X-Offset: {x_dist:.2f}m, Y-Offset: {y_dist:.2f}m, Z-Dist: {z_dist:.2f}m")
            
            if show_vision:
                cv2.polylines(draw_frame, [tag.corners.astype(int)], True, (0, 255, 0), 2)
                cv2.putText(draw_frame, str(tag.tag_id), (int(tag.center[0]), int(tag.center[1])), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            if tag.tag_id in ARENA_MAP:
                tag_info = ARENA_MAP[tag.tag_id]
                
                tx = tag.pose_t[0][0]
                tz = tag.pose_t[2][0] / SCALE  # SCALE factor from configuration
                
                # To know which way to spin if we lose sight, store the side of the last seen valid tag
                last_known_side = 1.0 if tx > 0 else -1.0
                
                alpha = math.degrees(math.atan2(tx, tz))
                h = (tag_info["wall_base_angle"] + alpha) % 360
                
                h_rad = math.radians(h)
                h_right_rad = math.radians(h - 90) 
                
                dx_world = tz * math.cos(h_rad) + tx * math.cos(h_right_rad)
                dy_world = tz * math.sin(h_rad) + tx * math.sin(h_right_rad)
                
                rx = tag_info["x"] - dx_world
                ry = tag_info["y"] - dy_world
                
                rx_list.append(rx)
                ry_list.append(ry)
                h_list.append(h)

        if show_vision:
            display_frame = draw_frame

        # 2. SENSOR FUSION
        if rx_list:
            is_localized = True
            last_seen_time = time.time()
            robot_x = sum(rx_list) / len(rx_list)
            robot_y = sum(ry_list) / len(ry_list)
            
            sum_sin = sum(math.sin(math.radians(angle)) for angle in h_list)
            sum_cos = sum(math.cos(math.radians(angle)) for angle in h_list)
            robot_heading = math.degrees(math.atan2(sum_sin, sum_cos)) % 360
            
            dx = HOME_X - robot_x
            dy = HOME_Y - robot_y
            est_distance = math.sqrt(dx**2 + dy**2)
            est_heading = math.degrees(math.atan2(dy, dx)) % 360
            
            # Shortest turn angle required (-180 to 180 directly)
            angle_diff = (est_heading - robot_heading + 180) % 360 - 180
            
            print(f"\nTags Detected: {detected_tag_ids}")
            print(f"Current Position -> X:{robot_x:.2f}m, Y:{robot_y:.2f}m, Pose:{robot_heading:.1f}deg")
            print(f"To Reach Home    -> Move: {est_distance:.2f}m | Turn: {angle_diff:.1f}deg (Target:{est_heading:.1f}deg)\n")
        else:
            print(f"Tags Detected: {detected_tag_ids} | Localized: False")

        # --- SENSOR OVERRIDE / OBSTACLE AVOIDANCE ---
        OBSTACLE_DETECTED = False
        if latest_C < STOP_DIST:
            print(f"Blocked directly ahead ({latest_C}cm)! Avoiding...")
            send_cmd('S', force=True) 
            time.sleep(0.2)
            
            send_cmd('1', force=True) 
            send_cmd('B', force=True) 
            time.sleep(0.3)
            
            if latest_L > latest_R:
                print("Turning LEFT (Left side has more space)")
                send_cmd('L', force=True) 
                last_known_side = 1.0  
            else:
                print("Turning RIGHT (Right side has more space)")
                send_cmd('R', force=True)
                last_known_side = -1.0 
                
            time.sleep(0.5) 
            send_cmd('S', force=True) 
            time.sleep(0.2) 
            
            last_seen_time = time.time() - SEARCH_TIMEOUT - 0.1 
            OBSTACLE_DETECTED = True

        elif latest_L < SIDE_DIST:
            print("Too close to LEFT -> Nudging Right")
            send_cmd('1', force=True) 
            send_cmd('R', force=True) 
            last_known_side = -1.0 
            time.sleep(0.1) 
            OBSTACLE_DETECTED = True
            
        elif latest_R < SIDE_DIST:
            print("Too close to RIGHT -> Nudging Left")
            send_cmd('1', force=True) 
            send_cmd('L', force=True) 
            last_known_side = 1.0 
            time.sleep(0.1) 
            OBSTACLE_DETECTED = True

        if OBSTACLE_DETECTED:
             continue

        # --- NAVIGATION STATE MACHINE ---
        if state == "HUNTING":
            if is_localized:
                send_cmd('S', force=True)
                if state != "WAIT_FOR_START":
                    print("\n>>> PRESS 'S' IN VIDEO WINDOW OR 'Enter' IN TERMINAL TO START MVOING <<<")
                    state = "WAIT_FOR_START"
            else:
                # If no tag is seen on boot, spin slowly to find one
                time_since_last_seen = time.time() - last_seen_time
                if time_since_last_seen > SEARCH_TIMEOUT:
                    send_cmd('1', force=True) 
                    send_cmd('R', force=True)
                    time.sleep(0.08) 
                    send_cmd('S', force=True)
                    time.sleep(0.2)  
                    frame_skip_counter = FRAME_SKIP 
            time.sleep(0.01)
            continue

        elif state == "WAIT_FOR_START":
            # Non-blocking check for user hitting Enter in the terminal
            dr, dw, de = select.select([sys.stdin], [], [], 0.0)
            if dr:
                sys.stdin.readline()
                start_motion = True
            
            if start_motion:
                state = "RETURNING_HOME"
                print("\n>>> Moving to home... <<<\n")
            
            time.sleep(0.01)
            continue

        elif state == "RETURNING_HOME":
            # Direct tracking override if home tags are visible and far away (>0.5m)
            home_tag = None
            if len(tags) > 0:
                for tag in tags:
                    if tag.tag_id in [HOME_TAG_1, HOME_TAG_2]:
                        home_tag = tag
                        break
            
            if home_tag and (home_tag.pose_t[2][0] / SCALE) > 0.5:
                # We see a home tag and it's further than 0.5m -> Direct approach like go_to_tag
                z_dist = home_tag.pose_t[2][0] / SCALE
                x_offset = home_tag.pose_t[0][0] / SCALE
                print(f"Direct Tracking HOME Tag {home_tag.tag_id} | Z={z_dist:.2f}m, X={x_offset:.2f}m")

                # Angle calculation based on camera offset
                angle_offset = math.degrees(math.atan2(x_offset, z_dist))
                print(f"Angle to Home Tag offset: {angle_offset:.2f} deg")

                if x_offset > 0.1:  # Tag is to the right
                    send_cmd('1') 
                    send_cmd('R')
                    print("Correction: Right")
                elif x_offset < -0.1: # Tag is to the left
                    send_cmd('1') 
                    send_cmd('L')
                    print("Correction: Left")
                else: 
                    # Tag is centered, move forward towards it
                    diff = abs(x_offset)
                    if diff < 0.05:
                        send_cmd('3') # High speed if very centered
                    elif diff < 0.08:
                        send_cmd('2') # Medium speed if slightly off
                    else:
                        send_cmd('1') # Slow speed if more off
                        
                    send_cmd('F')
                    print("Centered: Moving Forward")
                
                # Keep SLAM state updated to avoid losing localization immediately after
                last_seen_time = time.time()
                continue # Skip global calculation this loop

            # If we lose sight of ALL arena tags, we don't know our global position.
            # We spin to find ANY tag to re-localize. 
            if not is_localized:
                time_since_last_seen = time.time() - last_seen_time
                if time_since_last_seen > SEARCH_TIMEOUT:
                    search_dir = 'R' if last_known_side > 0 else 'L'
                    dir_name = 'RIGHT' if search_dir == 'R' else 'LEFT'
                    print(f"Lost ALL localization tags... Spinning {dir_name} to find any arena tag...")
                    
                    send_cmd('1', force=True) 
                    send_cmd(search_dir, force=True)
                    time.sleep(0.08) 
                    
                    send_cmd('S', force=True)
                    time.sleep(0.2)  
                    
                    frame_skip_counter = FRAME_SKIP 
                else:
                    # We briefly lost sight of tags, but rely on our last known location
                    # and allow the momentum/last command to carry us forward.
                    pass
            else:
                # We know exactly where we are using ANY visible map tag!
                # Automatically calculat distance and angle to Home Coordinates
                dx = HOME_X - robot_x
                dy = HOME_Y - robot_y
                distance_to_home = math.sqrt(dx**2 + dy**2)
                
                if distance_to_home < 0.10: # Arrived within 10cm!
                    print(f"ARRIVED HOME! Distance to home: {distance_to_home:.2f}m")
                    send_cmd('S', force=True)
                    state = "DONE"
                else:
                    target_heading = math.degrees(math.atan2(dy, dx)) % 360
                    angle_diff = (target_heading - robot_heading + 180) % 360 - 180
                    
                    print(f"Dist to Home: {distance_to_home:.2f}m | Target Head: {target_heading:.1f}* | Turn required: {angle_diff:.1f}*")
                    
                    # Steer the robot toward the target heading in discrete steps
                    if angle_diff > 15:
                        print(f"-> Turning LEFT to align ({angle_diff:.1f}deg)")
                        send_cmd('1', force=True)
                        send_cmd('L', force=True)
                        time.sleep(0.04) # Back to 40ms to overcome static friction 
                        send_cmd('S', force=True)
                        time.sleep(0.1)  # 100ms OFF (let robot settle)
                        frame_skip_counter = FRAME_SKIP
                    elif angle_diff < -15:
                        print(f"-> Turning RIGHT to align ({angle_diff:.1f}deg)")
                        send_cmd('1', force=True)
                        send_cmd('R', force=True)
                        time.sleep(0.04) # Back to 40ms to overcome static friction 
                        send_cmd('S', force=True)
                        time.sleep(0.1)  # 100ms OFF (let robot settle)
                        frame_skip_counter = FRAME_SKIP
                    else:
                        print("-> Aligned! Moving FORWARD to Home")
                        send_cmd('3') # High speed straight forward
                        send_cmd('F')

        elif state == "DONE":
            break
            
        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nEmergency Stop Triggered.")
finally:
    running = False
    send_cmd('S')
    time.sleep(0.1)
    picam2.stop()
    print("Clean exit.")