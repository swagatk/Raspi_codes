import cv2
import numpy as np
from pupil_apriltags import Detector
import serial
import time
import math

# --- CONFIGURATION ---
CAMERA_PARAMS = (954.4949188171072, 955.5979729485147, 332.0798756650343, 245.67451277016548)
TAG_SIZE = 0.10 # 10 centimeters = 0.10 meters

# --- SERIAL SETUP ---
try:
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    time.sleep(2)
except Exception as e:
    print(f"Serial Error: {e}")
    ser = None

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
    
    # Bottom Wall (Y = 0) - Staring at it means facing South (270 deg)
    12: {"x": xmax, "y": gap1, "wall_base_angle": 270},
    13: {"x": xmax, "y": gap2, "wall_base_angle": 270},
    14: {"x": xmax, "y": gap2, "wall_base_angle": 270},
    15: {"x": xmax, "y": gap3, "wall_base_angle": 270},
    16: {"x": xmax, "y": gap2, "wall_base_angle": 270},
    17: {"x": xmax, "y": gap2, "wall_base_angle": 270},
    # Right Wall (X = 2.0) - Staring at it means facing East (0 deg)
    6: {"x": gap1, "y": 0, "wall_base_angle": 0},
    7: {"x": gap2, "y": 0, "wall_base_angle": 0},
    8: {"x": gap2, "y": 0, "wall_base_angle": 0},
    9: {"x": gap3, "y": 0, "wall_base_angle": 0},
    10: {"x": gap2, "y": 0, "wall_base_angle": 0},
    11: {"x": gap2, "y": 0, "wall_base_angle": 0},
    # Top Wall (Y = 2.0) - Staring at it means facing North (90 deg)
    0: {"x": 0.0, "y": gap1, "wall_base_angle": 90},
    1: {"x": 0.0, "y": gap2, "wall_base_angle": 90},
    2: {"x": 0.0, "y": gap2, "wall_base_angle": 90},
    3: {"x": 0.0, "y": gap3, "wall_base_angle": 90},
    4: {"x": 0.0, "y": gap2, "wall_base_angle": 90},
    5: {"x": 0.0, "y": gap2, "wall_base_angle": 90},
    # Left Wall (X = 0) - Staring at it means facing West (180 deg)
    18: {"x": gap1, "y": ymax, "wall_base_angle": 180},
    19: {"x": gap2, "y": ymax, "wall_base_angle": 180},
    20: {"x": gap2, "y": ymax, "wall_base_angle": 180},
    21: {"x": gap3, "y": ymax, "wall_base_angle": 180},
    22: {"x": gap2, "y": ymax, "wall_base_angle": 180},
    23: {"x": gap2, "y": ymax, "wall_base_angle": 180}
}

# Define Home (between tag 20 21)
HOME_X = 1.0 
HOME_Y = 1.7 

# --- INITIALIZATION ---
camera = cv2.VideoCapture(0, cv2.CAP_V4L2)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
detector = Detector(families='tag36h11', quad_decimate=2.0)

state = "HUNTING" 
robot_x, robot_y, robot_heading = 0.0, 0.0, 0.0

print("--- MULTI-TAG SLAM SYSTEM ONLINE ---")

def command_robot(cmd):
    if ser: ser.write(cmd.encode('utf-8'))

try:
    while True:
        ret, frame = camera.read()
        if not ret: break
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = detector.detect(gray, estimate_tag_pose=True, camera_params=CAMERA_PARAMS, tag_size=TAG_SIZE)
        
        rx_list, ry_list, h_list = [], [], []
        sees_known_tag = False
        
        # 1. MULTI-TAG TRIANGULATION
        for tag in tags:
            if tag.tag_id in ARENA_MAP:
                tag_info = ARENA_MAP[tag.tag_id]
                
                # tx: lateral distance (right is positive)
                # tz: forward distance
                tx = tag.pose_t[0][0]
                tz = tag.pose_t[2][0]
                
                # --- Exact Trigonometric Position Math ---
                # 1. Calculate the exact compass heading of the robot
                alpha = math.degrees(math.atan2(tx, tz))
                h = (tag_info["wall_base_angle"] + alpha) % 360
                
                # 2. Convert camera view distances into Global Map distances
                h_rad = math.radians(h)
                h_right_rad = math.radians(h - 90) # Camera's Right axis
                
                dx_world = tz * math.cos(h_rad) + tx * math.cos(h_right_rad)
                dy_world = tz * math.sin(h_rad) + tx * math.sin(h_right_rad)
                
                # 3. Apply offset to known tag location to find robot
                rx = tag_info["x"] - dx_world
                ry = tag_info["y"] - dy_world
                
                rx_list.append(rx)
                ry_list.append(ry)
                h_list.append(h)

        # 2. SENSOR FUSION (Averaging the data)
        if rx_list:
            sees_known_tag = True
            robot_x = sum(rx_list) / len(rx_list)
            robot_y = sum(ry_list) / len(ry_list)
            
            # We use "Circular Mean" to average headings so 359 deg and 1 deg average to 0, not 180!
            sum_sin = sum(math.sin(math.radians(angle)) for angle in h_list)
            sum_cos = sum(math.cos(math.radians(angle)) for angle in h_list)
            robot_heading = math.degrees(math.atan2(sum_sin, sum_cos)) % 360

        # --- NAVIGATION STATE MACHINE ---
        if state == "HUNTING":
            # (Insert your ball hunting logic here)
            # If inventory is full, trigger the return sequence:
            # state = "RETURNING_HOME"
            pass

        elif state == "RETURNING_HOME":
            if not sees_known_tag:
                print("Lost! Spinning to find a wall tag...")
                command_robot('L') 
            else:
                # Calculate vector to Home
                dx = HOME_X - robot_x
                dy = HOME_Y - robot_y
                distance_to_home = math.sqrt(dx**2 + dy**2)
                
                if distance_to_home < 0.25: # Arrived within 25cm!
                    print("ARRIVED HOME!")
                    command_robot('S')
                    state = "DONE"
                else:
                    # Calculate required angle to face Home
                    target_heading = math.degrees(math.atan2(dy, dx)) % 360
                    
                    # Calculate the shortest turn direction
                    angle_diff = (target_heading - robot_heading + 180) % 360 - 180
                    
                    print(f"Dist: {distance_to_home:.2f}m | Head: {robot_heading:.1f}* | Target: {target_heading:.1f}*")
                    
                    # Steer the robot toward the target heading
                    if angle_diff > 15:
                        command_robot('L')
                    elif angle_diff < -15:
                        command_robot('R')
                    else:
                        command_robot('F')

        # Visual Debugging HUD
        cv2.putText(frame, f"State: {state} | Tags Avg: {len(rx_list)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        if sees_known_tag:
            cv2.putText(frame, f"X:{robot_x:.2f} Y:{robot_y:.2f} Head:{robot_heading:.0f}*", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        cv2.imshow("Absolute SLAM", frame)
        if cv2.waitKey(1) == ord('q'):
            break

finally:
    command_robot('S')
    camera.release()
    cv2.destroyAllWindows()