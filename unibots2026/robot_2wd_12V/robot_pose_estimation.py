import cv2
import numpy as np
from pupil_apriltags import Detector
import time
import math
import sys
import threading
from picamera2 import Picamera2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure

# --- CONFIGURATION ---
CAMERA_PARAMS = (907.462397724348, 908.550833315007, 358.40056240558073, 246.47297678800183)
TAG_SIZE = 0.10 # 10 centimeters = 0.10 meters
SCALE = 2.0     # Scale factor for Picamera distance calibration
FRAME_SKIP = 3  # Only run detection every Nth frame

# --- ABSOLUTE MAP (The "GPS" Coordinates) ---
gap1 = 0.15
gap2 = 0.3
gap3 = 0.5  
xmax = 2.0 
ymax = 2.0
ARENA_MAP = {
    # Right Wall (Y = ymax) - Staring at it means facing West (180 deg)
    12: {"x": xmax - gap1, "y": ymax, "wall_base_angle": 180},
    13: {"x": xmax - (gap1+gap2), "y": ymax, "wall_base_angle": 180},
    14: {"x": xmax - (gap1+gap2*2), "y": ymax, "wall_base_angle": 180},
    15: {"x": xmax - (gap1+gap2*2+gap3), "y": ymax, "wall_base_angle": 180},
    16: {"x": xmax - (gap1+gap2*3+gap3), "y": ymax, "wall_base_angle": 180},
    17: {"x": xmax - (gap1+gap2*4+gap3), "y": ymax, "wall_base_angle": 180},
    
    # Top Wall (X = xmax) - Staring at it means facing South (90 deg)
    6: {"x": xmax, "y": gap1, "wall_base_angle": 90},
    7: {"x": xmax, "y": gap1+gap2, "wall_base_angle": 90},
    8: {"x": xmax, "y": gap1+gap2*2, "wall_base_angle": 90},
    9: {"x": xmax, "y": gap1+gap2*2+gap3, "wall_base_angle": 90},
    10: {"x": xmax, "y": gap1+gap2*3+gap3, "wall_base_angle": 90},
    11: {"x": xmax, "y": gap1+gap2*4+gap3, "wall_base_angle": 90},
    
    # Left Wall (Y = 0.0) - Staring at it means facing East (0 deg)
    0: {"x": gap1, "y": 0.0, "wall_base_angle": 0},
    1: {"x": gap1+gap2, "y": 0.0, "wall_base_angle": 0},
    2: {"x": gap1+gap2*2, "y": 0.0, "wall_base_angle": 0},
    3: {"x": gap1+gap2*2+gap3, "y": 0.0, "wall_base_angle": 0},
    4: {"x": gap1+gap2*3+gap3, "y": 0.0, "wall_base_angle": 0},
    5: {"x": gap1+gap2*4+gap3, "y": 0.0, "wall_base_angle": 0},
    
    # Bottom Wall (X = 0) - Staring at it means facing North (270 deg)
    18: {"x": 0, "y": ymax - gap1, "wall_base_angle": 270},
    19: {"x": 0, "y": ymax - (gap1+gap2), "wall_base_angle": 270},
    20: {"x": 0, "y": ymax - (gap1+gap2*2), "wall_base_angle": 270},
    21: {"x": 0, "y": ymax - (gap1+gap2*2+gap3), "wall_base_angle": 270},
    22: {"x": 0, "y": ymax - (gap1+gap2*3+gap3), "wall_base_angle": 270},
    23: {"x": 0, "y": ymax - (gap1+gap2*4+gap3), "wall_base_angle": 270}
}

running = True

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

capture_thread = threading.Thread(target=camera_capture_thread)
capture_thread.daemon = True
capture_thread.start()

# --- OPTIMIZED VIDEO DISPLAY THREAD ---
show_vision = True
display_frame = None
robot_x, robot_y, robot_heading = 0.0, 0.0, 0.0
is_localized = False

def video_display_thread():
    global display_frame, show_vision, running
    global robot_x, robot_y, robot_heading, is_localized

    cv2.namedWindow("Robot Vision", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Robot Vision", 320, 240)
    
    cv2.namedWindow("Arena Map", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Arena Map", 800, 600)
    
    fig = Figure(figsize=(8, 6), dpi=100)
    canvas = FigureCanvasAgg(fig)
    ax = fig.add_axes([0.1, 0.1, 0.6, 0.8])
    last_map_update = 0
    
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
            
        now = time.time()
        if now - last_map_update > 0.2:
            last_map_update = now
            ax.clear()
            ax.set_xlim(-0.2, 2.2)
            ax.set_ylim(-0.2, 2.2)
            ax.set_aspect('equal')
            ax.set_title("Robot Pose Estimation")
            ax.grid(True)
            
            # Plot Tags
            for tid, tinfo in ARENA_MAP.items():
                tx = tinfo["x"]
                ty = tinfo["y"]
                c = 'black'
                
                label = 'Arena Tag' if tid == 0 else ""
                
                if label:
                    ax.plot(tx, ty, marker='s', color=c, markersize=8, linestyle='None', label=label)
                else:
                    ax.plot(tx, ty, marker='s', color=c, markersize=8, linestyle='None')
                    
                ax.text(tx, ty, f' {tid}', color=c, fontsize=9, verticalalignment='bottom')
                
            if is_localized:
                ax.plot(robot_x, robot_y, marker='o', color='red', markersize=12, linestyle='None', label='Robot')
                
                rad = math.radians(robot_heading)
                dx_head = 0.2 * math.sin(rad)
                dy_head = -0.2 * math.cos(rad)
                ax.arrow(robot_x, robot_y, dx_head, dy_head, 
                         head_width=0.04, head_length=0.06, fc='red', ec='red')
                
                info_text = (f"=== TELEMETRY ===\n\n"
                             f"Pos X: {robot_x:.2f} m\n"
                             f"Pos Y: {robot_y:.2f} m\n"
                             f"Heading: {robot_heading:.1f}*\n")
                
                fig.text(0.75, 0.5, info_text, transform=fig.transFigure,
                         fontsize=12, ha='left', va='center', family='monospace',
                         bbox=dict(facecolor='white', alpha=0.9, edgecolor='black', boxstyle='round,pad=1'))
            else:
               info_text = "Status: LOST\nNo Tags Visible"
               fig.text(0.75, 0.5, info_text, transform=fig.transFigure,
                         fontsize=12, ha='left', va='center', family='monospace', color='red',
                         bbox=dict(facecolor='white', alpha=0.9, edgecolor='red', boxstyle='round,pad=1'))

            ax.legend(loc='upper left', bbox_to_anchor=(1.05, 1), borderaxespad=0.)
            
            canvas.draw()
            buf = canvas.buffer_rgba()
            map_img = np.asarray(buf)
            map_bgr = cv2.cvtColor(map_img, cv2.COLOR_RGBA2BGR)
            cv2.imshow("Arena Map", map_bgr)
            
        key = cv2.waitKey(30) & 0xFF
        if key == ord('q') or key == ord('Q'):
            running = False
        elif key == ord('c') or key == ord('C'):
            show_vision = not show_vision

display_t = threading.Thread(target=video_display_thread)
display_t.daemon = True
display_t.start()

# --- MAIN DETECTION LOOP ---
print("--- POSE ESTIMATION SYSTEM ONLINE ---")
frame_skip_counter = 0

try:
    while running:
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
        detected_tag_ids = []
        
        if tags:
            print(f"\n--- Frame Data ---")
            
        draw_frame = frame.copy() if show_vision else None

        for tag in tags:
            detected_tag_ids.append(tag.tag_id)
            
            x_dist = tag.pose_t[0][0] / SCALE
            y_dist = tag.pose_t[1][0] / SCALE
            z_dist = tag.pose_t[2][0] / SCALE
            print(f"  > Tag {tag.tag_id}: X: {x_dist:.2f}m, Y: {y_dist:.2f}m, Z: {z_dist:.2f}m")
            
            if show_vision:
                cv2.polylines(draw_frame, [tag.corners.astype(int)], True, (0, 255, 0), 2)
                cv2.putText(draw_frame, str(tag.tag_id), (int(tag.center[0]), int(tag.center[1])),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            if tag.tag_id in ARENA_MAP:
                tag_info = ARENA_MAP[tag.tag_id]
                
                R = tag.pose_R
                t = tag.pose_t
                
                R_T = np.transpose(R)
                cam_pos_tag = -np.dot(R_T, t)
                
                local_x = cam_pos_tag[0][0] / SCALE     
                local_z = cam_pos_tag[2][0] / SCALE      
                
                wall_angle = tag_info["wall_base_angle"]
                
                if wall_angle == 0:     
                    rx = tag_info["x"] - local_x
                    ry = tag_info["y"] + local_z
                elif wall_angle == 90:  
                    rx = tag_info["x"] + local_z
                    ry = tag_info["y"] + local_x
                elif wall_angle == 180: 
                    rx = tag_info["x"] + local_x
                    ry = tag_info["y"] - local_z
                elif wall_angle == 270: 
                    rx = tag_info["x"] - local_z
                    ry = tag_info["y"] - local_x
                else: 
                    rad = math.radians(wall_angle)
                    rx = tag_info["x"] + local_z * math.sin(rad) - local_x * math.cos(rad)
                    ry = tag_info["y"] + local_z * math.cos(rad) + local_x * math.sin(rad)
                
                rad2 = math.radians(wall_angle)
                tag_x_vec = (math.cos(rad2), -math.sin(rad2))
                tag_z_vec = (-math.sin(rad2), math.cos(rad2))

                cam_forward_tag_x = R[2][0]
                cam_forward_tag_z = R[2][2]
                
                global_cam_vec_x = cam_forward_tag_x * tag_x_vec[0] + cam_forward_tag_z * tag_z_vec[0]
                global_cam_vec_y = cam_forward_tag_x * tag_x_vec[1] + cam_forward_tag_z * tag_z_vec[1]
                
                h = math.degrees(math.atan2(global_cam_vec_x, -global_cam_vec_y))
                
                rx_list.append(rx)
                ry_list.append(ry)
                h_list.append(h)

        if show_vision:
            display_frame = draw_frame

        is_localized = False
        
        if rx_list:
            is_localized = True
            robot_x = sum(rx_list) / len(rx_list)
            robot_y = sum(ry_list) / len(ry_list)
            
            sum_sin = sum(math.sin(math.radians(angle)) for angle in h_list)
            sum_cos = sum(math.cos(math.radians(angle)) for angle in h_list)
            r_head = math.degrees(math.atan2(sum_sin, sum_cos))
            robot_heading = (r_head + 180) % 360 - 180
            
            print(f"Current Position -> X:{robot_x:.2f}m, Y:{robot_y:.2f}m, Pose:{robot_heading:.1f}deg")
        else:
            if detected_tag_ids:
                print(f"Tags Detected (Unmapped): {detected_tag_ids} | Localized: False")

        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nStopping...")
finally:
    running = False
    time.sleep(0.1)
    picam2.stop()
    print("Clean exit.")
