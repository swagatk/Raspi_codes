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

# --- HOME TAGS (Goal Location) ---
# Specify two adjacent home tags - robot will navigate to the midpoint
HOME_TAGS = [14, 15]  # Tags on the North wall

# --- ABSOLUTE MAP (The "GPS" Coordinates) ---
gap1 = 0.15
gap2 = 0.3
gap3 = 0.5  
xmax = 2.0 
ymax = 2.0
ARENA_MAP = {
    # Top Wall (Y = ymax) - Staring at it means facing West (180 deg)
    12: {"x": xmax - gap1, "y": ymax, "wall_base_angle": 180},
    13: {"x": xmax - (gap1+gap2), "y": ymax, "wall_base_angle": 180},
    14: {"x": xmax - (gap1+gap2*2), "y": ymax, "wall_base_angle": 180},
    15: {"x": xmax - (gap1+gap2*2+gap3), "y": ymax, "wall_base_angle": 180},
    16: {"x": xmax - (gap1+gap2*3+gap3), "y": ymax, "wall_base_angle": 180},
    17: {"x": xmax - (gap1+gap2*4+gap3), "y": ymax, "wall_base_angle": 180},
    
    # Right Wall (X = xmax) - Staring at it means facing North (90 deg)
    6: {"x": xmax, "y": gap1, "wall_base_angle": 90},
    7: {"x": xmax, "y": gap1+gap2, "wall_base_angle": 90},
    8: {"x": xmax, "y": gap1+gap2*2, "wall_base_angle": 90},
    9: {"x": xmax, "y": gap1+gap2*2+gap3, "wall_base_angle": 90},
    10: {"x": xmax, "y": gap1+gap2*3+gap3, "wall_base_angle": 90},
    11: {"x": xmax, "y": gap1+gap2*4+gap3, "wall_base_angle": 90},
    
    # Bottom Wall (Y = 0.0) - Staring at it means facing East (0 deg)
    0: {"x": gap1, "y": 0.0, "wall_base_angle": 0},
    1: {"x": gap1+gap2, "y": 0.0, "wall_base_angle": 0},
    2: {"x": gap1+gap2*2, "y": 0.0, "wall_base_angle": 0},
    3: {"x": gap1+gap2*2+gap3, "y": 0.0, "wall_base_angle": 0},
    4: {"x": gap1+gap2*3+gap3, "y": 0.0, "wall_base_angle": 0},
    5: {"x": gap1+gap2*4+gap3, "y": 0.0, "wall_base_angle": 0},
    
    # Left Wall (X = 0) - Staring at it means facing South (270) deg)
    18: {"x": 0, "y": ymax - gap1, "wall_base_angle": 270},
    19: {"x": 0, "y": ymax - (gap1+gap2), "wall_base_angle": 270},
    20: {"x": 0, "y": ymax - (gap1+gap2*2), "wall_base_angle": 270},
    21: {"x": 0, "y": ymax - (gap1+gap2*2+gap3), "wall_base_angle": 270},
    22: {"x": 0, "y": ymax - (gap1+gap2*3+gap3), "wall_base_angle": 270},
    23: {"x": 0, "y": ymax - (gap1+gap2*4+gap3), "wall_base_angle": 270}
}


def get_home_position():
    home_points = [ARENA_MAP[tag_id] for tag_id in HOME_TAGS if tag_id in ARENA_MAP]
    if not home_points:
        return 0.0, 0.0
    home_x = sum(point["x"] for point in home_points) / len(home_points)
    home_y = sum(point["y"] for point in home_points) / len(home_points)
    return home_x, home_y


def get_wall_axes(tag_id):
    if 0 <= tag_id <= 5:
        return (1.0, 0.0), (0.0, 1.0)
    if 6 <= tag_id <= 11:
        return (0.0, 1.0), (-1.0, 0.0)
    if 12 <= tag_id <= 17:
        return (-1.0, 0.0), (0.0, -1.0)
    return (0.0, -1.0), (1.0, 0.0)


def normalize_rotation(angle_degrees):
    return (angle_degrees + 180.0) % 360.0 - 180.0


def heading_from_vector(vec_x, vec_y):
    return math.degrees(math.atan2(vec_y, vec_x)) % 360.0

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
            
            # Add Wall Labels
            ax.text(1.0, -0.15, "East", ha='center', va='center', weight='bold', color='darkblue')
            ax.text(-0.15, 1.0, "South", ha='center', va='center', rotation=90, weight='bold', color='darkblue')
            ax.text(2.15, 1.0, "North", ha='center', va='center', rotation=-90, weight='bold', color='darkblue')
            ax.text(1.0, 2.15, "West", ha='center', va='center', weight='bold', color='darkblue')
            
            # Plot Tags and Home location
            home_x, home_y = get_home_position()
            for tid, tinfo in ARENA_MAP.items():
                tx = tinfo["x"]
                ty = tinfo["y"]
                if tid in HOME_TAGS:
                    c = 'green'
                    ax.plot(tx, ty, marker='s', color=c, markersize=10, linestyle='None')
                else:
                    c = 'black'
                    ax.plot(tx, ty, marker='s', color=c, markersize=8, linestyle='None')
                    
                ax.text(tx, ty, f' {tid}', color=c, fontsize=9, verticalalignment='bottom')
            
            # Mark home location
            ax.plot(home_x, home_y, marker='*', color='green', markersize=15, linestyle='None', label='Home')
                
            if is_localized:
                ax.plot(robot_x, robot_y, marker='o', color='red', markersize=12, linestyle='None', label='Robot')
                
                # Current heading arrow (red)
                rad = math.radians(robot_heading)
                # 0°=North(+X), 90°=West(+Y), 180°=South(-X), 270°=East(-Y)
                dx_head = 0.2 * math.cos(rad)
                dy_head = 0.2 * math.sin(rad)
                ax.arrow(robot_x, robot_y, dx_head, dy_head, 
                         head_width=0.04, head_length=0.06, fc='red', ec='red')
                
                # Calculate desired heading to home (blue arrow)
                desired_heading = heading_from_vector(home_x - robot_x, home_y - robot_y)
                
                rad_desired = math.radians(desired_heading)
                dx_desired = 0.25 * math.cos(rad_desired)
                dy_desired = 0.25 * math.sin(rad_desired)
                ax.arrow(robot_x, robot_y, dx_desired, dy_desired, 
                         head_width=0.04, head_length=0.06, fc='blue', ec='blue', alpha=0.7)
                
                # Calculate rotation needed (positive=CCW, negative=CW)
                rotation_needed = normalize_rotation(desired_heading - robot_heading)
                
                # Distance to home
                distance_to_home = math.sqrt((home_x - robot_x)**2 + (home_y - robot_y)**2)
                
                info_text = (f"=== TELEMETRY ===\n\n"
                             f"Pos X: {robot_x:.2f} m\n"
                             f"Pos Y: {robot_y:.2f} m\n"
                             f"Heading: {robot_heading:.1f}*\n\n"
                             f"=== NAVIGATION ===\n\n"
                             f"Home: ({home_x:.2f},{home_y:.2f})\n"
                             f"Desired: {desired_heading:.1f}*\n"
                             f"Rotate: {rotation_needed:+.1f}*\n"
                             f"Distance: {distance_to_home:.2f}m\n")
                
                fig.text(0.75, 0.5, info_text, transform=fig.transFigure,
                         fontsize=11, ha='left', va='center', family='monospace',
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
        
        pose_estimates = []
        mapped_tag_observations = []
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
                tangent_vec, inward_normal_vec = get_wall_axes(tag.tag_id)

                cam_pos_in_tag = -np.dot(R.T, tag.pose_t)
                local_tangent = -cam_pos_in_tag[0][0] / SCALE
                local_inward = -cam_pos_in_tag[2][0] / SCALE

                rx = tag_info["x"] + local_tangent * tangent_vec[0] + local_inward * inward_normal_vec[0]
                ry = tag_info["y"] + local_tangent * tangent_vec[1] + local_inward * inward_normal_vec[1]

                weight = 1.0 / max(z_dist, 0.05)
                pose_estimates.append((rx, ry, weight))
                mapped_tag_observations.append((tag.tag_id, weight))

                print(
                    f"    Tag {tag.tag_id}: local_t={local_tangent:.2f}, "
                    f"local_n={local_inward:.2f}, pos=({rx:.2f},{ry:.2f})"
                )

        if show_vision:
            display_frame = draw_frame

        is_localized = False
        
        if pose_estimates:
            is_localized = True
            total_weight = sum(weight for _, _, weight in pose_estimates)
            robot_x = sum(rx * weight for rx, _, weight in pose_estimates) / total_weight
            robot_y = sum(ry * weight for _, ry, weight in pose_estimates) / total_weight
            
            # Ensure position is within arena bounds (always positive)
            robot_x = max(0.0, min(robot_x, 2.0))
            robot_y = max(0.0, min(robot_y, 2.0))

            heading_vec_x = 0.0
            heading_vec_y = 0.0
            for tag_id, weight in mapped_tag_observations:
                tag_info = ARENA_MAP[tag_id]
                vec_x = tag_info["x"] - robot_x
                vec_y = tag_info["y"] - robot_y
                vec_norm = math.hypot(vec_x, vec_y)
                if vec_norm > 1e-6:
                    heading_vec_x += (vec_x / vec_norm) * weight
                    heading_vec_y += (vec_y / vec_norm) * weight

            if abs(heading_vec_x) > 1e-6 or abs(heading_vec_y) > 1e-6:
                robot_heading = heading_from_vector(heading_vec_x, heading_vec_y)
            else:
                robot_heading = 0.0
            
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
