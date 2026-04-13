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
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure

# --- CONFIGURATION ---
CAMERA_PARAMS =  (907.462397724348, 908.550833315007, 358.40056240558073, 246.47297678800183)
TAG_SIZE = 0.10 # 10 centimeters = 0.10 meters
SCALE = 2.0     # Scale factor for Picamera distance calibration

STOP_DIST = 20             # Front ultrasonic sensor stop distance (cm)
SIDE_DIST = 20             # Side ultrasonic sensor push away distance (cm)
SENSOR_SMOOTHING_COUNT = 3 # Number of sensor readings to average to prevent false positives
SEARCH_TIMEOUT = 0.7       # If tag unseen for 0.7 seconds, start scanning for it
FRAME_SKIP = 3             # Only run detection every Nth frame
AUTO_START_ON_LOCALIZATION = True

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

# --- HOME SETTINGS ---
HOME_TAGS = [14, 15]
FINAL_ALIGN_DISTANCE_M = 0.5
FINAL_STOP_DISTANCE_CM = 50
TAG_REACQUIRE_BACKUP_DISTANCE_CM = 80
TAG_REACQUIRE_BACKUP_DURATION_S = 0.15 #0.35
TAG_REACQUIRE_BACKUP_COOLDOWN_S = 0.8
ROUTE_HEADING_TOL_DEG = 20.0
ROUTE_COARSE_TURN_THRESHOLD_DEG = 40.0
ROUTE_COARSE_TURN_DURATION_S = 0.14
ROUTE_FINE_TURN_DURATION_S = 0.05
ROUTE_COARSE_COMMIT_TIME_S = 0.6
ROUTE_DIRECTION_HOLD_TIME_S = 1.5
SMOOTH_APPROACH_DISTANCE_M = 0.75
SMOOTH_APPROACH_HEADING_TOL_DEG = 30.0
SMOOTH_APPROACH_SPEED = '1'
FINAL_HEADING_TOL_DEG = 15.0
HOME_X_TOL_M = 0.08


def get_wall_axes(tag_id):
    if 0 <= tag_id <= 5:
        return (1.0, 0.0), (0.0, 1.0)
    if 6 <= tag_id <= 11:
        return (0.0, 1.0), (-1.0, 0.0)
    if 12 <= tag_id <= 17:
        return (-1.0, 0.0), (0.0, -1.0)
    return (0.0, -1.0), (1.0, 0.0)


def heading_from_vector(vec_x, vec_y):
    return math.degrees(math.atan2(vec_y, vec_x)) % 360.0


def normalize_rotation(angle_degrees):
    return (angle_degrees + 180.0) % 360.0 - 180.0


def blend_heading_degrees(start_heading_deg, end_heading_deg, blend_ratio):
    blend_ratio = max(0.0, min(1.0, blend_ratio))
    start_rad = math.radians(start_heading_deg)
    end_rad = math.radians(end_heading_deg)
    vec_x = (1.0 - blend_ratio) * math.cos(start_rad) + blend_ratio * math.cos(end_rad)
    vec_y = (1.0 - blend_ratio) * math.sin(start_rad) + blend_ratio * math.sin(end_rad)
    return heading_from_vector(vec_x, vec_y)


def get_navigation_guidance(distance_to_home, route_heading, final_heading, robot_heading):
    if distance_to_home <= SMOOTH_APPROACH_DISTANCE_M:
        blend_span = max(SMOOTH_APPROACH_DISTANCE_M - FINAL_ALIGN_DISTANCE_M, 1e-6)
        blend_ratio = (SMOOTH_APPROACH_DISTANCE_M - distance_to_home) / blend_span
        nav_heading = blend_heading_degrees(route_heading, final_heading, blend_ratio)
        nav_tol = SMOOTH_APPROACH_HEADING_TOL_DEG
        nav_mode = "SMOOTH_APPROACH"
    else:
        nav_heading = route_heading
        nav_tol = ROUTE_HEADING_TOL_DEG
        nav_mode = "ROUTE"

    nav_turn = normalize_rotation(nav_heading - robot_heading)
    return nav_heading, nav_turn, nav_tol, nav_mode


def get_home_target():
    home_points = [ARENA_MAP[tag_id] for tag_id in HOME_TAGS if tag_id in ARENA_MAP]
    mid_x = sum(point["x"] for point in home_points) / len(home_points)
    mid_y = sum(point["y"] for point in home_points) / len(home_points)
    _, inward_normal = get_wall_axes(HOME_TAGS[0])
    approach_heading = heading_from_vector(-inward_normal[0], -inward_normal[1])
    return mid_x, mid_y, approach_heading


def estimate_robot_pose(tags):
    pose_estimates = []
    mapped_tag_observations = []

    for tag in tags:
        if tag.tag_id not in ARENA_MAP:
            continue

        tag_info = ARENA_MAP[tag.tag_id]
        tangent_vec, inward_normal_vec = get_wall_axes(tag.tag_id)
        cam_pos_in_tag = -np.dot(tag.pose_R.T, tag.pose_t)
        local_tangent = -cam_pos_in_tag[0][0] / SCALE
        local_inward = -cam_pos_in_tag[2][0] / SCALE

        rx = tag_info["x"] + local_tangent * tangent_vec[0] + local_inward * inward_normal_vec[0]
        ry = tag_info["y"] + local_tangent * tangent_vec[1] + local_inward * inward_normal_vec[1]
        weight = 1.0 / max(tag.pose_t[2][0] / SCALE, 0.05)

        pose_estimates.append((tag.tag_id, rx, ry, local_tangent, local_inward, weight))
        mapped_tag_observations.append((tag.tag_id, weight))

    if not pose_estimates:
        return None

    total_weight = sum(weight for _, _, _, _, _, weight in pose_estimates)
    robot_x = sum(rx * weight for _, rx, _, _, _, weight in pose_estimates) / total_weight
    robot_y = sum(ry * weight for _, _, ry, _, _, weight in pose_estimates) / total_weight
    robot_x = max(0.0, min(robot_x, xmax))
    robot_y = max(0.0, min(robot_y, ymax))

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

    robot_heading = heading_from_vector(heading_vec_x, heading_vec_y) if abs(heading_vec_x) > 1e-6 or abs(heading_vec_y) > 1e-6 else 0.0
    return robot_x, robot_y, robot_heading, pose_estimates


HOME_X, HOME_Y, HOME_APPROACH_HEADING = get_home_target()

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
robot_x, robot_y, robot_heading = 0.0, 0.0, 0.0
is_localized = False

def video_display_thread():
    global display_frame, show_vision, running, start_motion
    global robot_x, robot_y, robot_heading, is_localized

    cv2.namedWindow("Robot Vision", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Robot Vision", 320, 240)
    
    cv2.namedWindow("Arena Map", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Arena Map", 800, 600)
    
    # Make figure wider to accommodate an external legend / info box
    fig = Figure(figsize=(8, 6), dpi=100)
    canvas = FigureCanvasAgg(fig)
    # Give the main plot less width (e.g. 75%) so the right side is free for text
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
            
        # Draw the matplotlib Arena map roughly 5 times a second
        now = time.time()
        if now - last_map_update > 0.2:
            last_map_update = now
            ax.clear()
            ax.set_xlim(-0.2, 2.2)
            ax.set_ylim(-0.2, 2.2)
            ax.set_aspect('equal')
            ax.set_title("Robot Arena")
            ax.grid(True)
            
            # Plot Tags
            for tid, tinfo in ARENA_MAP.items():
                tx = tinfo["x"]
                ty = tinfo["y"]
                is_home = (tid in HOME_TAGS)
                c = 'blue' if is_home else 'black'
                
                # Only add labels for the legend once
                label = 'Home Tag' if (is_home and tid == HOME_TAGS[0]) else ('Arena Tag' if (not is_home and tid == 0) else "")
                
                if label:
                    ax.plot(tx, ty, marker='s', color=c, markersize=8, linestyle='None', label=label)
                else:
                    ax.plot(tx, ty, marker='s', color=c, markersize=8, linestyle='None')
                    
                ax.text(tx, ty, f' {tid}', color=c, fontsize=9, verticalalignment='bottom')
                
            # Plot generic Home center reference
            ax.plot(HOME_X, HOME_Y, marker='x', color='dodgerblue', markersize=10, linestyle='None', label='Home Target')
            
            if is_localized:
                # Robot Position (Filled Red Circle)
                ax.plot(robot_x, robot_y, marker='o', color='red', markersize=12, linestyle='None', label='Robot')
                
                # Robot orientation arrow using the same 0deg=+X convention as localization
                rad = math.radians(robot_heading)
                dx_head = 0.2 * math.cos(rad)
                dy_head = 0.2 * math.sin(rad)
                ax.arrow(robot_x, robot_y, dx_head, dy_head, 
                         head_width=0.04, head_length=0.06, fc='red', ec='red')
                
                # Distance and Required Angle Arrow (Target vector)
                dx_t = HOME_X - robot_x
                dy_t = HOME_Y - robot_y
                t_angle = heading_from_vector(dx_t, dy_t)
                turn_angle = normalize_rotation(t_angle - robot_heading)
                
                # Draw dashed arrow pointing precisely toward the destination
                ax.arrow(robot_x, robot_y, dx_t, dy_t, 
                         head_width=0.03, head_length=0.05, fc='green', ec='green', 
                         linestyle='--', length_includes_head=True, alpha=0.7, label='Target Path')
                
                dist = math.hypot(dx_t, dy_t)
                info_text = (f"=== TELEMETRY ===\n\n"
                             f"Pos X: {robot_x:.2f} m\n"
                             f"Pos Y: {robot_y:.2f} m\n"
                             f"Heading: {robot_heading:.1f}*\n\n"
                             f"--- ROUTE ---\n"
                             f"Dist: {dist:.2f} m\n"
                             f"Angle: {t_angle:.1f}*\n"
                             f"Turn: {turn_angle:+.1f}*")
                
                # Place info text box completely OUTSIDE the plot on the right
                fig.text(0.75, 0.5, info_text, transform=fig.transFigure,
                         fontsize=12, ha='left', va='center', family='monospace',
                         bbox=dict(facecolor='white', alpha=0.9, edgecolor='black', boxstyle='round,pad=1'))
            
            # Place the visual legend outside the plot area on the right, above the telemetry
            ax.legend(loc='upper left', bbox_to_anchor=(1.05, 1), borderaxespad=0.)
            
            canvas.draw()
            buf = canvas.buffer_rgba()
            map_img = np.asarray(buf)
            map_bgr = cv2.cvtColor(map_img, cv2.COLOR_RGBA2BGR)
            cv2.imshow("Arena Map", map_bgr)
            
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

def stop_robot():
    send_cmd('S', force=True)


def pulse_turn(direction, duration=0.05, speed='1'):
    send_cmd(speed, force=True)
    send_cmd(direction, force=True)
    time.sleep(duration)
    stop_robot()
    time.sleep(0.08)


def get_route_turn_duration(turn_error_deg):
    if abs(turn_error_deg) > ROUTE_COARSE_TURN_THRESHOLD_DEG:
        return ROUTE_COARSE_TURN_DURATION_S
    return ROUTE_FINE_TURN_DURATION_S


def get_turn_direction_from_error(turn_error_deg):
    return 'L' if turn_error_deg > 0 else 'R'


def drive_forward(speed='2'):
    send_cmd(speed)
    send_cmd('F')


def drive_backward(speed='1', duration=0.2):
    send_cmd(speed, force=True)
    send_cmd('B', force=True)
    time.sleep(duration)
    stop_robot()


def get_nearest_wall_distance_cm():
    valid_distances = [dist for dist in (latest_L, latest_C, latest_R) if 0 < dist < 300]
    if not valid_distances:
        return 999.0
    return min(valid_distances)


def should_back_up_for_tag_reacquire():
    return get_nearest_wall_distance_cm() < TAG_REACQUIRE_BACKUP_DISTANCE_CM


def get_ultrasonic_status():
    return f"L:{latest_L:.0f}cm C:{latest_C:.0f}cm R:{latest_R:.0f}cm"


def back_up_for_tag_reacquire():
    print(
        f"Too close to wall for reliable tag view ({get_ultrasonic_status()}, "
        f"nearest={get_nearest_wall_distance_cm():.0f}cm). "
        f"Backing up for {TAG_REACQUIRE_BACKUP_DURATION_S:.2f}s..."
    )
    drive_backward(duration=TAG_REACQUIRE_BACKUP_DURATION_S)
    time.sleep(0.1)


def get_average_home_tag_measurement(tags):
    visible_home_tags = [tag for tag in tags if tag.tag_id in HOME_TAGS]
    if not visible_home_tags:
        return None
    avg_x = sum(tag.pose_t[0][0] / SCALE for tag in visible_home_tags) / len(visible_home_tags)
    avg_z = sum(tag.pose_t[2][0] / SCALE for tag in visible_home_tags) / len(visible_home_tags)
    return avg_x, avg_z, [tag.tag_id for tag in visible_home_tags]


def handle_obstacle(last_known_side, allow_side_nudges=True):
    if latest_C < STOP_DIST:
        print(f"Blocked directly ahead ({latest_C:.0f}cm)! Avoiding...")
        stop_robot()
        time.sleep(0.15)
        drive_backward(duration=0.25)

        if latest_L > latest_R:
            print("Turning LEFT to avoid obstacle")
            pulse_turn('L', duration=0.28)
            return True, 1.0

        print("Turning RIGHT to avoid obstacle")
        pulse_turn('R', duration=0.28)
        return True, -1.0

    if not allow_side_nudges:
        return False, last_known_side

    if latest_L < SIDE_DIST:
        print("Too close to LEFT -> Nudging Right")
        pulse_turn('R', duration=0.08)
        return True, -1.0

    if latest_R < SIDE_DIST:
        print("Too close to RIGHT -> Nudging Left")
        pulse_turn('L', duration=0.08)
        return True, 1.0

    return False, last_known_side


# --- MAIN NAVIGATION LOOP ---
state = "HUNTING"

print("--- GO TO HOME SYSTEM ONLINE ---")

last_seen_time = time.time()
last_known_side = 1.0
frame_skip_counter = 0
committed_turn_direction = None
committed_turn_until = 0.0
preferred_turn_direction = None
preferred_turn_until = 0.0
last_tag_reacquire_backup_time = 0.0

send_cmd('1')

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
        draw_frame = frame.copy() if show_vision else None
        detected_tag_ids = []

        if tags:
            print("\n--- Frame Data ---")

        for tag in tags:
            detected_tag_ids.append(tag.tag_id)
            x_dist = tag.pose_t[0][0] / SCALE
            y_dist = tag.pose_t[1][0] / SCALE
            z_dist = tag.pose_t[2][0] / SCALE
            print(f"  > Tag {tag.tag_id}: X:{x_dist:.2f}m, Y:{y_dist:.2f}m, Z:{z_dist:.2f}m")

            if x_dist > 0:
                last_known_side = 1.0
            elif x_dist < 0:
                last_known_side = -1.0

            if show_vision:
                cv2.polylines(draw_frame, [tag.corners.astype(int)], True, (0, 255, 0), 2)
                cv2.putText(
                    draw_frame,
                    str(tag.tag_id),
                    (int(tag.center[0]), int(tag.center[1])),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 255),
                    2,
                )

        if show_vision:
            display_frame = draw_frame

        pose_result = estimate_robot_pose(tags)
        is_localized = pose_result is not None
        home_measurement = get_average_home_tag_measurement(tags)

        if is_localized:
            robot_x, robot_y, robot_heading, pose_estimates = pose_result
            last_seen_time = time.time()
            for tag_id, rx, ry, local_tangent, local_inward, _ in pose_estimates:
                print(
                    f"    Tag {tag_id}: local_t={local_tangent:.2f}, local_n={local_inward:.2f}, "
                    f"pos=({rx:.2f},{ry:.2f})"
                )

            distance_to_home = math.hypot(HOME_X - robot_x, HOME_Y - robot_y)
            desired_heading = heading_from_vector(HOME_X - robot_x, HOME_Y - robot_y)
            route_turn = normalize_rotation(desired_heading - robot_heading)
            final_turn = normalize_rotation(HOME_APPROACH_HEADING - robot_heading)

            print(f"Tags Detected: {detected_tag_ids}")
            print(f"Current Position -> X:{robot_x:.2f}m, Y:{robot_y:.2f}m, Pose:{robot_heading:.1f}deg")
            print(
                f"Route -> Dist:{distance_to_home:.2f}m | Desired:{desired_heading:.1f}deg | "
                f"Turn:{route_turn:+.1f}deg | Final:{HOME_APPROACH_HEADING:.1f}deg ({final_turn:+.1f})"
            )
        else:
            distance_to_home = None
            desired_heading = None
            route_turn = None
            final_turn = None
            print(f"Tags Detected: {detected_tag_ids} | Localized: False")

        search_mode = not is_localized

        if search_mode and should_back_up_for_tag_reacquire():
            now = time.time()
            if (now - last_tag_reacquire_backup_time) > TAG_REACQUIRE_BACKUP_COOLDOWN_S:
                back_up_for_tag_reacquire()
                last_tag_reacquire_backup_time = time.time()
                continue

        obstacle_detected, last_known_side = handle_obstacle(
            last_known_side,
            allow_side_nudges=not search_mode,
        )
        if obstacle_detected:
            continue

        if state == "HUNTING":
            if is_localized:
                stop_robot()
                committed_turn_direction = None
                committed_turn_until = 0.0
                preferred_turn_direction = None
                preferred_turn_until = 0.0
                if AUTO_START_ON_LOCALIZATION:
                    start_motion = True
                    state = "NAVIGATING"
                    print("\n>>> Localized. Auto-starting navigation toward home target <<<\n")
                else:
                    print("\n>>> PRESS 'S' IN VIDEO WINDOW OR ENTER IN TERMINAL TO START <<<")
                    state = "WAIT_FOR_START"
            elif (time.time() - last_seen_time) > SEARCH_TIMEOUT:
                if (
                    should_back_up_for_tag_reacquire()
                    and (time.time() - last_tag_reacquire_backup_time) > TAG_REACQUIRE_BACKUP_COOLDOWN_S
                ):
                    back_up_for_tag_reacquire()
                    last_tag_reacquire_backup_time = time.time()
                    continue
                print(
                    f"Searching for tags ({get_ultrasonic_status()}) -> "
                    f"turning {'RIGHT' if last_known_side > 0 else 'LEFT'}"
                )
                pulse_turn('R' if last_known_side > 0 else 'L', duration=0.08)
            time.sleep(0.01)
            continue

        if state == "WAIT_FOR_START":
            dr, dw, de = select.select([sys.stdin], [], [], 0.0)
            if dr:
                sys.stdin.readline()
                start_motion = True

            if start_motion:
                committed_turn_direction = None
                committed_turn_until = 0.0
                preferred_turn_direction = None
                preferred_turn_until = 0.0
                state = "NAVIGATING"
                print("\n>>> Navigating toward home target <<<\n")

            time.sleep(0.01)
            continue

        if state == "NAVIGATING":
            now = time.time()
            coarse_turn_committed = committed_turn_direction is not None and now < committed_turn_until
            preferred_turn_active = preferred_turn_direction is not None and now < preferred_turn_until

            if not is_localized:
                if (
                    should_back_up_for_tag_reacquire()
                    and (now - last_tag_reacquire_backup_time) > TAG_REACQUIRE_BACKUP_COOLDOWN_S
                ):
                    back_up_for_tag_reacquire()
                    last_tag_reacquire_backup_time = time.time()
                elif coarse_turn_committed:
                    print(
                        f"Localization intermittent ({get_ultrasonic_status()}). "
                        f"Continuing committed coarse turn {committed_turn_direction} "
                        f"for {committed_turn_until - now:.2f}s"
                    )
                    pulse_turn(committed_turn_direction, duration=ROUTE_COARSE_TURN_DURATION_S)
                elif preferred_turn_active:
                    print(
                        f"Localization lost ({get_ultrasonic_status()}). "
                        f"Holding navigation turn {preferred_turn_direction} "
                        f"for {preferred_turn_until - now:.2f}s"
                    )
                    pulse_turn(preferred_turn_direction, duration=ROUTE_COARSE_TURN_DURATION_S)
                elif (time.time() - last_seen_time) > SEARCH_TIMEOUT:
                    search_dir = 'R' if last_known_side > 0 else 'L'
                    print(f"Localization lost. Searching {search_dir} ({get_ultrasonic_status()})...")
                    pulse_turn(search_dir, duration=0.08)
                continue

            if distance_to_home <= FINAL_ALIGN_DISTANCE_M:
                stop_robot()
                committed_turn_direction = None
                committed_turn_until = 0.0
                preferred_turn_direction = None
                preferred_turn_until = 0.0
                state = "FINAL_ALIGN"
                print("Within final alignment distance. Switching to FINAL_ALIGN.")
                continue

            nav_heading, nav_turn, nav_tol, nav_mode = get_navigation_guidance(
                distance_to_home,
                desired_heading,
                HOME_APPROACH_HEADING,
                robot_heading,
            )

            if abs(nav_turn) > nav_tol:
                turn_duration = get_route_turn_duration(nav_turn)
                turn_direction = get_turn_direction_from_error(nav_turn)
                if abs(nav_turn) > ROUTE_COARSE_TURN_THRESHOLD_DEG:
                    if not coarse_turn_committed:
                        committed_turn_direction = turn_direction
                        committed_turn_until = now + ROUTE_COARSE_COMMIT_TIME_S
                    turn_direction = committed_turn_direction
                    turn_mode = "COARSE"
                else:
                    committed_turn_direction = None
                    committed_turn_until = 0.0
                    turn_mode = "FINE" if nav_mode == "ROUTE" else "SMOOTH"
                print(
                    f"{turn_mode} rotating {'LEFT' if turn_direction == 'L' else 'RIGHT'} toward "
                    f"{nav_heading:.1f}deg ({nav_turn:+.1f}deg, tol={nav_tol:.1f}deg, pulse={turn_duration:.2f}s)"
                )
                preferred_turn_direction = turn_direction
                preferred_turn_until = now + ROUTE_DIRECTION_HOLD_TIME_S
                pulse_turn(turn_direction, duration=turn_duration)
            else:
                committed_turn_direction = None
                committed_turn_until = 0.0
                preferred_turn_direction = None
                preferred_turn_until = 0.0
                if nav_mode == "SMOOTH_APPROACH":
                    print(
                        f"Smooth approach within tolerance ({nav_tol:.1f}deg). "
                        f"Driving forward slowly toward {HOME_APPROACH_HEADING:.1f}deg."
                    )
                    drive_forward(SMOOTH_APPROACH_SPEED)
                else:
                    print(f"Within route heading tolerance ({nav_tol:.1f}deg). Driving forward.")
                    drive_forward('3' if distance_to_home > 1.0 else '2')

            continue

        if state == "FINAL_ALIGN":
            committed_turn_direction = None
            committed_turn_until = 0.0
            preferred_turn_direction = None
            preferred_turn_until = 0.0
            if not is_localized:
                stop_robot()
                if (
                    should_back_up_for_tag_reacquire()
                    and (time.time() - last_tag_reacquire_backup_time) > TAG_REACQUIRE_BACKUP_COOLDOWN_S
                ):
                    back_up_for_tag_reacquire()
                    last_tag_reacquire_backup_time = time.time()
                elif (time.time() - last_seen_time) > SEARCH_TIMEOUT:
                    print(
                        f"Final alignment lost tag view ({get_ultrasonic_status()}) -> "
                        f"searching {'RIGHT' if last_known_side > 0 else 'LEFT'}"
                    )
                    pulse_turn('R' if last_known_side > 0 else 'L', duration=0.08)
                continue

            if abs(final_turn) > FINAL_HEADING_TOL_DEG:
                print(f"Final wall-normal alignment {'LEFT' if final_turn > 0 else 'RIGHT'} ({final_turn:+.1f}deg)")
                pulse_turn('L' if final_turn > 0 else 'R', duration=0.05)
                continue

            stop_robot()
            state = "FINAL_APPROACH"
            print("Final heading aligned. Switching to FINAL_APPROACH.")
            continue

        if state == "FINAL_APPROACH":
            committed_turn_direction = None
            committed_turn_until = 0.0
            preferred_turn_direction = None
            preferred_turn_until = 0.0
            if latest_C <= FINAL_STOP_DISTANCE_CM:
                stop_robot()
                print(f"HOME REACHED: stopping at proximity distance {latest_C:.0f}cm")
                state = "DONE"
                continue

            if home_measurement is not None:
                avg_home_x, avg_home_z, home_ids = home_measurement
                print(f"Final approach using home tags {home_ids}: X={avg_home_x:.2f}m, Z={avg_home_z:.2f}m")

                if avg_home_x > HOME_X_TOL_M:
                    pulse_turn('R', duration=0.04)
                elif avg_home_x < -HOME_X_TOL_M:
                    pulse_turn('L', duration=0.04)
                else:
                    drive_forward('1' if latest_C < (FINAL_STOP_DISTANCE_CM + 15) else '2')
                continue

            if is_localized:
                if abs(final_turn) > FINAL_HEADING_TOL_DEG:
                    pulse_turn('L' if final_turn > 0 else 'R', duration=0.05)
                else:
                    drive_forward('1')
            elif (
                should_back_up_for_tag_reacquire()
                and (time.time() - last_tag_reacquire_backup_time) > TAG_REACQUIRE_BACKUP_COOLDOWN_S
            ):
                back_up_for_tag_reacquire()
                last_tag_reacquire_backup_time = time.time()
            elif (time.time() - last_seen_time) > SEARCH_TIMEOUT:
                print(
                    f"Final approach lost tags ({get_ultrasonic_status()}) -> "
                    f"searching {'RIGHT' if last_known_side > 0 else 'LEFT'}"
                )
                pulse_turn('R' if last_known_side > 0 else 'L', duration=0.08)
            continue

        if state == "DONE":
            break

        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nEmergency Stop Triggered.")
finally:
    running = False
    stop_robot()
    time.sleep(0.1)
    picam2.stop()
    print("Clean exit.")