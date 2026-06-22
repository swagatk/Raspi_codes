import time
import threading
import sys
import math
import os
import shutil
import cv2
import logging
import numpy as np
from gpiozero import Button
from tqdm import tqdm
from robot_hardware import RobotController
from vision_module import VisionModule
import config
from config import *

LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "log")
if os.path.exists(LOG_DIR):
    shutil.rmtree(LOG_DIR, ignore_errors=True)
os.makedirs(LOG_DIR, exist_ok=True)

class TqdmLoggingHandler(logging.Handler):
    def emit(self, record):
        try:
            msg = self.format(record)
            tqdm.write(msg)
            self.flush()
        except Exception:
            self.handleError(record)

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    handlers=[
        logging.FileHandler(os.path.join(LOG_DIR, "motion_plan.log")),
        TqdmLoggingHandler()
    ]
)

button_pressed = False
mission_running = False
button = None
desired_exit_arm_pose = 'down'

def is_valid_distance_cm(dist):
    return SAFETY_MIN_VALID_CM <= dist <= SAFETY_MAX_VALID_CM

def can_continue():
    return mission_running

def wait_with_abort(duration_s, step_s=0.02):
    end_t = time.time() + max(0.0, duration_s)
    while can_continue() and time.time() < end_t:
        time.sleep(min(step_s, end_t - time.time()))
    return can_continue()

def execute_burst(direction, speed, run_s, settle_s=0.0):
    if not can_continue():
        return False
    robot.move(direction, speed)
    if not wait_with_abort(run_s):
        robot.halt()
        return False
    robot.halt()
    if settle_s > 0 and not wait_with_abort(settle_s):
        return False
    return True

def obstacle_detected(robot):
    center_blocked = is_valid_distance_cm(robot.latest_C) and robot.latest_C < STOP_DIST
    if not OBSTACLE_SIDE_CHECK_ENABLED:
        return center_blocked
    left_blocked = is_valid_distance_cm(robot.latest_L) and robot.latest_L < SIDE_DIST
    right_blocked = is_valid_distance_cm(robot.latest_R) and robot.latest_R < SIDE_DIST
    return center_blocked or left_blocked or right_blocked

def capture_obstacle_detected(robot):
    # During arm-down pick motion use center ultrasonic only, with same safe-wall threshold.
    return is_valid_distance_cm(robot.latest_C) and robot.latest_C < SAFE_WALL_DISTANCE

def compute_home_target_xy(arena_map):
    home_points = [arena_map[tag_id] for tag_id in HOME_TAG_IDS if tag_id in arena_map]
    if not home_points:
        return None
    mid_x = sum(point["x"] for point in home_points) / len(home_points)
    mid_y = sum(point["y"] for point in home_points) / len(home_points)
    _, inward_normal = get_wall_axes(HOME_TAG_IDS[0])
    approach_heading = heading_from_vector(-inward_normal[0], -inward_normal[1])
    return mid_x, mid_y, approach_heading, inward_normal

def heading_from_vector(vec_x, vec_y):
    return math.degrees(math.atan2(vec_y, vec_x)) % 360.0

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

def rotate_towards_heading(current_heading_deg, desired_heading_deg):
    heading_error = normalize_rotation(desired_heading_deg - current_heading_deg)
    if abs(heading_error) <= STEP5_HEADING_TOL_DEG:
        return True, heading_error, None

    turn_dir = 'L' if heading_error > 0 else 'R'
    turn_burst_s = (
        STEP5_COARSE_TURN_BURST_S
        if abs(heading_error) >= STEP5_COARSE_TURN_THRESHOLD_DEG
        else STEP5_FINE_TURN_BURST_S
    )
    moved = execute_burst(turn_dir, CENTERING_TURN_SPEED, turn_burst_s, CENTERING_SETTLE_S)
    if not moved:
        return None, heading_error, turn_dir
    return False, heading_error, turn_dir

def execute_course_step_forward():
    step_duration = STEP5_STEADY_FORWARD_BURST_S
    return execute_forward_with_sensor_guard(
        STEP5_STEADY_FORWARD_SPEED,
        step_duration,
        STEP5_STEADY_FORWARD_SETTLE_S,
        "course approach",
        check_line=False
    )


def execute_forward_with_sensor_guard(speed, run_s, settle_s=0.0, context_label="forward move", check_line=False):
    if not can_continue():
        return False

    robot.move('F', speed)
    blocked = False
    line_detected = False
    end_t = time.time() + max(0.0, run_s)
    while can_continue() and time.time() < end_t:
        # Continuously poll all ultrasonic sensors while moving forward.
        if obstacle_detected(robot):
            blocked = True
            break
        if check_line and (robot.line_L == 0 or robot.line_R == 0):
            line_detected = True
            break
        time.sleep(min(0.02, end_t - time.time()))

    robot.halt()
    if not can_continue():
        return False

    if blocked:
        logging.warning(
            f"{context_label}: obstacle detected during forward motion "
            f"(L:{robot.latest_L} C:{robot.latest_C} R:{robot.latest_R})."
        )
        return False

    if line_detected:
        logging.info(f"{context_label}: line sensor triggered (L:{robot.line_L} R:{robot.line_R}). Stopping forward motion.")
        return True

    if settle_s > 0 and not wait_with_abort(settle_s):
        return False
    return True

def run_obstacle_avoidance(robot):
    logging.warning(
        f"Obstacle detected (L:{robot.latest_L} C:{robot.latest_C} R:{robot.latest_R}) -> evasive maneuver"
    )
    robot.halt()
    if not execute_burst('B', OBSTACLE_BACKUP_SPEED, OBSTACLE_BACKUP_DURATION_S):
        return False
    if not execute_burst(OBSTACLE_TURN_DIR, OBSTACLE_TURN_SPEED, OBSTACLE_TURN_DURATION_S, OBSTACLE_SETTLE_S):
        return False
    return True

def wait_for_valid_ultrasonic(robot, timeout_s):
    start_t = time.time()
    while time.time() - start_t < timeout_s:
        valid = any(
            is_valid_distance_cm(v)
            for v in (robot.latest_L, robot.latest_C, robot.latest_R)
        )
        if valid:
            return True
        time.sleep(0.05)
    return False

def request_mission_stop(reason):
    global mission_running
    if not mission_running:
        return
    mission_running = False
    logging.warning(reason)
    if robot is not None:
        try:
            robot.halt()
        except Exception:
            pass

def check_button_thread():
    global button_pressed, mission_running, button

    if button is None:
        logging.warning("Physical start/stop button is not available; waiting for keyboard fallback.")
        return

    last_pressed = False
    while True:
        try:
            if button is None:
                break
            current_pressed = bool(button.is_pressed)
        except Exception:
            # GPIO objects can become unavailable during shutdown; end thread quietly.
            break
        if current_pressed and not last_pressed:
            if not mission_running:
                button_pressed = True
                mission_running = True
                logging.info("Start command received via physical button on GPIO %d.", BUTTON_PIN)
            else:
                request_mission_stop("Stop command received via physical button on GPIO %d." % BUTTON_PIN)
        last_pressed = current_pressed
        time.sleep(0.05)

def estimate_wall_distance_cm_from_picam(vision):
    # Use PiCamera AprilTag pose while arm is down to estimate wall distance.
    if getattr(vision, 'picam_frame', None) is None:
        return None

    try:
        from apriltag_detection import capture_home_tag_function, SCALE
        tags = capture_home_tag_function(vision.picam_frame, vision.picam_camera_params)
        if not tags:
            return None

        distances_cm = []
        for tag in tags:
            if hasattr(tag, 'pose_t') and tag.pose_t[2][0] > 0:
                distances_cm.append((tag.pose_t[2][0] / max(SCALE, 1e-6)) * 100.0)

        return min(distances_cm) if distances_cm else None
    except Exception as e:
        logging.warning(f"PiCamera AprilTag wall-distance estimation failed: {e}")
        return None

def save_annotated_frame(vision, filename):
    snapshot = None
    if hasattr(vision, 'get_ball_overlay_snapshot'):
        snapshot = vision.get_ball_overlay_snapshot()

    if snapshot and snapshot.get('frame') is not None:
        annotated = snapshot['frame']
        ball_candidates = snapshot.get('ball_candidates', [])
        target_box = snapshot.get('selected_ball') or snapshot.get('ball_box')
        ball_detected = snapshot.get('ball_detected', False)
        ball_distance = snapshot.get('ball_distance', 999.0)
    else:
        if hasattr(vision, 'active_frame') and vision.active_frame is not None:
            annotated = vision.active_frame.copy()
        elif vision.frame is not None:
            annotated = vision.frame.copy()
        else:
            return
        ball_candidates = getattr(vision, 'ball_candidates', [])
        target_box = getattr(vision, 'selected_ball', None) or getattr(vision, 'ball_box', None)
        ball_detected = vision.ball_detected
        ball_distance = vision.ball_distance

    # Draw all valid detections in blue, and selected target in cyan.
    for cand in ball_candidates:
        cx, cy, w, h, _ = cand
        x1 = int(cx - w / 2)
        y1 = int(cy - h / 2)
        x2 = int(cx + w / 2)
        y2 = int(cy + h / 2)
        cv2.rectangle(annotated, (x1, y1), (x2, y2), (255, 0, 0), 2)

    if ball_detected and target_box:
        x, y, w, h = target_box[:4]
        x1 = int(x - w / 2)
        y1 = int(y - h / 2)
        x2 = int(x + w / 2)
        y2 = int(y + h / 2)
        cv2.rectangle(annotated, (x1, y1), (x2, y2), (255, 255, 0), 3)
        cv2.putText(annotated, f"TARGET Dist: {ball_distance:.1f}cm | Area: {w*h:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)
    elif ball_detected:
        cv2.putText(annotated, f"Dist: {ball_distance:.1f}cm", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    
    try:
        full_path = os.path.join(LOG_DIR, filename)
        cv2.imwrite(full_path, annotated)
    except Exception as e:
        logging.error(f"Failed to save image {full_path}: {e}")

def save_home_tag_frame(vision, tag_id, filename):
    frame = getattr(vision, 'active_frame', None)
    if frame is None:
        frame = getattr(vision, 'frame', None)
    tags = getattr(vision, 'tags', [])
    save_home_tag_frame_from_source(frame, tags, tag_id, filename)

def save_home_tag_frame_from_source(frame, tags, tag_id, filename):
    if frame is None:
        return

    annotated = frame.copy()
    for tag in tags:
        if getattr(tag, 'tag_id', None) != tag_id:
            continue
        try:
            corners = tag.corners.astype(int)
            for i in range(4):
                cv2.line(annotated, tuple(corners[i]), tuple(corners[(i + 1) % 4]), (0, 255, 255), 2)
            center = tuple(tag.center.astype(int))
            cv2.circle(annotated, center, 4, (0, 0, 255), -1)
            cv2.putText(
                annotated,
                f"HOME TAG ID: {tag_id}",
                (max(5, corners[0][0]), max(25, corners[0][1] - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2,
            )
        except Exception:
            pass

    try:
        cv2.imwrite(os.path.join(LOG_DIR, filename), annotated)
    except Exception as e:
        logging.error(f"Failed to save image {filename}: {e}")

def get_pose_with_camera_fallback(vision, arena_map, estimate_robot_pose_func):
    # Try USB first, then PiCamera. Returns (pose, camera_name) where camera_name in {'usb','picam'}.
    usb_frame = getattr(vision, 'frame', None)
    if usb_frame is not None:
        pose = estimate_robot_pose_func(usb_frame, vision.usb_camera_params, arena_map)
        if pose:
            return pose, 'usb'

    picam_frame = getattr(vision, 'picam_frame', None)
    if picam_frame is not None:
        pose = estimate_robot_pose_func(picam_frame, vision.picam_camera_params, arena_map)
        if pose:
            return pose, 'picam'

    return None, None

def estimate_robot_pose_from_tags(tags, arena_map, pose_scale=1.0):
    if not tags:
        return None

    xmax = max((max(info["x"] for info in arena_map.values()), 2.0)) if arena_map else 2.0
    ymax = max((max(info["y"] for info in arena_map.values()), 2.0)) if arena_map else 2.0

    pose_estimates = []
    for tag in tags:
        if tag.tag_id not in arena_map:
            continue
        if tag.pose_t[2][0] <= 0:
            continue

        tag_info = arena_map[tag.tag_id]
        tangent_vec, inward_normal_vec = get_wall_axes(tag.tag_id)

        cam_pos_in_tag = -np.dot(tag.pose_R.T, tag.pose_t)
        local_tangent = -cam_pos_in_tag[0][0] / max(pose_scale, 1e-6)
        local_inward = -cam_pos_in_tag[2][0] / max(pose_scale, 1e-6)

        rx = tag_info["x"] + local_tangent * tangent_vec[0] + local_inward * inward_normal_vec[0]
        ry = tag_info["y"] + local_tangent * tangent_vec[1] + local_inward * inward_normal_vec[1]
        weight = 1.0 / max(tag.pose_t[2][0] / max(pose_scale, 1e-6), 0.05)

        pose_estimates.append((tag.tag_id, rx, ry, local_tangent, local_inward, weight))

    if not pose_estimates:
        return None

    best_tag = max(pose_estimates, key=lambda x: x[5])
    _, robot_x, robot_y, _, _, _ = best_tag

    robot_x = max(0.0, min(robot_x, xmax))
    robot_y = max(0.0, min(robot_y, ymax))

    heading_vec_x = 0.0
    heading_vec_y = 0.0

    for tag in tags:
        if tag.tag_id not in arena_map or tag.pose_t[2][0] <= 0:
            continue

        tag_info = arena_map[tag.tag_id]
        weight = 1.0 / max(tag.pose_t[2][0] / max(pose_scale, 1e-6), 0.05)
        vec_x = tag_info["x"] - robot_x
        vec_y = tag_info["y"] - robot_y

        angle_to_tag_world = math.degrees(math.atan2(vec_y, vec_x))
        angle_in_cam = math.degrees(math.atan2(tag.pose_t[0][0], tag.pose_t[2][0]))
        true_heading = angle_to_tag_world + angle_in_cam

        heading_vec_x += math.cos(math.radians(true_heading)) * weight
        heading_vec_y += math.sin(math.radians(true_heading)) * weight

    if abs(heading_vec_x) > 1e-6 or abs(heading_vec_y) > 1e-6:
        robot_heading = heading_from_vector(heading_vec_x, heading_vec_y)
    else:
        robot_heading = 0.0

    return robot_x, robot_y, robot_heading, pose_estimates

def get_pose_with_tag_data_fallback(vision, arena_map, preferred_camera=None):
    from apriltag_detection import capture_home_tag_function

    camera_order = ['usb', 'picam']
    if preferred_camera in camera_order:
        camera_order.remove(preferred_camera)
        camera_order.insert(0, preferred_camera)

    for camera_name in camera_order:
        if camera_name == 'usb':
            frame = getattr(vision, 'frame', None)
            camera_params = getattr(vision, 'usb_camera_params', None)
            pose_scale = USB_TAG_POSE_SCALE
        else:
            frame = getattr(vision, 'picam_frame', None)
            camera_params = getattr(vision, 'picam_camera_params', None)
            pose_scale = PICAM_TAG_POSE_SCALE

        if frame is None or camera_params is None:
            continue

        frame_to_use = frame.copy()

        tags = capture_home_tag_function(frame_to_use, camera_params)
        pose = estimate_robot_pose_from_tags(tags, arena_map, pose_scale=pose_scale)
        if pose:
            return pose, tags, camera_name, pose_scale, frame_to_use

    return None, [], None, None, None

def get_nearest_wall_distance_cm():
    valid_distances = [dist for dist in (robot.latest_L, robot.latest_C, robot.latest_R) if 0 < dist < 300]
    if not valid_distances:
        return 999.0
    return min(valid_distances)

def should_back_up_for_tag_reacquire():
    return get_nearest_wall_distance_cm() < STEP5_TAG_REACQUIRE_BACKUP_DISTANCE_CM

def get_ultrasonic_status():
    return f"L:{robot.latest_L:.0f}cm C:{robot.latest_C:.0f}cm R:{robot.latest_R:.0f}cm"

def back_up_for_tag_reacquire():
    logging.info(
        f"Too close to wall for tag reacquire ({get_ultrasonic_status()}, "
        f"nearest={get_nearest_wall_distance_cm():.0f}cm). "
        f"Backing up for {STEP5_TAG_REACQUIRE_BACKUP_DURATION_S:.2f}s."
    )
    execute_burst('B', STEP5_RELOCALIZE_BACKUP_SPEED, STEP5_TAG_REACQUIRE_BACKUP_DURATION_S, 0.1)

def get_average_home_tag_measurement(tags, pose_scale=1.0):
    visible_home_tags = [tag for tag in tags if tag.tag_id in HOME_TAG_IDS]
    if not visible_home_tags:
        return None

    avg_x = sum(tag.pose_t[0][0] / max(pose_scale, 1e-6) for tag in visible_home_tags) / len(visible_home_tags)
    avg_z = sum(tag.pose_t[2][0] / max(pose_scale, 1e-6) for tag in visible_home_tags) / len(visible_home_tags)
    return avg_x, avg_z, [tag.tag_id for tag in visible_home_tags]

def save_tag_detection_frame_from_source(frame, tags, filename):
    if frame is None:
        return

    annotated = frame.copy()
    for tag in tags:
        try:
            corners = tag.corners.astype(int)
            for i in range(4):
                cv2.line(annotated, tuple(corners[i]), tuple(corners[(i + 1) % 4]), (0, 255, 255), 2)
            center = tuple(tag.center.astype(int))
            cv2.circle(annotated, center, 4, (0, 0, 255), -1)
            cv2.putText(
                annotated,
                f"ID:{tag.tag_id}",
                (max(5, corners[0][0]), max(20, corners[0][1] - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2,
            )
        except Exception:
            continue

    try:
        cv2.imwrite(os.path.join(LOG_DIR, filename), annotated)
    except Exception as e:
        logging.warning(f"Failed to save Step 5 tag detection image {filename}: {e}")

def get_confirmed_pose_with_camera_fallback(vision, arena_map, estimate_robot_pose_func):
    # Confirm pose across consecutive frames to reject one-off jitter.
    confirmed_count = 0
    last_pose = None
    last_cam = None

    # Track how many attempts we've made to get a stable pose
    max_attempts = 12
    attempts = 0

    while mission_running and confirmed_count < STEP5_POSE_CONFIRM_FRAMES and attempts < max_attempts:
        attempts += 1
        pose, pose_cam = get_pose_with_camera_fallback(vision, arena_map, estimate_robot_pose_func)
        if not pose:
            time.sleep(STEP5_POSE_CONFIRM_INTERVAL_S)
            continue

        if last_pose is None:
            confirmed_count = 1
            last_pose = pose
            last_cam = pose_cam
        else:
            pos_jump_m = math.hypot(pose[0] - last_pose[0], pose[1] - last_pose[1])
            heading_jump_deg = abs(normalize_rotation(pose[2] - last_pose[2]))

            if (
                pose_cam == last_cam
                and pos_jump_m <= STEP5_POSE_CONFIRM_MAX_POS_JUMP_M
                and heading_jump_deg <= STEP5_POSE_CONFIRM_MAX_HEADING_JUMP_DEG
            ):
                confirmed_count += 1
            else:
                # Reset if the jump is too large
                confirmed_count = 1
            
            last_pose = pose
            last_cam = pose_cam

        if confirmed_count < STEP5_POSE_CONFIRM_FRAMES:
            time.sleep(STEP5_POSE_CONFIRM_INTERVAL_S)

    if confirmed_count >= STEP5_POSE_CONFIRM_FRAMES:
        return last_pose, last_cam
    return None, None

def get_home_tag_center_error_px(vision):
    # Prefer USB tags when available, otherwise PiCamera tags.
    usb_tags = getattr(vision, 'usb_tags', [])
    picam_tags = getattr(vision, 'picam_tags', [])

    for tag in usb_tags:
        if getattr(tag, 'tag_id', None) in HOME_TAG_IDS:
            return float(tag.center[0]) - CENTER_X

    for tag in picam_tags:
        if getattr(tag, 'tag_id', None) in HOME_TAG_IDS:
            picam_center_x = 320.0
            return float(tag.center[0]) - picam_center_x

    return None

def compute_heading_and_distance_to_home(rx, ry, heading, home_target_data):
    hx, hy, approach_heading, inward_normal = home_target_data
    dx = hx - rx
    dy = hy - ry
    linear_distance_m = math.hypot(dx, dy)

    dist_along_normal = -(dx * inward_normal[0] + dy * inward_normal[1])
    cross_track_x = rx - (hx + inward_normal[0] * dist_along_normal)
    cross_track_y = ry - (hy + inward_normal[1] * dist_along_normal)
    cross_track_error = math.hypot(cross_track_x, cross_track_y)

    if linear_distance_m <= (ALIGN_DIST_TO_HOME / 100.0):
        # We are close to the wall, so orient to be normal to it
        desired_heading = approach_heading
    else:
        # We are far, just head in a straight line towards the target
        desired_heading = heading_from_vector(dx, dy)

    rotation_deg = normalize_rotation(desired_heading - heading)
    return desired_heading, rotation_deg, linear_distance_m, cross_track_error

def global_progress_thread():
    total_dur = int(
        STEP2_CONFIRM_HOME_TIMEOUT_S
        + STEP4_SEARCH_AND_APPROACH_TIMEOUT_S
        + STEP5_RETURN_HOME_TIMEOUT_S
    )
    with tqdm(total=total_dur, desc="Mission Progress", position=0, leave=True, bar_format="{l_bar}{bar}| {n_fmt}/{total_fmt}s") as pbar:
        while mission_running and pbar.n < total_dur:
            time.sleep(1)
            pbar.update(1)

def wait_for_exit_button_press(lower_arm_before_exit=False):
    # check_button_thread toggles mission_running=False on a press while mission is active.
    if button is None:
        logging.warning("Exit wait requested, but button is unavailable. Exiting immediately.")
        return

    logging.info("Waiting for physical button press to exit program...")
    while mission_running:
        time.sleep(0.1)
    logging.info("Exit button press received.")

    if lower_arm_before_exit and robot is not None:
        logging.info("Lowering ARM to DOWN pose before exiting.")
        try:
            robot.arm_down()
            if 'vision' in globals() and vision:
                vision.update_active_camera(False)
            time.sleep(1.0)
        except Exception:
            pass

def main():
    global robot, vision, desired_exit_arm_pose
    logging.info("--- Step 1: Initialization phase ---")
    robot = RobotController()
    vision = VisionModule()
    
    if not robot.ser:
        logging.error("Motor/Arduino initialization failed (Check serial connection).")
        sys.exit(1)
    else:
        logging.info("Motor status: OK")
        
    robot.start_sensors()
    vision.start()
    
    # Check individual camera statuses
    time.sleep(INIT_CAMERA_WARMUP_S)
    usb_ok = getattr(vision, 'frame', None) is not None
    picam_ok = getattr(vision, 'picam_frame', None) is not None
    
    if usb_ok:
        logging.info("USB Camera status: OK")
    else:
        logging.error("USB Camera status: FAILED to capture frames.")
        
    if picam_ok:
        logging.info("PiCamera status: OK")
    else:
        logging.error("PiCamera status: FAILED to capture frames.")
        
    if not usb_ok or not picam_ok:
        logging.error("One or more cameras failed to initialize. Exiting as per mission requirement.")
        sys.exit(1)
        
    if not wait_for_valid_ultrasonic(robot, INIT_SENSOR_TIMEOUT_S):
        logging.error("Ultrasonic sensor status: FAILED to receive valid readings in time.")
        sys.exit(1)
    else:
        logging.info(f"Ultrasonic sensor status: OK (L:{robot.latest_L} C:{robot.latest_C} R:{robot.latest_R})")

    global button

    try:
        button = Button(BUTTON_PIN, bounce_time=BUTTON_DEBOUNCE_S)
        logging.info("Physical start/stop button ready on GPIO %d.", BUTTON_PIN)
    except Exception as exc:
        logging.error("Failed to initialize physical start/stop button on GPIO %d: %s", BUTTON_PIN, exc)
        return

    logging.info("Button status: Listener running (press the physical button on GPIO %d to start/stop).", BUTTON_PIN)

    logging.info("Assuming ARM starts in DOWN pose. Moving ARM to UP pose.")
    robot.arm_up()
    if 'vision' in globals() and vision:
        vision.update_active_camera(True)
    time.sleep(1.0)
    
    threading.Thread(target=check_button_thread, daemon=True).start()
    logging.info("System Online. Waiting for the physical button press on GPIO %d to start the mission sequence...", BUTTON_PIN)
    
    while not button_pressed:
        time.sleep(0.1)
        
    logging.info("Button pressed. Mission Started!")
    if not mission_running: return
    
    threading.Thread(target=global_progress_thread, daemon=True).start()
    
    # ---------------------------------------------------------
    # STEP 2: Confirm Home Location (~10 seconds)
    # ---------------------------------------------------------
    logging.info("--- Step 2: Confirm Home Location ---")
    start_time = time.time()
    home_confirmed = False
    saved_home_tag_ids = set()
    
    while mission_running and (time.time() - start_time < STEP2_CONFIRM_HOME_TIMEOUT_S):
        time.sleep(0.2)
        usb_tags = list(getattr(vision, 'usb_tags', []))
        picam_tags = list(getattr(vision, 'picam_tags', []))
        usb_ids = [t.tag_id for t in usb_tags]
        picam_ids = [t.tag_id for t in picam_tags]
        usb_matched = [t for t in config.HOME_TAG_IDS if t in usb_ids]
        picam_matched = [t for t in config.HOME_TAG_IDS if t in picam_ids]

        source_name = None
        source_tags = []
        source_frame = None
        matched = []

        if usb_matched:
            source_name = 'USB'
            source_tags = usb_tags
            source_frame = getattr(vision, 'frame', None)
            matched = usb_matched
        elif picam_matched:
            source_name = 'PiCamera'
            source_tags = picam_tags
            source_frame = getattr(vision, 'picam_frame', None)
            matched = picam_matched
            logging.info("USB did not detect configured home tags. Using PiCamera fallback for Step 2.")
        elif usb_ids or picam_ids:
            detected = sorted(set(usb_ids + picam_ids))
            logging.error(
                f"Tags detected do not match configured home tags. Detected={detected}, "
                f"Expected any of {config.HOME_TAG_IDS}. Exiting mission."
            )
            sys.exit(1)

        if matched:
            logging.info(f"Home tag(s) {matched} confirmed via {source_name} camera.")
            for idx, tag_id in enumerate(HOME_TAG_IDS, start=1):
                if tag_id in matched and tag_id not in saved_home_tag_ids:
                    save_home_tag_frame_from_source(source_frame, source_tags, tag_id, f"home_tag_{idx}.jpg")
                    saved_home_tag_ids.add(tag_id)
            home_confirmed = True
            break
        else:
            logging.info("Home tags not visible. Adjusting view backwards...")
            execute_burst('B', STEP2_BACKUP_SPEED, STEP2_BACKUP_DURATION_S, STEP2_BACKUP_SETTLE_S)
            
    if not home_confirmed:
        logging.error("Home tags not detected within time limit. Exiting mission.")
        sys.exit(1)
        
    step2_elapsed_s = time.time() - start_time
    step2_saved_s = max(0.0, STEP2_CONFIRM_HOME_TIMEOUT_S - step2_elapsed_s)
    logging.info("Step 2 completed successfully.")
    if not mission_running: return

    # ---------------------------------------------------------
    # STEP 3: Ready to Explore
    # ---------------------------------------------------------
    logging.info("--- Step 3: Ready to Explore ---")
    step3_start_s = time.time()
    execute_burst(STEP3_TURN_DIR, STEP3_TURN_SPEED, STEP3_TURN_DURATION_S, STEP3_POST_TURN_SETTLE_S)
    step3_budget_s = STEP3_READY_TO_EXPLORE_S + step2_saved_s
    step3_elapsed_s = time.time() - step3_start_s
    step3_saved_s = max(0.0, step3_budget_s - step3_elapsed_s)
    logging.info("Completion of Step 3")

    # ---------------------------------------------------------
    # STEP 4: Search and Approach balls (~ 2 minutes)
    # ---------------------------------------------------------
    logging.info("--- Step 4: Search and Approach balls ---")
    exploration_start = time.time()
    step4_budget_s = STEP4_SEARCH_AND_APPROACH_TIMEOUT_S + step3_saved_s
    
    while mission_running and (time.time() - exploration_start < step4_budget_s):
        
        # 1. Search for target
        logging.info("Searching for ball...")
        ball_found = False
        search_start = time.time()
        
        while mission_running and (time.time() - search_start < BALL_SEARCH_WINDOW_S) and (time.time() - exploration_start < step4_budget_s):
            if vision.ball_detected:
                robot.halt()
                logging.info(
                    f"Target detected (dist={vision.ball_distance:.1f}cm, x={vision.ball_x:.1f}). "
                    "Switching immediately to approach."
                )
                save_annotated_frame(vision, f"ball_target_{time.strftime('%Y%m%d_%H%M%S')}.jpg")
                ball_found = True
                break
            # Rotate by a slightly larger amount to search faster
            if not execute_burst(SEARCH_TURN_DIR, SEARCH_TURN_SPEED, SEARCH_TURN_DURATION_S, SEARCH_TURN_SETTLE_S):
                break
            
        if not ball_found:
            # Continue tracking elapsed time
            continue
            
        # 2. Approach target
        logging.info("Approaching the ball...")
        reached = False
        approach_start = time.time()
        last_seen_ts = time.time()
        last_seen_error = 0.0
        last_seen_distance = 999.0
        
        while mission_running and (time.time() - approach_start < BALL_APPROACH_WINDOW_S) and (time.time() - exploration_start < step4_budget_s):
            # Obstacle avoidance priority
            if obstacle_detected(robot):
                if not run_obstacle_avoidance(robot):
                    break
                continue
                
            if vision.ball_detected:
                error = vision.ball_x - CENTER_X
                last_seen_error = error
                last_seen_ts = time.time()
                last_seen_distance = vision.ball_distance

                # Use a small hysteresis band so tiny detector jitter does not flip the turn direction.
                if abs(error) > CENTER_TOLERANCE + 8:
                    turn_dir = 'L' if error < 0 else 'R'
                    logging.debug(f"Centering ball: error={error:+.1f}px -> turn {turn_dir}")
                    if not execute_burst(turn_dir, CENTERING_TURN_SPEED, CENTERING_TURN_BURST_S, CENTERING_SETTLE_S):
                        break
                    continue
                else:
                    if vision.ball_distance <= BALL_PICK_DISTANCE:
                        logging.info(f"Target reached BALL_PICK_DISTANCE ({BALL_PICK_DISTANCE:.1f}cm).")
                        robot.halt()
                        reached = True
                        break
                    else:
                        # Move slowly and trim heading in small bursts so target stays centered.
                        if abs(error) > APPROACH_TRIM_TOLERANCE_PX:
                            trim_dir = 'L' if error < 0 else 'R'
                            if not execute_burst(
                                trim_dir,
                                CENTERING_TURN_SPEED,
                                APPROACH_TRIM_TURN_BURST_S,
                                CENTERING_SETTLE_S,
                            ):
                                break
                        if not execute_burst('F', APPROACH_FORWARD_SPEED, APPROACH_FORWARD_BURST_S, APPROACH_FORWARD_SETTLE_S):
                            break
            else:
                # Keep target lock for short dropouts instead of immediately rescanning.
                if (time.time() - last_seen_ts) <= TARGET_LOSS_GRACE_S:
                    if last_seen_distance <= (BALL_PICK_DISTANCE + BALL_PICK_LOSS_MARGIN_CM):
                        logging.info(
                            f"Target briefly lost near pickup threshold ({last_seen_distance:.1f}cm). Proceeding to grab."
                        )
                        robot.halt()
                        reached = True
                        break
                    reacquire_dir = 'L' if last_seen_error < 0 else 'R'
                    if not execute_burst(
                        reacquire_dir,
                        TARGET_REACQUIRE_TURN_SPEED,
                        TARGET_REACQUIRE_TURN_BURST_S,
                        TARGET_REACQUIRE_SETTLE_S,
                    ):
                        break
                    continue

                logging.warning("Target lost beyond grace interval. Trying local reacquire before search reset.")
                reacquire_start = time.time()
                reacquired = False
                while mission_running and (time.time() - reacquire_start) <= TARGET_REACQUIRE_MAX_S:
                    if vision.ball_detected:
                        last_seen_ts = time.time()
                        last_seen_error = vision.ball_x - CENTER_X
                        last_seen_distance = vision.ball_distance
                        reacquired = True
                        break
                    reacquire_dir = 'L' if last_seen_error < 0 else 'R'
                    if not execute_burst(
                        reacquire_dir,
                        TARGET_REACQUIRE_TURN_SPEED,
                        TARGET_REACQUIRE_TURN_BURST_S,
                        TARGET_REACQUIRE_SETTLE_S,
                    ):
                        break

                if reacquired:
                    continue

                logging.warning("Target lost beyond local reacquire window. Returning to search.")
                robot.halt()
                break # break approach loop, re-enter search
                
        # 3. Grab the ball
        if reached:
            blocked_channels = []
            for ch_name, ch_dist in (
                ("L", robot.latest_L),
                ("C", robot.latest_C),
                ("R", robot.latest_R),
            ):
                if is_valid_distance_cm(ch_dist) and ch_dist < STEP4_ARM_LOWER_MIN_CLEARANCE_CM:
                    blocked_channels.append(f"{ch_name}:{ch_dist:.1f}cm")

            if blocked_channels:
                logging.warning(
                    f"Too close to obstacle for arm-lower safety (< {STEP4_ARM_LOWER_MIN_CLEARANCE_CM:.1f}cm) "
                    f"[{', '.join(blocked_channels)}]. Skipping arm down and backing up."
                )
                robot.halt()
                execute_burst(
                    'B',
                    OBSTACLE_BACKUP_SPEED,
                    OBSTACLE_BACKUP_DURATION_S,
                    OBSTACLE_SETTLE_S,
                )
                reached = False
                continue

            logging.info("Grabbing the ball...")
            robot.arm_down()
            if 'vision' in globals() and vision:
                vision.update_active_camera(False)
            wait_with_abort(2.0)

            wall_dist_cm = estimate_wall_distance_cm_from_picam(vision)
            if wall_dist_cm is None:
                c_reading = robot.latest_C
                if is_valid_distance_cm(c_reading) and c_reading < SAFE_WALL_DISTANCE:
                    logging.warning(
                        f"No AprilTag wall-distance available from PiCamera during arm-down capture. "
                        f"Falling back to C sensor: C={c_reading:.1f}cm < {SAFE_WALL_DISTANCE:.1f}cm. "
                        "Aborting this capture attempt."
                    )
                    robot.arm_up()
                    if 'vision' in globals() and vision:
                        vision.update_active_camera(True)
                    time.sleep(1.0)
                    continue
                logging.warning(
                    f"No AprilTag wall-distance available from PiCamera during arm-down capture. "
                    f"Falling back to C sensor: C={c_reading:.1f}cm."
                )
            else:
                logging.info(f"PiCamera wall distance during arm-down capture: {wall_dist_cm:.1f}cm")
                if wall_dist_cm < SAFE_WALL_DISTANCE:
                    logging.warning(
                        f"Wall too close ({wall_dist_cm:.1f}cm < {SAFE_WALL_DISTANCE:.1f}cm). Aborting this capture attempt."
                    )
                    robot.arm_up()
                    if 'vision' in globals() and vision:
                        vision.update_active_camera(True)
                    time.sleep(1.0)
                    continue

            robot.gripper_open()
            wait_with_abort(1.0)
            
            logging.info(
                f"Moving forward for {config.SLOW_FORWARD_PICK_MOTION_S:.1f}s while opening/closing gripper "
                f"{config.MAX_OPEN_CLOSE_TIMES} times..."
            )
            robot.move('F', CAPTURE_FORWARD_SPEED)

            time_to_move = config.SLOW_FORWARD_PICK_MOTION_S
            total_toggles = max(0, int(config.MAX_OPEN_CLOSE_TIMES) * 2)
            toggle_interval = (time_to_move / total_toggles) if total_toggles > 0 else (time_to_move + 1.0)
                
            start_m = time.time()
            last_toggle = start_m
            gripper_is_open = True
            toggles_done = 0
            forward_blocked = False
            last_wall_check_ts = 0.0
            
            while (time.time() - start_m) < time_to_move:
                current_time = time.time()
                if not mission_running:
                    robot.halt()
                    break
                if not forward_blocked and capture_obstacle_detected(robot):
                    logging.warning(
                        f"Obstacle detected during capture move (C:{robot.latest_C}cm). Backing up and continuing gripper cycles."
                    )
                    robot.halt()
                    execute_burst('B', CAPTURE_BACKUP_SPEED, CAPTURE_BACKUP_DURATION_S)
                    forward_blocked = True

                # Re-check PiCamera wall distance while arm is down and moving/picking.
                if (current_time - last_wall_check_ts) >= 0.2:
                    live_wall_dist_cm = estimate_wall_distance_cm_from_picam(vision)
                    last_wall_check_ts = current_time
                    if live_wall_dist_cm is not None and live_wall_dist_cm < SAFE_WALL_DISTANCE:
                        logging.warning(
                            f"Wall too close during capture move ({live_wall_dist_cm:.1f}cm < {SAFE_WALL_DISTANCE:.1f}cm). Backing up."
                        )
                        robot.halt()
                        execute_burst('B', CAPTURE_BACKUP_SPEED, CAPTURE_BACKUP_DURATION_S)
                        forward_blocked = True

                if total_toggles > 0 and toggles_done < total_toggles and (current_time - last_toggle) >= toggle_interval:
                    if gripper_is_open:
                        robot.gripper_close()
                        gripper_is_open = False
                    else:
                        robot.gripper_open()
                        gripper_is_open = True
                    toggles_done += 1
                    last_toggle = current_time
                time.sleep(0.02)

            # If the loop ended early or missed schedule slots, finish pending toggles before final close.
            while mission_running and toggles_done < total_toggles:
                if gripper_is_open:
                    robot.gripper_close()
                    gripper_is_open = False
                else:
                    robot.gripper_open()
                    gripper_is_open = True
                toggles_done += 1
                if not wait_with_abort(0.10):
                    break
                
            # Final close one last time before lifting arm.
            robot.gripper_close()
            wait_with_abort(0.5)
            robot.halt()
            wait_with_abort(0.5)
            
            robot.arm_up()
            if 'vision' in globals() and vision:
                vision.update_active_camera(True)
            wait_with_abort(2)
            logging.info("Ball grab complete.")
            
    logging.info("Completion of Step 4")
    if not mission_running: return

    # ---------------------------------------------------------
    # STEP 5: Returning home (~40 seconds)
    # ---------------------------------------------------------
    logging.info("--- Step 5: Returning home ---")
    home_start = time.time()
    last_localize_save_ts = 0.0
    near_home_confirm = 0
    failed_localize_cycles = 0
    
    from map import ARENA_MAP
    from map_visualizer import generate_localization_image
    home_target_data = compute_home_target_xy(ARENA_MAP)
    if home_target_data is None:
        logging.error("Home target cannot be computed from map/home tags. Exiting mission.")
        return
    
    logging.info("Moving to Home tag")

    active_nav_camera = None
    pose_lock_acquired = False
    last_align_turn_dir = 'L'

    arrived_at_home = False
    blind_approach_active = False
    last_valid_dist = None
    unrealistic_jump_count = 0

    while mission_running and (time.time() - home_start < STEP5_RETURN_HOME_TIMEOUT_S):
        if not blind_approach_active:
            pose_result, detected_tags, pose_cam, pose_scale, source_frame = get_pose_with_tag_data_fallback(
                vision,
                ARENA_MAP,
                preferred_camera=active_nav_camera,
            )
            if pose_result is None:
                failed_localize_cycles += 1
                if not pose_lock_acquired:
                    logging.info(
                        f"Step 5: pose not found (attempt {failed_localize_cycles}). "
                        f"Backing up for {STEP5_RELOCALIZE_BACKUP_DURATION_S:.2f}s and retrying."
                    )
                    execute_burst(
                        'B',
                        STEP5_RELOCALIZE_BACKUP_SPEED,
                        STEP5_RELOCALIZE_BACKUP_DURATION_S,
                        STEP5_RELOCALIZE_BACKUP_SETTLE_S,
                    )
                else:
                    logging.info(
                        f"Step 5: pose lost after lock (attempt {failed_localize_cycles}). "
                        f"Turning {last_align_turn_dir} briefly to reacquire (no backward move)."
                    )
                    execute_burst(
                        last_align_turn_dir,
                        STEP5_SEARCH_TURN_SPEED,
                        STEP5_SEARCH_TURN_BURST_S,
                        0.05,
                    )
                continue

            failed_localize_cycles = 0
            pose_lock_acquired = True
            active_nav_camera = pose_cam
            rx, ry, heading, _ = pose_result
            desired_heading, route_turn, linear_distance_m, cross_track_error = compute_heading_and_distance_to_home(
                rx, ry, heading, home_target_data
            )
            dist_to_home_cm = linear_distance_m * 100.0

            if last_valid_dist is not None and abs(dist_to_home_cm - last_valid_dist) > 30.0:
                unrealistic_jump_count += 1
                logging.warning(f"Unrealistic pose jump detected ({unrealistic_jump_count}/3): {last_valid_dist:.1f}cm -> {dist_to_home_cm:.1f}cm. Discarding pose.")
                if unrealistic_jump_count >= 3:
                    if last_valid_dist <= 80.0:
                        logging.warning("Persistent jump while close to home. Forcing blind approach to break deadlock.")
                        blind_approach_active = True
                        dist_to_home_cm = last_valid_dist
                    else:
                        logging.warning("Persistent jump. Accepting new pose to break deadlock.")
                        last_valid_dist = dist_to_home_cm
                        unrealistic_jump_count = 0
                else:
                    continue
            else:
                unrealistic_jump_count = 0
                last_valid_dist = dist_to_home_cm

            home_measurement = get_average_home_tag_measurement(detected_tags, pose_scale=pose_scale)

            if dist_to_home_cm <= 45.0:
                logging.info("Entering blind approach mode (dist <= 45cm). Ignoring camera, using ultrasonic sensors.")
                blind_approach_active = True

            now = time.time()
            if (now - last_localize_save_ts) >= STEP5_LOCALIZE_IMAGE_INTERVAL_S:
                ts = time.strftime('%Y%m%d_%H%M%S')
                save_path = os.path.join(LOG_DIR, f"localize_{ts}.jpg")
                threading.Thread(
                    target=generate_localization_image,
                    args=(rx, ry, heading, ARENA_MAP, config.HOME_TAG_IDS, save_path),
                    daemon=True
                ).start()
                threading.Thread(
                    target=save_tag_detection_frame_from_source,
                    args=(source_frame, detected_tags, f"tag_detection_{ts}.png"),
                    daemon=True
                ).start()
                last_localize_save_ts = now

            logging.info(
                f"Robot Pose: x={rx:.2f}, y={ry:.2f}, heading={heading:.1f} (camera={pose_cam}), dist_to_home={dist_to_home_cm:.1f}cm"
            )
            logging.info(
                f"Route: desired={desired_heading:.1f}deg, turn={route_turn:+.1f}deg"
            )

        valid_wall_dists = [
            d for d in (robot.latest_L, robot.latest_C, robot.latest_R)
            if is_valid_distance_cm(d)
        ]
        if valid_wall_dists and min(valid_wall_dists) <= STOP_DIST:
            near_home_confirm += 1
            logging.info(
                f"Wall stop check {near_home_confirm}/{HOME_WALL_STOP_CONFIRM_COUNT}: "
                f"L={robot.latest_L:.1f} C={robot.latest_C:.1f} R={robot.latest_R:.1f}, "
                f"threshold={STOP_DIST:.1f}cm"
            )
            if near_home_confirm >= HOME_WALL_STOP_CONFIRM_COUNT:
                robot.halt()
                arrived_at_home = True
                break
        else:
            near_home_confirm = 0

        if blind_approach_active or dist_to_home_cm <= ALIGN_DIST_TO_HOME:
            logging.info(f"Line sensors: L={robot.line_L} R={robot.line_R}")
            if robot.line_L == 0 and robot.line_R == 0:
                logging.info("Home boundary detected")
                robot.halt()
                arrived_at_home = True
                break
            elif robot.line_L == 0 or robot.line_R == 0:
                turn_dir = 'L' if robot.line_L == 0 else 'R'
                logging.info(f"Step 5 align: Line aligning, turning {turn_dir} (L:{robot.line_L} R:{robot.line_R})")
                execute_burst(turn_dir, STEP5_FINE_TURN_SPEED, 0.04, CENTERING_SETTLE_S)
                continue

        if not blind_approach_active and obstacle_detected(robot):
            logging.warning(
                f"Obstacle ahead during Step 5 (L:{robot.latest_L:.1f} C:{robot.latest_C:.1f} R:{robot.latest_R:.1f}) -> avoid"
            )
            run_obstacle_avoidance(robot)
            robot.halt()
            continue

        if not blind_approach_active and abs(route_turn) > STEP5_HEADING_TOL_DEG:
            turn_direction = 'L' if route_turn > 0 else 'R'
            last_align_turn_dir = turn_direction
            turn_speed = STEP5_COARSE_TURN_SPEED if abs(route_turn) > STEP5_COARSE_TURN_THRESHOLD_DEG else STEP5_FINE_TURN_SPEED
            logging.info(
                f"Step 5 align: turning {turn_direction} (error={route_turn:+.1f}deg, speed={turn_speed})"
            )
            execute_burst(
                turn_direction,
                turn_speed,
                STEP5_ALIGN_TURN_BURST_S,
                CENTERING_SETTLE_S,
            )
            continue

        if not blind_approach_active and dist_to_home_cm > ALIGN_DIST_TO_HOME:
            logging.info(
                f"Step 5 far approach (> {ALIGN_DIST_TO_HOME:.0f}cm): large forward step"
            )
            moved = execute_forward_with_sensor_guard(
                COURSE_FORWARD_SPEED,
                STEP5_FAR_FORWARD_BURST_S,
                STEP5_STEADY_FORWARD_SETTLE_S,
                "Step 5 far forward",
                check_line=False
            )
        else:
            if blind_approach_active:
                logging.info("Step 5 blind approach (<= 45cm): blind forward step")
            else:
                logging.info(f"Step 5 near approach (<= {ALIGN_DIST_TO_HOME:.0f}cm): fine forward step")
            moved = execute_forward_with_sensor_guard(
                STEP5_STEADY_FORWARD_SPEED,
                STEP5_FINE_TUNE_FORWARD_BURST_S,
                STEP5_STEADY_FORWARD_SETTLE_S,
                "Step 5 fine/blind forward",
                check_line=True
            )

        if not moved:
            if not blind_approach_active and obstacle_detected(robot):
                logging.warning(
                    f"Obstacle during Step 5 forward step (L:{robot.latest_L:.1f} C:{robot.latest_C:.1f} R:{robot.latest_R:.1f}) -> avoid"
                )
                run_obstacle_avoidance(robot)
                robot.halt()
            if not mission_running:
                break
            continue

    if not mission_running:
        logging.info("Mission stopped before drop sequence.")
        return

    drop_executed = False
    final_home_tag_distance_cm = None
    if arrived_at_home:
        _, final_tags, final_cam, final_pose_scale, _ = get_pose_with_tag_data_fallback(
            vision,
            ARENA_MAP,
            preferred_camera=active_nav_camera,
        )
        final_home_measurement = get_average_home_tag_measurement(final_tags, pose_scale=final_pose_scale if final_pose_scale is not None else 1.0)
        if final_home_measurement is not None:
            final_home_tag_distance_cm = final_home_measurement[1] * 100.0
            logging.info(
                f"Final home-tag distance from {final_cam} camera: {final_home_tag_distance_cm:.1f}cm"
            )
        else:
            logging.warning("Could not get final home-tag distance for drop check.")

    if arrived_at_home:
        if final_home_tag_distance_cm is None or final_home_tag_distance_cm <= (HOME_WALL_STOP_DISTANCE + 5.0):
            logging.info("Executing Ball drop...")
            robot.arm_drop()
            wait_with_abort(2)
            robot.gripper_open()
            wait_with_abort(1)
            drop_executed = True
        else:
            logging.warning(
                f"At home proximity, but home-tag distance ({final_home_tag_distance_cm:.1f}cm > {HOME_WALL_STOP_DISTANCE + 5.0:.1f}cm). "
                "Skipping drop; keeping arm UP and gripper closed."
            )
    else:
        logging.error("Failed to reach home within time limit. Skipping drop sequence and keeping arm UP.")

    if drop_executed:
        logging.info("Post-drop: turning around by 180 degrees, then moving arm to DOWN pose.")
        execute_burst(
            STEP5_POST_DROP_TURN_DIR,
            STEP5_POST_DROP_TURN_SPEED,
            STEP5_POST_DROP_TURN_DURATION_S,
            STEP5_POST_DROP_TURN_SETTLE_S,
        )
        robot.arm_down()
        desired_exit_arm_pose = 'down'
        if 'vision' in globals() and vision:
            vision.update_active_camera(False)
        robot.halt()
        wait_for_exit_button_press(lower_arm_before_exit=True)
        desired_exit_arm_pose = 'down'
    elif not arrived_at_home:
        # Step-5 timeout before drop: wait in current pose until button press, then lower arm and exit.
        robot.halt()
        wait_for_exit_button_press(lower_arm_before_exit=True)
        desired_exit_arm_pose = 'down'
    else:
        robot.gripper_close()
        robot.arm_up()
        if 'vision' in globals() and vision:
            vision.update_active_camera(True)
        robot.halt()
        logging.info("Waiting for button press before exit after skipped drop.")
        wait_for_exit_button_press(lower_arm_before_exit=True)
        desired_exit_arm_pose = 'down'

    robot.halt()
    logging.info("Mission sequence finalized. Shutting down with configured final arm pose.")

global robot, vision
robot = None
vision = None

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        logging.info("Interrupted by user (Ctrl+C)")
    except Exception as e:
        logging.error(f"Mission crashed due to exception: {e}")
    finally:
        if robot is not None:
            final_pose = desired_exit_arm_pose if desired_exit_arm_pose in ('up', 'down', 'keep') else 'down'
            if final_pose == 'keep':
                logging.info("Keeping current ARM pose before exit.")
            else:
                logging.info("Moving ARM to %s pose before exit.", final_pose.upper())
            try:
                if final_pose == 'up':
                    robot.arm_up()
                    if 'vision' in globals() and vision:
                        vision.update_active_camera(True)
                elif final_pose == 'down':
                    robot.arm_down()
                    if 'vision' in globals() and vision:
                        vision.update_active_camera(False)
                time.sleep(1)
            except Exception:
                pass
            robot.stop()
        if vision is not None:
            vision.stop()
        sys.exit(0)
