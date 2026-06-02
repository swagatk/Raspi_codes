import time
import threading
import sys
import math
import termios
import tty
import select
import os
import cv2
import logging
from tqdm import tqdm
from robot_hardware import RobotController
from vision_module import VisionModule
import config
from config import *

LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "log")
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

def get_key(timeout=0.1):
    if not sys.stdin.isatty():
        return None
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        r, w, e = select.select([sys.stdin], [], [], timeout)
        if r:
            ch = sys.stdin.read(1)
            # cbreak mode disables normal tty echo; echo typed control keys explicitly.
            if ch and ch.isprintable():
                sys.stdout.write(ch)
                sys.stdout.flush()
        else:
            ch = None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def restore_terminal_echo():
    if not sys.stdin.isatty():
        return
    fd = sys.stdin.fileno()
    try:
        attrs = termios.tcgetattr(fd)
        attrs[3] |= termios.ECHO
        termios.tcsetattr(fd, termios.TCSADRAIN, attrs)
    except Exception:
        pass

button_pressed = False
mission_running = False

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
    return mid_x, mid_y

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
    step_duration = COURSE_FWD_MOTION_DIST_CM / max(COURSE_SPEED_CM_S, 1e-6)
    return execute_burst('F', COURSE_FORWARD_SPEED, step_duration, APPROACH_FORWARD_SETTLE_S)

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
    global button_pressed, mission_running
    while True:
        key = get_key(0.5)
        if key:
            key = key.lower()
            if key == START_STOP_KEY:
                if not mission_running:
                    button_pressed = True
                    mission_running = True
                    logging.info(f"Start command received via '{START_STOP_KEY}' key.")
                else:
                    request_mission_stop(f"Stop command received via '{START_STOP_KEY}' key.")

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
    if hasattr(vision, 'active_frame') and vision.active_frame is not None:
        annotated = vision.active_frame.copy()
    elif vision.frame is not None:
        annotated = vision.frame.copy()
    else:
        return
    # Draw all valid detections in blue, and selected target in cyan.
    for cand in getattr(vision, 'ball_candidates', []):
        cx, cy, w, h, _ = cand
        x1 = int(cx - w / 2)
        y1 = int(cy - h / 2)
        x2 = int(cx + w / 2)
        y2 = int(cy + h / 2)
        cv2.rectangle(annotated, (x1, y1), (x2, y2), (255, 0, 0), 2)

    target_box = getattr(vision, 'selected_ball', None) or getattr(vision, 'ball_box', None)
    if vision.ball_detected and target_box:
        x, y, w, h = target_box[:4]
        x1 = int(x - w / 2)
        y1 = int(y - h / 2)
        x2 = int(x + w / 2)
        y2 = int(y + h / 2)
        cv2.rectangle(annotated, (x1, y1), (x2, y2), (255, 255, 0), 3)
        cv2.putText(annotated, f"TARGET Dist: {vision.ball_distance:.1f}cm | Area: {w*h:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)
    elif vision.ball_detected:
        cv2.putText(annotated, f"Dist: {vision.ball_distance:.1f}cm", (10, 30),
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

def get_confirmed_pose_with_camera_fallback(vision, arena_map, estimate_robot_pose_func):
    # Confirm pose across consecutive frames to reject one-off jitter.
    confirmed_count = 0
    last_pose = None
    last_cam = None

    while mission_running and confirmed_count < STEP5_POSE_CONFIRM_FRAMES:
        pose, pose_cam = get_pose_with_camera_fallback(vision, arena_map, estimate_robot_pose_func)
        if not pose:
            return None, None

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
                confirmed_count = 1

            last_pose = pose
            last_cam = pose_cam

        if confirmed_count < STEP5_POSE_CONFIRM_FRAMES:
            time.sleep(STEP5_POSE_CONFIRM_INTERVAL_S)

    return last_pose, last_cam

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

def compute_heading_and_distance_to_home(rx, ry, heading, home_target_xy):
    dx = home_target_xy[0] - rx
    dy = home_target_xy[1] - ry
    linear_distance_m = math.hypot(dx, dy)
    desired_heading = math.degrees(math.atan2(dy, dx)) % 360.0
    rotation_deg = normalize_rotation(desired_heading - heading)
    return desired_heading, rotation_deg, linear_distance_m

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

def main():
    global robot, vision
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

    logging.info(f"Button status: Listener running (press '{START_STOP_KEY}' to start/stop).")

    logging.info("Moving ARM to UP pose.")
    robot.arm_up()
    if 'vision' in globals() and vision:
        vision.update_active_camera(True)
    time.sleep(1)
    
    threading.Thread(target=check_button_thread, daemon=True).start()
    logging.info(f"System Online. Waiting for button press ('{START_STOP_KEY}' key) to start mission sequence...")
    
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
        confirm_count = 0
        last_confirm_x = None
        last_confirm_dist = None
        search_start = time.time()
        
        while mission_running and (time.time() - search_start < BALL_SEARCH_WINDOW_S) and (time.time() - exploration_start < step4_budget_s):
            if vision.ball_detected:
                robot.halt()
                current_x = float(vision.ball_x)
                current_dist = float(vision.ball_distance)

                stable_with_previous = (
                    last_confirm_x is None
                    or (
                        abs(current_x - last_confirm_x) <= STEP4_BALL_CONFIRM_MAX_X_JUMP_PX
                        and abs(current_dist - last_confirm_dist) <= STEP4_BALL_CONFIRM_MAX_DIST_JUMP_CM
                    )
                )

                if stable_with_previous:
                    confirm_count += 1
                else:
                    confirm_count = 1

                last_confirm_x = current_x
                last_confirm_dist = current_dist

                logging.info(
                    f"Ball candidate confirmation {confirm_count}/{STEP4_BALL_CONFIRM_FRAMES} "
                    f"(dist={current_dist:.1f}cm, x={current_x:.1f})"
                )

                # Keep robot stationary during confirmation checks.
                confirm_deadline = time.time() + 1.2
                while (
                    mission_running
                    and confirm_count < STEP4_BALL_CONFIRM_FRAMES
                    and time.time() < confirm_deadline
                    and (time.time() - search_start < BALL_SEARCH_WINDOW_S)
                    and (time.time() - exploration_start < step4_budget_s)
                ):
                    time.sleep(0.12)
                    if not vision.ball_detected:
                        logging.info("Ball confirmation reset due to missed frame.")
                        confirm_count = 0
                        last_confirm_x = None
                        last_confirm_dist = None
                        break

                    current_x = float(vision.ball_x)
                    current_dist = float(vision.ball_distance)
                    stable_with_previous = (
                        last_confirm_x is None
                        or (
                            abs(current_x - last_confirm_x) <= STEP4_BALL_CONFIRM_MAX_X_JUMP_PX
                            and abs(current_dist - last_confirm_dist) <= STEP4_BALL_CONFIRM_MAX_DIST_JUMP_CM
                        )
                    )

                    if stable_with_previous:
                        confirm_count += 1
                    else:
                        # Target likely changed; restart confirmation from this stationary frame.
                        confirm_count = 1

                    last_confirm_x = current_x
                    last_confirm_dist = current_dist

                    logging.info(
                        f"Ball candidate confirmation {confirm_count}/{STEP4_BALL_CONFIRM_FRAMES} "
                        f"(dist={current_dist:.1f}cm, x={current_x:.1f})"
                    )

                if confirm_count >= STEP4_BALL_CONFIRM_FRAMES:
                    logging.info(f"Target confirmed at {vision.ball_distance:.1f}cm! Saving image.")
                    save_annotated_frame(vision, f"ball_target_{time.strftime('%Y%m%d_%H%M%S')}.jpg")
                    ball_found = True
                    break
            else:
                if confirm_count > 0:
                    logging.info("Ball confirmation reset due to missed frame.")
                confirm_count = 0
                last_confirm_x = None
                last_confirm_dist = None
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
                if abs(error) >= CENTER_TOLERANCE:
                    # Bring target to center slowly before forward advance.
                    if error < 0:
                        moved = execute_burst('L', CENTERING_TURN_SPEED, CENTERING_TURN_BURST_S, CENTERING_SETTLE_S)
                    else:
                        moved = execute_burst('R', CENTERING_TURN_SPEED, CENTERING_TURN_BURST_S, CENTERING_SETTLE_S)
                    if not moved:
                        break
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
            if robot.latest_L < STOP_DIST or robot.latest_C < STOP_DIST or robot.latest_R < STOP_DIST:
                logging.warning("Safety limits breached during catch! Aborting!")
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
    from apriltag_detection import estimate_robot_pose
    from map_visualizer import generate_localization_image
    home_target_xy = compute_home_target_xy(ARENA_MAP)
    if home_target_xy is None:
        logging.error("Home target cannot be computed from map/home tags. Exiting mission.")
        return
    
    logging.info("Moving to Home tag")
    
    while mission_running and (time.time() - home_start < STEP5_RETURN_HOME_TIMEOUT_S):
        if is_valid_distance_cm(robot.latest_C) and robot.latest_C <= HOME_WALL_STOP_DISTANCE:
            near_home_confirm += 1
            logging.info(
                f"Home wall proximity check {near_home_confirm}/{HOME_WALL_STOP_CONFIRM_COUNT}: "
                f"C={robot.latest_C:.1f}cm <= {HOME_WALL_STOP_DISTANCE:.1f}cm"
            )
            if near_home_confirm >= HOME_WALL_STOP_CONFIRM_COUNT:
                logging.info("Arrived close to home wall.")
                robot.halt()
                break
        else:
            near_home_confirm = 0

        if obstacle_detected(robot):
            if not run_obstacle_avoidance(robot):
                break
            continue

        visible_home_tags = any(
            getattr(tag, 'tag_id', None) in HOME_TAG_IDS
            for tag in (list(getattr(vision, 'usb_tags', [])) + list(getattr(vision, 'picam_tags', [])))
        )
        if visible_home_tags:
            logging.info("State: tag spotted .. estimating robot pose")
        else:
            logging.info("State: estimating robot pose (USB first, then PiCamera fallback)")
        pose, pose_cam = get_confirmed_pose_with_camera_fallback(vision, ARENA_MAP, estimate_robot_pose)

        if not pose:
            failed_localize_cycles += 1
            logging.warning("Localization failed on USB and PiCamera. Backing up and retrying.")
            if not execute_burst(
                'B',
                STEP5_RELOCALIZE_BACKUP_SPEED,
                STEP5_RELOCALIZE_BACKUP_DURATION_S,
                STEP5_RELOCALIZE_BACKUP_SETTLE_S,
            ):
                break
            if failed_localize_cycles % 3 == 0:
                logging.warning(
                    f"Localization has failed {failed_localize_cycles} consecutive cycles. "
                    "Verify that wall tags are visible in at least one camera view."
                )
            continue

        failed_localize_cycles = 0

        rx, ry, heading = pose
        logging.info(
            f"Robot Pose (2-frame confirmed): x={rx:.2f}, y={ry:.2f}, heading={heading:.1f} (camera={pose_cam})"
        )

        now = time.time()
        if (now - last_localize_save_ts) >= STEP5_LOCALIZE_IMAGE_INTERVAL_S:
            save_path = os.path.join(LOG_DIR, f"localize_{time.strftime('%Y%m%d_%H%M%S')}.jpg")
            generate_localization_image(rx, ry, heading, ARENA_MAP, config.HOME_TAG_IDS, save_path)
            logging.info(f"Saved layout visualization to {save_path}")
            last_localize_save_ts = now

        desired_heading, rotation_deg, linear_distance_m = compute_heading_and_distance_to_home(
            rx, ry, heading, home_target_xy
        )
        logging.info(
            f"Computed to-home command: rotation={rotation_deg:+.1f}deg, "
            f"linear_distance={linear_distance_m:.2f}m"
        )
        aligned, heading_error, turn_dir = rotate_towards_heading(heading, desired_heading)
        if aligned is None:
            break
        if not aligned:
            logging.info(
                f"Aligning to home heading in larger step: error={heading_error:+.1f}deg -> turn {turn_dir}"
            )
            if abs(heading_error) > STEP5_FORWARD_AFTER_TURN_MAX_ERROR_DEG:
                logging.info(
                    f"Heading error still large ({heading_error:+.1f}deg). Skipping forward motion this cycle."
                )
                continue

        logging.info("Home tag spotted. Approaching...")
        if is_valid_distance_cm(robot.latest_C) and robot.latest_C < FINE_TUNING_HOME_DIST_CM:
            tag_error_px = get_home_tag_center_error_px(vision)
            if tag_error_px is not None and abs(tag_error_px) > CENTER_TOLERANCE:
                turn_dir = 'L' if tag_error_px < 0 else 'R'
                logging.info(
                    f"Fine tuning near wall: tag_error_px={tag_error_px:+.1f}, turn={turn_dir}"
                )
                if not execute_burst(turn_dir, CENTERING_TURN_SPEED, 0.04, CENTERING_SETTLE_S):
                    break
            else:
                logging.info("Fine tuning near wall: moving forward cautiously.")
                if not execute_burst('F', APPROACH_FORWARD_SPEED, 0.10, APPROACH_FORWARD_SETTLE_S):
                    break
        else:
            logging.info(
                f"Course approach: advancing {COURSE_FWD_MOTION_DIST_CM:.1f}cm toward home."
            )
            if not execute_course_step_forward():
                break

    if not mission_running:
        logging.info("Mission stopped before drop sequence.")
        return

    logging.info("Executing Ball drop...")
    robot.arm_drop()
    wait_with_abort(2)
    robot.gripper_open()
    wait_with_abort(1)

    robot.arm_down()
    if 'vision' in globals() and vision:
        vision.update_active_camera(False)
    robot.halt()
    logging.info("Mission sequence finalized. Entering rest pose and shutting down.")

global robot, vision
robot = None
vision = None

if __name__ == "__main__":
    try:
        restore_terminal_echo()
        main()
    except KeyboardInterrupt:
        logging.info("Interrupted by user (Ctrl+C)")
    except Exception as e:
        logging.error(f"Mission crashed due to exception: {e}")
    finally:
        restore_terminal_echo()
        if robot is not None:
            logging.info("Moving ARM to DOWN pose before exit.")
            try:
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
