import time
import threading
import sys
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
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        r, w, e = select.select([sys.stdin], [], [], timeout)
        if r:
            ch = sys.stdin.read(1)
        else:
            ch = None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

button_pressed = False
mission_running = False

def check_button_thread():
    global button_pressed, mission_running
    while True:
        key = get_key(0.5)
        if key:
            key = key.lower()
            if key == 'y':  
                if not mission_running:
                    button_pressed = True
                    mission_running = True
                else:
                    mission_running = False  # emergency stop

def save_annotated_frame(vision, filename):
    if vision.frame is not None:
        annotated = vision.frame.copy()
        if vision.ball_detected:
            cv2.putText(annotated, f"Dist: {vision.ball_distance:.1f}cm", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        try:
            full_path = os.path.join(LOG_DIR, filename)
            cv2.imwrite(full_path, annotated)
        except Exception as e:
            logging.error(f"Failed to save image {full_path}: {e}")

def global_progress_thread():
    TOTAL_DUR = 180
    with tqdm(total=TOTAL_DUR, desc="Mission Progress (3m)", position=0, leave=True, bar_format="{l_bar}{bar}| {n_fmt}/{total_fmt}s") as pbar:
        while mission_running and pbar.n < TOTAL_DUR:
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
    
    # Simple check for camera start
    time.sleep(2)
    if vision.frame is None:
        logging.error("Camera status: FAILED to capture frames. Exiting.")
        robot.stop()
        vision.stop()
        sys.exit(1)
    else:
        logging.info("Camera status: OK")
        
    # Assuming ultrasonic is returning valid numbers
    if robot.latest_C == 999 and robot.latest_L == 999:
        logging.warning("Ultrasonic sensor status: No initial readings received yet.")
    else:
        logging.info(f"Ultrasonic sensor status: OK (L:{robot.latest_L} C:{robot.latest_C} R:{robot.latest_R})")

    logging.info("Moving ARM to UP pose.")
    robot.arm_up()
    time.sleep(1)
    
    threading.Thread(target=check_button_thread, daemon=True).start()
    logging.info("System Online. Waiting for button press ('y' key) to start mission sequence...")
    
    while not button_pressed:
        time.sleep(0.1)
        
    logging.info("Button pressed. Mission Started!")
    if not mission_running: return
    
    threading.Thread(target=global_progress_thread, daemon=True).start()
    
    # ---------------------------------------------------------
    # STEP 2: Confirm Home Location (~20 seconds)
    # ---------------------------------------------------------
    logging.info("--- Step 2: Confirm Home Location ---")
    start_time = time.time()
    home_confirmed = False
    
    while mission_running and (time.time() - start_time < 20.0):
        # We expect some delay to fetch tags
        time.sleep(0.2)
        visible_tags = [t.tag_id for t in vision.tags]
        matched = [t for t in config.HOME_TAG_IDS if t in visible_tags]
        
        if len(matched) > 0: # At least one home tag
            logging.info(f"Home tag(s) {matched} confirmed!")
            save_annotated_frame(vision, "home_tag.jpg")
            home_confirmed = True
            break
        else:
            logging.info("Home tags not visible. Adjusting view backwards...")
            robot.move('B', '1')
            time.sleep(0.3)
            robot.halt()
            time.sleep(0.5) # Wait for camera to stabilize
            
    if not home_confirmed:
        logging.error("Home tags not detected within time limit. Exiting mission.")
        robot.stop()
        vision.stop()
        sys.exit(1)
        
    logging.info("Step 2 completed successfully.")
    if not mission_running: return

    # ---------------------------------------------------------
    # STEP 3: Exploration (~ 2 minutes)
    # ---------------------------------------------------------
    logging.info("--- Step 3: Exploration Phase ---")
    exploration_start = time.time()
    
    while mission_running and (time.time() - exploration_start < 120.0):
        
        # 1. Search for target
        logging.info("Searching for target...")
        ball_found = False
        search_start = time.time()
        
        while mission_running and (time.time() - search_start < 15.0) and (time.time() - exploration_start < 120.0):
            if vision.ball_detected:
                logging.info(f"Target detected at {vision.ball_distance:.1f}cm! Saving image.")
                save_annotated_frame(vision, f"target_detected_{int(time.time())}.jpg")
                ball_found = True
                break
            # Rotate by a slightly larger amount to search faster
            robot.move('R', '2')
            time.sleep(0.3)
            robot.halt()
            time.sleep(0.3)
            
        if not ball_found:
            # Continue tracking elapsed time
            continue
            
        # 2. Approach target
        logging.info("Approaching the target...")
        reached = False
        approach_start = time.time()
        
        while mission_running and (time.time() - approach_start < 30.0) and (time.time() - exploration_start < 120.0):
            # Obstacle avoidance priority
            if robot.latest_C < STOP_DIST:
                logging.warning(f"Obstacle in center ({robot.latest_C}cm)! Evading...")
                robot.halt()
                robot.move('B', '1')
                time.sleep(0.3)
                robot.move('L', '1')
                time.sleep(0.3)
                robot.halt()
                time.sleep(0.2)
                continue
                
            if vision.ball_detected:
                error = vision.ball_x - 160
                if abs(error) >= CENTER_TOLERANCE:
                    # Centering
                    if error < 0: robot.move('L', '1')
                    else: robot.move('R', '1')
                    time.sleep(0.1)
                    robot.halt()
                    time.sleep(0.2)
                else:
                    if vision.ball_distance <= TARGET_REACHED_CM:
                        logging.info("Target reached distance limit.")
                        robot.halt()
                        reached = True
                        break
                    else:
                        robot.move('F', '1')
                        time.sleep(0.2)
                        robot.halt()
                        time.sleep(0.1)
            else:
                logging.warning("Target lost during approach. Will rescan.")
                robot.halt()
                break # break approach loop, re-enter search
                
        # 3. Grab the ball
        if reached:
            logging.info("Grabbing the ball...")
            robot.arm_down()
            time.sleep(2.0)
            robot.gripper_open()
            time.sleep(1.0)
            
            logging.info(f"Moving forward {config.CAPTURE_DISTANCE_CM}cm while toggling gripper {config.CAPTURE_GRIPPER_CYCLES} times...")
            robot.move('F', '2')
            
            time_to_move = config.CAPTURE_DISTANCE_CM / config.CAPTURE_SPEED_CM_S
            
            if config.CAPTURE_GRIPPER_CYCLES > 0:
                toggle_interval = time_to_move / (config.CAPTURE_GRIPPER_CYCLES * 2)
            else:
                toggle_interval = time_to_move + 1  # Never toggle
                
            start_m = time.time()
            last_toggle = start_m
            gripper_is_open = True
            
            while (time.time() - start_m) < time_to_move:
                current_time = time.time()
                if current_time - last_toggle >= toggle_interval:
                    if gripper_is_open:
                        robot.gripper_close()
                        gripper_is_open = False
                    else:
                        robot.gripper_open()
                        gripper_is_open = True
                    last_toggle = current_time
                time.sleep(0.02)
                
            robot.gripper_close()
            time.sleep(0.5)
            robot.halt()
            time.sleep(0.5)
            
            robot.arm_up()
            time.sleep(2)
            logging.info("Ball grab complete.")
            
    logging.info("Step 3 Exploration time completed.")
    if not mission_running: return

    # ---------------------------------------------------------
    # STEP 4: Returning home (~40 seconds)
    # ---------------------------------------------------------
    logging.info("--- Step 4: Returning home ---")
    home_start = time.time()
    
    # We would normally estimate robot pose here using pupil_apriltags map
    logging.info("Estimating robot pose from vision data...")
    
    while mission_running and (time.time() - home_start < 40.0):
        visible_tags = [t.tag_id for t in vision.tags]
        matched = [t for t in config.HOME_TAG_IDS if t in visible_tags]
        
        if len(matched) > 0:
            logging.info("Home tag spotted. Approaching...")
            if robot.latest_C <= config.TARGET_REACHED_CM + 10:
                logging.info("Arrived at home location.")
                robot.halt()
                break
            else:
                robot.move('F', '1')
                time.sleep(0.2)
                robot.halt()
                time.sleep(0.1)
        else:
            # Spin with a slightly larger rotation to scan for home tags faster
            robot.move('R', '2')
            time.sleep(0.3)
            robot.halt()
            time.sleep(0.3)
            
    # Drop sequence
    logging.info("Executing Ball drop...")
    robot.arm_drop()
    time.sleep(2)
    robot.gripper_open()
    time.sleep(1)
    
    robot.arm_down()
    robot.halt()
    logging.info("Mission sequence finalized. Entering rest pose and shutting down.")

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
            logging.info("Moving ARM to DOWN pose before exit.")
            try:
                robot.arm_down()
                time.sleep(1)
            except:
                pass
            robot.stop()
        if vision is not None:
            vision.stop()
        sys.exit(0)
