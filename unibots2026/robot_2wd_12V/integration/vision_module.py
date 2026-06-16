import cv2
import time
import threading
import logging
from pupil_apriltags import Detector
from ultralytics import YOLO
import glob
from config import *

try:
    from picamera2 import Picamera2
    from libcamera import Transform
except ImportError:
    Picamera2 = None

logger = logging.getLogger(__name__)

def _open_capture_for_device(dev):
    # Some OpenCV builds fail for '/dev/videoX' strings with CAP_V4L2; try numeric index too.
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if cap is not None and cap.isOpened():
        return cap
    if cap is not None:
        cap.release()

    try:
        idx = int(dev.replace('/dev/video', ''))
    except ValueError:
        return None

    cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
    if cap is not None and cap.isOpened():
        return cap
    if cap is not None:
        cap.release()
    return None

def find_usb_camera():
    video_devices = sorted(glob.glob('/dev/video[0-9]*'))
    for dev in video_devices:
        try:
            num = int(dev.replace('/dev/video', ''))
            if num >= 10:
                continue
        except ValueError:
            continue

        cap = _open_capture_for_device(dev)
        if cap is not None and cap.isOpened():
            ret, _ = cap.read()
            cap.release()
            if ret:
                return dev
    return None

class VisionModule:
    def __init__(self):
        self.running = False
        self._state_lock = threading.Lock()
        self.camera_path = find_usb_camera()
        self.cap = None
        self.picam2 = None
        self.model = YOLO(MODEL_PATH, task='detect')
        self.at_detector_usb = Detector(families=APRILTAG_FAMILY, quad_decimate=2.0)
        self.at_detector_picam = Detector(families=APRILTAG_FAMILY, quad_decimate=2.0)
        
        self.frame = None
        self.picam_frame = None
        
        
        self.ball_detected = False
        self.ball_box = None
        self.ball_candidates = []
        self.selected_ball = None
        self.detection_frame = None
        self.ball_x = 0
        self.ball_distance = 999.0
        self.arm_is_up = True
        
        self.usb_camera_params = (742.843247995633, 743.2228374107693, 322.3205884283167, 234.06623771807327)
        self.focal_length_x = self.usb_camera_params[0] * (CAPTURE_WIDTH / 640.0)
        self.picam_camera_params = (954.4949188171072, 955.5979729485147, 332.0798756650343, 245.67451277016548)

    def start(self):
        self.running = True

        # USB camera can enumerate a bit late on boot; retry discovery/open briefly.
        self.cap = None
        for attempt in range(6):
            if not self.camera_path:
                self.camera_path = find_usb_camera()
            if self.camera_path:
                self.cap = _open_capture_for_device(self.camera_path)
            if self.cap is not None and self.cap.isOpened():
                break
            time.sleep(0.5)
            self.camera_path = find_usb_camera()

        if self.cap is not None:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAPTURE_WIDTH)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_HEIGHT)
        else:
            logger.error("USB camera open failed after retries.")

        threading.Thread(target=self._usb_capture_loop, daemon=True).start()
        
        if Picamera2 is not None:
            try:
                self.picam2 = Picamera2()
                config = self.picam2.create_video_configuration(
                    main={"size": (640, 480), "format": "RGB888"},
                    transform=Transform(hflip=1, vflip=1),
                    controls={"FrameRate": 30},
                    buffer_count=2
                )
                self.picam2.configure(config)
                self.picam2.start()

                # Seed an initial PiCamera frame before the background loop starts.
                # This avoids false init failures when startup takes slightly longer.
                seeded = False
                for _ in range(25):
                    try:
                        frame_bgr = self.picam2.capture_array()
                        with self._state_lock:
                            self.picam_frame = frame_bgr.copy()
                        seeded = True
                        break
                    except Exception:
                        time.sleep(0.08)

                if not seeded:
                    logger.warning("PiCamera started but initial frame was not available during warmup window.")

                threading.Thread(target=self._picam_capture_loop, daemon=True).start()
                logger.info("PiCamera started.")
            except Exception as e:
                logger.error(f"PiCamera init error: {e}")

    def stop(self):
        self.running = False
        if self.cap:
            self.cap.release()
        if self.picam2:
            self.picam2.stop()

    def _usb_capture_loop(self):
        frame_idx = 0
        consecutive_read_failures = 0
        current_target_box = None
        
        while self.running:
            if self.cap is None or not self.cap.isOpened():
                time.sleep(0.1)
                continue

            ret, frame = self.cap.read()
            if not ret:
                consecutive_read_failures += 1
                if consecutive_read_failures >= 10:
                    logger.warning("USB camera read failures detected. Attempting reopen.")
                    try:
                        self.cap.release()
                    except Exception:
                        pass
                    self.camera_path = find_usb_camera() or self.camera_path
                    self.cap = _open_capture_for_device(self.camera_path) if self.camera_path else None
                    if self.cap is not None:
                        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAPTURE_WIDTH)
                        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_HEIGHT)
                    consecutive_read_failures = 0
                time.sleep(0.1)
                continue
            consecutive_read_failures = 0
            
            with self._state_lock:
                self.frame = frame
            frame_idx += 1
            
            if frame_idx % SKIP_FRAMES == 0:
                results = self.model.predict(frame, imgsz=320, verbose=False, conf=0.4)
                candidate_boxes = []
                
                if len(results) > 0 and len(results[0].boxes) > 0:
                    for box in results[0].boxes:
                        x, y, w, h = box.xywh[0]
                        w_f, h_f = float(w), float(h)
                        aspect_ratio = w_f / h_f if h_f > 0 else 0
                        area = w_f * h_f
                        
                        if 0.8 <= aspect_ratio <= 1.2:
                            candidate_boxes.append((float(x), float(y), w_f, h_f, area))

                best_match = None
                if candidate_boxes:
                    largest_candidate = max(candidate_boxes, key=lambda b: b[4])
                    closest_candidate = None
                    if current_target_box:
                        cx, cy = current_target_box[0], current_target_box[1]
                        best_dist = TARGET_MATCH_MAX_DIST_PX
                        for b in candidate_boxes:
                            dist = ((b[0] - cx)**2 + (b[1] - cy)**2)**0.5
                            if dist < best_dist:
                                best_dist = dist
                                closest_candidate = b
                    
                    if current_target_box and closest_candidate:
                        # Keep lock on the same target unless a much larger candidate appears.
                        if largest_candidate[4] > closest_candidate[4] * TARGET_SWITCH_AREA_GAIN:
                            best_match = largest_candidate
                        else:
                            best_match = closest_candidate
                    else:
                        best_match = largest_candidate

                largest_box = None
                if best_match:
                    if current_target_box is None:
                        current_target_box = (best_match[0], best_match[1], best_match[2], best_match[3])
                    else:
                        alpha = 0.5
                        current_target_box = (
                            alpha * best_match[0] + (1 - alpha) * current_target_box[0],
                            alpha * best_match[1] + (1 - alpha) * current_target_box[1],
                            alpha * best_match[2] + (1 - alpha) * current_target_box[2],
                            alpha * best_match[3] + (1 - alpha) * current_target_box[3]
                        )
                    largest_box = current_target_box
                else:
                    current_target_box = None
                    largest_box = None
                
                if largest_box:
                    ball_detected = True
                    ball_x = largest_box[0]
                    pixel_diam = min(largest_box[2], largest_box[3])
                    ball_distance = (BALL_DIAMETER_CM * self.focal_length_x) / pixel_diam if pixel_diam > 0 else 999.0
                    ball_box = largest_box
                    selected_ball = largest_box
                else:
                    ball_detected = False
                    ball_x = 0
                    ball_distance = 999.0
                    ball_box = None
                    selected_ball = None

                with self._state_lock:
                    self.ball_candidates = list(candidate_boxes)
                    self.ball_detected = ball_detected
                    self.ball_x = ball_x
                    self.ball_distance = ball_distance
                    self.ball_box = ball_box
                    self.selected_ball = selected_ball
                    # Keep the exact frame used by the current YOLO inference to avoid box/frame drift.
                    self.detection_frame = frame.copy()

                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                try:
                    self.usb_tags = self.at_detector_usb.detect(gray, estimate_tag_pose=False, camera_params=self.usb_camera_params, tag_size=0.10)
                except Exception:
                    self.usb_tags = []
                
            time.sleep(0.01)

    def _picam_capture_loop(self):
        frame_idx = 0
        last_error_log = 0.0

        while self.running:
            try:
                frame_bgr = self.picam2.capture_array()
                frame_idx += 1
                if frame_idx % SKIP_FRAMES != 0:
                    time.sleep(0.01)
                    continue
                    
                with self._state_lock:
                    self.picam_frame = frame_bgr.copy()
                
                gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
                try:
                    self.picam_tags = self.at_detector_picam.detect(gray, estimate_tag_pose=False, camera_params=self.picam_camera_params, tag_size=0.10)
                except Exception:
                    self.picam_tags = []
                    
            except Exception as e:
                now = time.time()
                if now - last_error_log >= PICAM_ERROR_LOG_INTERVAL_S:
                    logger.warning(f"PiCamera capture warning: {e}")
                    last_error_log = now
                time.sleep(0.01)
            time.sleep(0.01)

    def update_active_camera(self, arm_is_up):
        with self._state_lock:
            self.arm_is_up = arm_is_up

    def get_ball_overlay_snapshot(self):
        with self._state_lock:
            frame = None
            if self.detection_frame is not None:
                frame = self.detection_frame.copy()
            elif self.frame is not None:
                frame = self.frame.copy()

            return {
                'frame': frame,
                'ball_detected': bool(self.ball_detected),
                'ball_candidates': list(self.ball_candidates),
                'selected_ball': self.selected_ball,
                'ball_box': self.ball_box,
                'ball_distance': float(self.ball_distance),
            }
        
    @property
    def active_camera_params(self):
        with self._state_lock:
            arm_is_up = getattr(self, 'arm_is_up', True)
        return self.usb_camera_params if arm_is_up else getattr(self, 'picam_camera_params', self.usb_camera_params)
        
    @property
    def active_frame(self):
        with self._state_lock:
            arm_is_up = getattr(self, 'arm_is_up', True)
            frame = self.frame if arm_is_up else self.picam_frame
            return frame.copy() if frame is not None else None
        
    @property
    def tags(self):
        return getattr(self, 'usb_tags', []) if getattr(self, 'arm_is_up', True) else getattr(self, 'picam_tags', [])
