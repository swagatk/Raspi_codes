import cv2
import time
import threading
from pupil_apriltags import Detector
from ultralytics import YOLO
import glob
from config import *

try:
    from picamera2 import Picamera2
    from libcamera import Transform
except ImportError:
    Picamera2 = None

def find_usb_camera():
    video_devices = sorted(glob.glob('/dev/video*'))
    for dev in video_devices:
        try:
            num = int(dev.replace('/dev/video', ''))
            if num >= 10:
                continue
        except ValueError:
            continue
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if cap.isOpened():
            ret, _ = cap.read()
            cap.release()
            if ret:
                return dev
    return "/dev/video2"

class VisionModule:
    def __init__(self):
        self.running = False
        self.camera_path = find_usb_camera()
        self.cap = None
        self.picam2 = None
        self.model = YOLO(MODEL_PATH, task='detect')
        self.at_detector_usb = Detector(families='tagStandard41h12', quad_decimate=2.0)
        self.at_detector_picam = Detector(families='tagStandard41h12', quad_decimate=2.0)
        
        self.frame = None
        self.picam_frame = None
        
        
        self.ball_detected = False
        self.ball_box = None
        self.ball_x = 0
        self.ball_distance = 999.0
        self.arm_is_up = True
        
        self.usb_camera_params = (742.843247995633, 743.2228374107693, 322.3205884283167, 234.06623771807327)
        self.focal_length_x = self.usb_camera_params[0] * (CAPTURE_WIDTH / 640.0)
        self.picam_camera_params = (954.4949188171072, 955.5979729485147, 332.0798756650343, 245.67451277016548)

    def start(self):
        self.running = True
        self.cap = cv2.VideoCapture(self.camera_path, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAPTURE_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_HEIGHT)
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
                threading.Thread(target=self._picam_capture_loop, daemon=True).start()
                print("PiCamera Started.")
            except Exception as e:
                print(f"PiCamera Init Error: {e}")

    def stop(self):
        self.running = False
        if self.cap:
            self.cap.release()
        if self.picam2:
            self.picam2.stop()

    def _usb_capture_loop(self):
        frame_idx = 0
        consecutive_detections = 0
        consecutive_misses = 0
        CONFIRM_FRAMES = 3
        MISS_FRAMES_TOLERANCE = 5
        current_target_box = None
        
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.1)
                continue
            
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
                        best_dist = 80
                        for b in candidate_boxes:
                            dist = ((b[0] - cx)**2 + (b[1] - cy)**2)**0.5
                            if dist < best_dist:
                                best_dist = dist
                                closest_candidate = b
                    
                    if current_target_box and closest_candidate:
                        if largest_candidate[4] > closest_candidate[4] * 1.5:
                            best_match = largest_candidate
                            consecutive_detections = 1
                        else:
                            best_match = closest_candidate
                    else:
                        best_match = largest_candidate

                largest_box = None
                if best_match:
                    consecutive_misses = 0
                    consecutive_detections += 1
                    
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

                    if consecutive_detections >= CONFIRM_FRAMES:
                        largest_box = current_target_box
                else:
                    consecutive_misses += 1
                    if consecutive_misses > MISS_FRAMES_TOLERANCE:
                        current_target_box = None
                        consecutive_detections = 0
                        largest_box = None
                    else:
                        if consecutive_detections >= CONFIRM_FRAMES:
                            largest_box = current_target_box
                
                if largest_box:
                    self.ball_detected = True
                    self.ball_x = largest_box[0]
                    pixel_diam = min(largest_box[2], largest_box[3])
                    self.ball_distance = (BALL_DIAMETER_CM * self.focal_length_x) / pixel_diam if pixel_diam > 0 else 999.0
                    self.ball_box = largest_box
                else:
                    self.ball_detected = False
                    self.ball_box = None

                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                try:
                    self.usb_tags = self.at_detector_usb.detect(gray, estimate_tag_pose=False, camera_params=self.usb_camera_params, tag_size=0.10)
                except Exception:
                    self.usb_tags = []
                
            time.sleep(0.01)

    def _picam_capture_loop(self):
        frame_idx = 0
        while self.running:
            try:
                frame_bgr = self.picam2.capture_array()
                frame_idx += 1
                if frame_idx % SKIP_FRAMES != 0:
                    time.sleep(0.01)
                    continue
                    
                self.picam_frame = frame_bgr.copy()
                
                gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
                try:
                    self.picam_tags = self.at_detector_picam.detect(gray, estimate_tag_pose=False, camera_params=self.picam_camera_params, tag_size=0.10)
                except Exception:
                    self.picam_tags = []
                    
            except Exception as e:
                time.sleep(0.01)
            time.sleep(0.01)

    def update_active_camera(self, arm_is_up):
        self.arm_is_up = arm_is_up
        
    @property
    def active_camera_params(self):
        return self.usb_camera_params if getattr(self, 'arm_is_up', True) else getattr(self, 'picam_camera_params', self.usb_camera_params)
        
    @property
    def active_frame(self):
        return self.frame if getattr(self, 'arm_is_up', True) else self.picam_frame
        
    @property
    def tags(self):
        return getattr(self, 'usb_tags', []) if getattr(self, 'arm_is_up', True) else getattr(self, 'picam_tags', [])
