import cv2
import time
import threading
from pupil_apriltags import Detector
from ultralytics import YOLO
import glob
from config import *

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
        self.model = YOLO(MODEL_PATH, task='detect')
        self.at_detector = Detector(families='tagStandard41h12', quad_decimate=2.0)
        self.frame = None
        self.ball_detected = False
        self.ball_x = 0
        self.ball_distance = 999.0
        self.tags = []
        
        # Fixed scale calibration factors
        self.camera_params = (742.84, 743.22, 322.32, 234.06) 
        self.focal_length_x = self.camera_params[0] * (CAPTURE_WIDTH / 640.0)

    def start(self):
        self.running = True
        self.cap = cv2.VideoCapture(self.camera_path, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAPTURE_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_HEIGHT)
        threading.Thread(target=self._capture_loop, daemon=True).start()

    def stop(self):
        self.running = False
        if self.cap:
            self.cap.release()

    def _capture_loop(self):
        frame_idx = 0
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.1)
                continue
            
            self.frame = frame
            frame_idx += 1
            
            # Sub-sample frames
            if frame_idx % SKIP_FRAMES == 0:
                # 1. YOLO inference
                results = self.model.predict(frame, imgsz=320, verbose=False, conf=0.4)
                best_ball = None
                max_area = 0
                if len(results) > 0 and len(results[0].boxes) > 0:
                    for box in results[0].boxes:
                        x, y, w, h = box.xywh[0]
                        area = float(w * h)
                        if area > max_area:
                            max_area = area
                            best_ball = (float(x), float(w), float(h))
                
                if best_ball:
                    self.ball_detected = True
                    self.ball_x = best_ball[0]
                    pixel_diam = min(best_ball[1], best_ball[2])
                    self.ball_distance = (BALL_DIAMETER_CM * self.focal_length_x) / pixel_diam if pixel_diam > 0 else 999
                else:
                    self.ball_detected = False
                
                # 2. AprilTag detect
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                try:
                    # estimate_tag_pose=False to prevent C-level aborts during fast motion blur
                    self.tags = self.at_detector.detect(gray, estimate_tag_pose=False, camera_params=self.camera_params, tag_size=0.10)
                except Exception as e:
                    import logging
                    logging.warning(f"AprilTag detection error skipped: {e}")
                    pass
                
            time.sleep(0.01)
