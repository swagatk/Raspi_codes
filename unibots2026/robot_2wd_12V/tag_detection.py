import cv2
import os
from pathlib import Path
from picamera2 import Picamera2
import time
from libcamera import Transform

from pupil_apriltags import Detector

# --- AprilTag Detector setup ---
# Ensure "tagStandard41h12" matches your physical tags
detector = Detector(families='tag36h11')

# ------- Camera Setup -----
# Choose camera source: "picamera" or "usb"
CAMERA_TYPE = "usb"
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240


def open_usb_camera(width, height):
    def video_device_sort_key(path_obj: Path):
        name = path_obj.name  # e.g. video0
        suffix = name.replace("video", "", 1)
        return int(suffix) if suffix.isdigit() else 10_000

    def is_usb_video_node(path_obj: Path):
        sysfs_node = Path("/sys/class/video4linux") / path_obj.name / "device"
        try:
            resolved = Path(os.path.realpath(sysfs_node))
        except OSError:
            return False
        return "usb" in str(resolved)

    def can_grab_frame(video_cap):
        for _ in range(10):
            ok, _ = video_cap.read()
            if ok:
                return True
        return False

    devices = sorted(Path("/dev").glob("video*"), key=video_device_sort_key)
    usb_devices = [d for d in devices if is_usb_video_node(d)]
    candidates = usb_devices if usb_devices else devices

    if not candidates:
        raise RuntimeError("No USB camera device found under /dev/video*")

    for node in candidates:
        node_path = str(node)
        cap = cv2.VideoCapture(node_path, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            continue

        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        if can_grab_frame(cap):
            return cap, node_path
        cap.release()

    raise RuntimeError(f"Found devices {candidates}, but none could stream frames")


picam = None
cap = None

if CAMERA_TYPE == "picamera":
    picam = Picamera2()
    config = picam.create_preview_configuration(
        main={"size": (IMAGE_WIDTH, IMAGE_HEIGHT), "format": "RGB888"},
        # AprilTag decoding fails on mirrored images; use 180-degree rotation instead.
        transform=Transform(hflip=True, vflip=True),
    )
    picam.configure(config)
    picam.start()
    print("Using PiCamera2")
elif CAMERA_TYPE == "usb":
    cap, node = open_usb_camera(IMAGE_WIDTH, IMAGE_HEIGHT)
    print(f"Using USB camera: {node}")
else:
    raise ValueError("CAMERA_TYPE must be 'picamera' or 'usb'")

print("Starting AprilTag detection... Press 'q' to quit.")

try:
    prev_time = time.time()
    while True:
        if CAMERA_TYPE == "picamera":
            frame = picam.capture_array()
            gray_image = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        else:
            ok, frame = cap.read()
            if not ok:
                print("Failed to read frame from USB camera")
                break
            gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        detections = detector.detect(gray_image)
        
        # --- Visualization ---
        for detection in detections:
            # 1. Get Corners
            corners = detection.corners.astype(int)
            
            # 2. Get Tag ID
            tag_id = str(detection.tag_id)

            # 3. Get Center
            center = detection.center.astype(int)

            # --- DRAWING ---
            # Draw the bounding box
            cv2.polylines(frame, [corners], isClosed=True, color=(0, 255, 0), thickness=2)

            # Draw the Center point
            cv2.circle(frame, tuple(center), 5, (0, 0, 255), -1)

            # Draw the Tag ID above the first corner
            cv2.putText(frame, f"ID: {tag_id}", (corners[0][0], corners[0][1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        
        # Display the frame
        # --- FPS Calculation ---
        curr_time = time.time()
        fps = 1 / (curr_time - prev_time)
        prev_time = curr_time
        cv2.putText(frame, f"FPS: {int(fps)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        
        # --- Display ---
        cv2.imshow("AprilTag Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except Exception as e:
    print(f'View camera errors: {e}')

finally:
    if picam is not None:
        picam.stop()
    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()