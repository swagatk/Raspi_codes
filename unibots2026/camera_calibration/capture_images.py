"""
Capture images for camera calibration using Picamera2 or USB camera. Press 'SPACE' to save an image and 'Q' to quit.
"""
import cv2
import os
import glob

# --- CONFIGURATION ---
CAMERA_TYPE = "usb" # Options: "picamera" or "usb"

def find_usb_camera():
    video_devices = sorted(glob.glob('/dev/video*'))
    for dev in video_devices:
        try:
            num = int(dev.replace('/dev/video', ''))
            if num >= 10:
                continue
        except ValueError:
            continue
            
        print(f"Testing {dev}...")
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if cap.isOpened():
            ret, _ = cap.read()
            cap.release()
            if ret:
                print(f"Found working camera at {dev}")
                return dev
    return "/dev/video2" # Fallback

USB_CAMERA_PATH = find_usb_camera()
# ---------------------

try:
    if CAMERA_TYPE == "picamera":
        from picamera2 import Picamera2
        from libcamera import Transform
except ImportError:
    if CAMERA_TYPE == "picamera":
        print("Picamera2 is not installed or not found. Please install proper packages (e.g. python3-picamera2).")
        exit(1)

# Create folder if it doesn't exist
calibration_dir = os.path.expanduser(f'~/calibration_images_{CAMERA_TYPE}')
if not os.path.exists(calibration_dir):
    os.makedirs(calibration_dir)

try:
    picam2 = None
    cap = None
    
    # Initialize Camera
    if CAMERA_TYPE == "picamera":
        picam2 = Picamera2()
        config = picam2.create_video_configuration(main={"size": (640, 480), "format": "RGB888"})
        picam2.configure(config)
        picam2.start()
        print("PiCamera Started.")
    elif CAMERA_TYPE == "usb":
        cap = cv2.VideoCapture(USB_CAMERA_PATH, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        if not cap.isOpened():
            raise Exception(f"Cannot open USB camera at {USB_CAMERA_PATH}")
        print(f"USB Camera Started ({USB_CAMERA_PATH}).")
    else:
        raise Exception("Invalid CAMERA_TYPE specified.")

    count = 0
    print("Press 'SPACE' to save an image. Press 'Q' to quit.")

    while True:
        # Capture frame
        if CAMERA_TYPE == "picamera":
            frame = picam2.capture_array()
        else:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame. Reconnecting...")
                import time; time.sleep(0.5)
                continue
        
        cv2.imshow("Calibration Capture (640x480)", frame)
        key = cv2.waitKey(1)
        
        if key == 32:  # SPACEBAR
            img_name = f"{calibration_dir}/calib_{count}.jpg"
            cv2.imwrite(img_name, frame)
            print(f"Saved {img_name}!")
            count += 1
        elif key == ord('q'):
            break

    # Stop the camera and close windows
    if picam2: picam2.stop()
    if cap: cap.release()
    cv2.destroyAllWindows()
    print("Camera stopped.")

except Exception as e:
    import traceback
    traceback.print_exc()
    print(f"An error occurred: {e}")
    # Ensure cleanup if possible
    try:
        if picam2: picam2.stop()
        if cap: cap.release()
        cv2.destroyAllWindows()
    except:
        pass
