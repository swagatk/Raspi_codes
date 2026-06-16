import cv2
import sys
from pathlib import Path
import os


def _video_device_sort_key(path_obj: Path):
    name = path_obj.name  # e.g. video0
    suffix = name.replace("video", "", 1)
    return int(suffix) if suffix.isdigit() else 10_000


def _is_usb_video_node(path_obj: Path):
    sysfs_node = Path("/sys/class/video4linux") / path_obj.name / "device"
    try:
        resolved = Path(os.path.realpath(sysfs_node))
    except OSError:
        return False
    return "usb" in str(resolved)


def _can_grab_frame(cap):
    for _ in range(10):
        ok, _ = cap.read()
        if ok:
            return True
    return False


def find_available_camera():
    devices = sorted(Path("/dev").glob("video*"), key=_video_device_sort_key)
    usb_devices = [d for d in devices if _is_usb_video_node(d)]
    candidates = usb_devices if usb_devices else devices

    if not candidates:
        print("Error: No /dev/video* devices found.")
        return None, None

    for device in candidates:
        device_path = str(device)
        cap = cv2.VideoCapture(device_path, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            continue

        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

        if _can_grab_frame(cap):
            return cap, device_path
        cap.release()

    print(f"Error: Camera nodes found ({[str(d) for d in candidates]}), but none returned frames.")
    return None, None

def main():
    cap, device_path = find_available_camera()
    if cap is None:
        sys.exit(1)

    # Set the desired resolution: 320x240
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    print(f"Using camera device: {device_path}")
    print("USB Camera Live View started.")
    print("Press 'q' inside the video window to quit.")

    while True:
        # Read a frame from the camera
        ret, frame = cap.read()
        
        if not ret:
            print("Error: Failed to grab frame.")
            break

        # Display the frame
        cv2.imshow("USB Camera Test - 320x240", frame)

        # Listen for the 'q' key to stop the loop
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print("Quit signal received.")
            break

    # Release the camera and close the window
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
