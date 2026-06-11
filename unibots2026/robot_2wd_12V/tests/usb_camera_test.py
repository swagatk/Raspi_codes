import cv2
import sys
from pathlib import Path


def _video_device_sort_key(path_obj: Path):
    name = path_obj.name  # e.g. video0
    suffix = name.replace("video", "", 1)
    return int(suffix) if suffix.isdigit() else 10_000


def find_available_camera():
    devices = sorted(Path("/dev").glob("video*"), key=_video_device_sort_key)

    if not devices:
        print("Error: No /dev/video* devices found.")
        return None, None

    for device in devices:
        device_path = str(device)
        cap = cv2.VideoCapture(device_path, cv2.CAP_V4L2)
        if cap.isOpened():
            return cap, device_path
        cap.release()

    print("Error: Found /dev/video* devices, but none could be opened.")
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
