"""
Capture images for camera calibration using Picamera2. Press 'SPACE' to save an image and 'Q' to quit.
"""
import cv2
import os

try:
    from picamera2 import Picamera2
except ImportError:
    print("Picamera2 is not installed or not found. Please install proper packages (e.g. python3-picamera2).")
    exit(1)

# Create folder if it doesn't exist
calibration_dir = os.path.expanduser('~/calibration_images')
if not os.path.exists(calibration_dir):
    os.makedirs(calibration_dir)

try:
    # Initialize Picamera2
    picam2 = Picamera2()

    # Configure the camera for video capture
    # We need BGR format for OpenCV
    config = picam2.create_video_configuration(main={"size": (640, 480), "format": "RGB888"})
    picam2.configure(config)

    # Start the camera system
    picam2.start()

    count = 0
    print("Camera started.")
    print("Press 'SPACE' to save an image. Press 'Q' to quit.")

    while True:
        # Capture the array (image) from the camera
        frame = picam2.capture_array()
        
        cv2.imshow("Calibration Capture (640x480)", frame)
        key = cv2.waitKey(1)
        
        if key == 32:  # SPACEBAR
            img_name = f"calibration_images/calib_{count}.jpg"
            cv2.imwrite(img_name, frame)
            print(f"Saved {img_name}!")
            count += 1
        elif key == ord('q'):
            break

    # Stop the camera and close windows
    picam2.stop()
    cv2.destroyAllWindows()
    print("Camera stopped.")

except Exception as e:
    import traceback
    traceback.print_exc()
    print(f"An error occurred: {e}")
    # Ensure cleanup if possible
    try:
        picam2.stop()
        cv2.destroyAllWindows()
    except:
        pass
