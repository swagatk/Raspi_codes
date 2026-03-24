import time
import os
import cv2
import numpy as np

try:
    from picamera2 import Picamera2
except ImportError:
    print("Picamera2 is not installed or not found. Please install proper packages (e.g. python3-picamera2).")
    exit(1)

def test_camera():
    print("Testing Picamera2 with OpenCV Stream...")
    print("Press 'q' to exit the video stream.")

    try:
        # Initialize Picamera2
        picam2 = Picamera2()

        # Configure the camera for video capture
        # We need BGR format for OpenCV
        config = picam2.create_video_configuration(main={"size": (640, 480), "format": "BGR888"})
        picam2.configure(config)

        # Start the camera system
        picam2.start()

        print("Camera started.")
        
        while True:
            # Capture the array (image) from the camera
            frame = picam2.capture_array()
            
            # Display the frame using OpenCV
            cv2.imshow("Picamera2 Live Stream", frame)
            
            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
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

if __name__ == "__main__":
    test_camera()


