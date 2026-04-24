import cv2
import sys

def main():
    # Pass the explicit device path to avoid OpenCV falling back to /dev/video0
    device_path = "/dev/video2" 
    # Force the V4L2 backend which handles USB webcams much better on the Pi
    cap = cv2.VideoCapture(device_path, cv2.CAP_V4L2)

    if not cap.isOpened():
        print(f"Error: Cannot open USB camera at {device_path}.")
        print("Try changing the device_path to '/dev/video3' if this fails.")
        sys.exit(1)

    # Set the desired resolution: 320x240
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

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
