import cv2
from picamera2 import Picamera2

def main():
    # Initialize the Picamera2 object
    picam2 = Picamera2()

    # Configure the camera's main stream for 320x240 resolution, BGR format (native to OpenCV), and 30 FPS
    config = picam2.create_video_configuration(
        main={"size": (320, 240), "format": "BGR888"},
        controls={"FrameRate": 30}
    )
    picam2.configure(config)

    # Start the camera
    picam2.start()

    print("Starting live video. Press 'q' to quit.")

    try:
        while True:
            # Capture the latest frame as a numpy array directly in BGR format
            # Using create_video_configuration with BGR888 avoids costly color conversion
            frame_raw = picam2.capture_array()
            
            # Rotate frame by 180 degrees (flip horizontally AND vertically)
            frame_bgr = cv2.flip(frame_raw, -1)
            
            # Display the video frame
            cv2.imshow("Live Video", frame_bgr)
            
            # Wait 1 ms and check if the 'q' key is pressed to exit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    except KeyboardInterrupt:
        # Handle ctrl+c gracefully
        print("Interrupted by user.")
    finally:
        # Clean up resources
        picam2.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
