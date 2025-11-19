import cv2
import time

print("Initializing Camera...")

# Open the camera using the V4L2 backend (Required for Bookworm)
# Index 0 is usually the default Pi Camera
cap = cv2.VideoCapture(2, cv2.CAP_V4L2)

# Optional: Set Resolution
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    print("Error: Could not open video device.")
    print("Make sure you mapped '/dev' in your docker run command.")
    exit()

# Allow the camera to warm up
time.sleep(2)

# Capture one frame
ret, frame = cap.read()

if ret:
    filename = "test_image.jpg"
    cv2.imwrite(filename, frame)
    print(f"Success! Image saved as {filename}")
else:
    print("Error: Failed to capture image.")

# Release the camera
cap.release()
