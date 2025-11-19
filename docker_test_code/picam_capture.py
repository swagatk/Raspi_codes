import cv2
import time

print("Connecting to Camera Stream on Host...")

# INSTEAD OF INDEX (0 or 1), USE THE TCP ADDRESS
# Since you used --network host, you can use localhost (127.0.0.1)
cap = cv2.VideoCapture("tcp://127.0.0.1:5000")

if not cap.isOpened():
    print("Error: Could not connect to stream.")
    print("Make sure rpicam-vid is running on the host terminal!")
    exit()

# Allow stream to buffer slightly
time.sleep(1)

ret, frame = cap.read()

if ret:
    filename = "docker_image.jpg"
    cv2.imwrite(filename, frame)
    print(f"Success! Image saved as {filename}")
else:
    print("Error: Failed to capture frame from stream.")

cap.release()
