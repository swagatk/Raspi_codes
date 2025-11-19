import cv2

# --- CONFIGURATION ---
# OPTION A: Use this for Raspberry Pi 5 Camera (Requires rpicam-vid on host)
SOURCE = "tcp://127.0.0.1:5000"

# OPTION B: Use this for USB Webcam (uncomment line below)
# SOURCE = 0   
# ---------------------

print(f"Attempting to open video source: {SOURCE}")
cap = cv2.VideoCapture(SOURCE)

# If using USB Camera, force MJPG video for better compatibility
if SOURCE == 0:
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

if not cap.isOpened():
    print("Error: Could not open video source.")
    print("1. If using Pi Camera: Is 'rpicam-vid' running on the host?")
    print("2. If using USB Camera: Did you map /dev/video0?")
    exit()

print("Video Stream Started. Press 'q' to exit.")

try:
    while True:
        # 1. Capture frame-by-frame
        ret, frame = cap.read()

        if not ret:
            print("Failed to grab frame (Stream ended?)")
            break

        # 2. Display the resulting frame
        cv2.imshow('Docker Video Stream', frame)

        # 3. Wait for 'q' key to stop the loop
        # waitKey(1) is required for the image to render!
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    pass

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
print("Stream stopped.")
