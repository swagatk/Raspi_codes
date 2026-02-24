import cv2
import mediapipe as mp
import time

# Initialize MediaPipe Face Detection
mp_face_detection = mp.solutions.face_detection
mp_drawing = mp.solutions.drawing_utils

# Use specific model selection (0 for short range, 1 for long range)
face_detection = mp_face_detection.FaceDetection(model_selection=0, min_detection_confidence=0.5)

print("Starting Camera...")
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Warmup delay
time.sleep(2)

if not cap.isOpened():
    print("Camera failed to open.")
    exit()

print("Camera running. Press 'q' to quit.")

while cap.isOpened():
    success, image = cap.read()
    if not success:
        print("Ignoring empty camera frame.")
        continue

    # To improve performance, optionally mark the image as not writeable to
    # pass by reference.
    image.flags.writeable = False
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    
    # --- MEDIAPIPE PROCESS ---
    # This is the line that might crash if XNNPACK fails
    try:
        results = face_detection.process(image)
    except Exception as e:
        print(f"MediaPipe Crash: {e}")
        break
    # -------------------------

    # Draw the face detection annotations on the image.
    image.flags.writeable = True
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    
    if results.detections:
        for detection in results.detections:
            mp_drawing.draw_detection(image, detection)

    cv2.imshow('MediaPipe Face Detection', image)
    if cv2.waitKey(5) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
