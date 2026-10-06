import cv2
import pytesseract
from picamera2 import Picamera2
import numpy as np
import time

# 1. Initialize camera
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"format": "XBGR8888", "size": (1280, 720)})
picam2.configure(config)
picam2.start()

print("\n--- Live OCR Ready ---")
print("Align text in front of camera.")
print("Press [SPACE] to capture and read text.")
print("Press [Q] to quit.\n")

def preprocess_for_ocr(frame):
    """Convert frame to grayscale and apply adaptive thresholding to boost contrast."""
    if frame.ndim == 3 and frame.shape[2] == 4:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGRA2GRAY)
    else:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Remove high-frequency noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Adaptive thresholding handles uneven lighting across the paper
    thresh = cv2.adaptiveThreshold(
        blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2
    )
    return thresh

try:
    ocr_interval = 1.0
    next_ocr_time = 0.0
    last_extracted_text = None

    while True:
        # Capture raw frame directly into a NumPy array
        frame = picam2.capture_array()

        current_time = time.monotonic()
        if current_time >= next_ocr_time:
            processed_img = preprocess_for_ocr(frame)
            extracted_text = pytesseract.image_to_string(
                processed_img, config=r'--oem 3 --psm 6'
            ).strip()
            next_ocr_time = current_time + ocr_interval

            if extracted_text != last_extracted_text and extracted_text:
                print("\nRECOGNIZED TEXT:")
                print(extracted_text)
            last_extracted_text = extracted_text

        # Display camera stream
        cv2.imshow("Camera View (Press Space to OCR, Q to Exit)", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        if key == ord(' '):
            next_ocr_time = 0.0

finally:
    picam2.stop()
    cv2.destroyAllWindows()