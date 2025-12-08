import time
import numpy as np
import cv2
from picamera2 import Picamera2

# --- 1. Initialize Picamera2 ---
picam2 = Picamera2()
# We still set BGR888, but we will also manually flip it later since your setup requires it
config = picam2.create_video_configuration(main={"size": (640, 480), "format": "BGR888"})
picam2.configure(config)
picam2.start()

# --- Global Variables ---
selection = None       # Stores (x1, y1, x2, y2) of the mouse selection
track_window = None    # Stores (x, y, w, h) for the tracker
drag_start = None      # Stores (x, y) where mouse drag started
hist = None            # Stores the histogram of the object
show_backproj = False  # Flag to toggle the back projection window

# --- Helper Function: Display Histogram ---
def update_histogram_window(new_hist):
    bin_count = new_hist.shape[0]
    bin_w = 24
    img = np.zeros((256, bin_count * bin_w, 3), np.uint8)
    
    for i in range(bin_count):
        h = int(new_hist[i])
        cv2.rectangle(img, (i * bin_w + 2, 255), ((i + 1) * bin_w - 2, 255 - h), 
                      (int(180.0 * i / bin_count), 255, 255), -1)
    
    img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
    cv2.imshow('Object Histogram', img)

# --- Mouse Callback Function ---
def onmouse(event, x, y, flags, param):
    global selection, track_window, drag_start, hist
    
    if event == cv2.EVENT_LBUTTONDOWN:
        drag_start = (x, y)
        track_window = None
        selection = None
        hist = None 
        
    if drag_start:
        xmin = min(x, drag_start[0])
        ymin = min(y, drag_start[1])
        xmax = max(x, drag_start[0])
        ymax = max(y, drag_start[1])
        selection = (xmin, ymin, xmax, ymax)
        
    if event == cv2.EVENT_LBUTTONUP:
        drag_start = None
        if selection:
            xmin, ymin, xmax, ymax = selection
            w = xmax - xmin
            h = ymax - ymin
            if w > 0 and h > 0:
                track_window = (xmin, ymin, w, h)
            else:
                selection = None

# --- Setup Windows ---
cv2.namedWindow('Original Tracking')
cv2.setMouseCallback('Original Tracking', onmouse)

print("Status: Camera Started.")
print("  - Drag mouse on 'Original Tracking' window to select an object.")
print("  - Press 'b' to toggle Back Projection view.")
print("  - Press 'ESC' to quit.")

try:
    while True:
        # Capture frame 
        frame = picam2.capture_array()
        if frame is None:
            break

        # --- OPTION 2 FIX: Convert RGB to BGR ---
        # This fixes the Blue/Red swap issue
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        vis = frame.copy()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array((0., 60., 32.)), np.array((180., 255., 255.)))

        # 1. Selection Mode Drawing
        if selection:
            x0, y0, x1, y1 = selection
            cv2.rectangle(vis, (x0, y0), (x1, y1), (0, 255, 0), 2)

        # 2. Tracking Mode Logic
        if track_window and track_window[2] > 0 and track_window[3] > 0:
            
            # Calculate Histogram if needed
            if hist is None:
                x, y, w, h = track_window
                roi = hsv[y:y+h, x:x+w]
                roi_mask = mask[y:y+h, x:x+w]
                hist = cv2.calcHist([roi], [0], roi_mask, [16], [0, 180])
                cv2.normalize(hist, hist, 0, 255, cv2.NORM_MINMAX)
                hist = hist.reshape(-1)
                update_histogram_window(hist)

            # Back Projection Calculation
            prob = cv2.calcBackProject([hsv], [0], hist, [0, 180], 1)
            prob &= mask
            
            # CamShift Algorithm
            term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)
            track_box, track_window = cv2.CamShift(prob, track_window, term_crit)

            # Draw Tracking Result
            try:
                cv2.ellipse(vis, track_box, (0, 0, 255), 2)
            except:
                cv2.rectangle(vis, (track_window[0], track_window[1]), 
                              (track_window[0]+track_window[2], track_window[1]+track_window[3]), 
                              (0, 0, 255), 2)

            # --- HANDLE 'b' KEY (Back Projection View) ---
            if show_backproj:
                # Show probability map in a separate window
                cv2.imshow("Back Projection", prob)
            else:
                # If toggled off, ensure the window is closed
                try:
                    cv2.destroyWindow("Back Projection")
                except:
                    pass
        else:
            # If we aren't tracking, make sure backproj window is closed
            try:
                cv2.destroyWindow("Back Projection")
            except:
                pass

        cv2.imshow('Original Tracking', vis)

        k = cv2.waitKey(1) & 0xFF
        if k == 27: # ESC
            break
        if k == ord('b'):
            show_backproj = not show_backproj
            print(f"Back Projection: {'ON' if show_backproj else 'OFF'}")

except KeyboardInterrupt:
    pass
finally:
    picam2.stop()
    cv2.destroyAllWindows()
