# Camshift object tracking
# runs on Python 2.7
import numpy as np
import cv2
from picamera import PiCamera
from picamera.array import PiRGBArray
import time

# Configure the PiCamera
camera = PiCamera()
camera.resolution = (320,240)
camera.rotation = 180
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(320,240))

# Give time for the camera to stabilize
time.sleep(1)

selection = None
track_window = None
show_backproj = False
drag_start = None



def onmouse(event, x, y, flags, param):
    global selection, track_window, drag_start
    if event == cv2.EVENT_LBUTTONDOWN:
        drag_start = (x, y)
        track_window = None
    if drag_start:
        xmin = min(x, drag_start[0])
        ymin = min(y, drag_start[1])
        xmax = max(x, drag_start[0])
        ymax = max(y, drag_start[1])
        selection = (xmin, ymin, xmax, ymax)
    if event == cv2.EVENT_LBUTTONUP:
        drag_start = None
        track_window = (xmin, ymin, xmax - xmin, ymax - ymin)

def show_hist(hist):
    bin_count = hist.shape[0]
    bin_w = 24
    img = np.zeros((256, bin_count*bin_w, 3), np.uint8)
    for i in xrange(bin_count):
        h = int(hist[i])
        cv2.rectangle(img, (i*bin_w+2, 255), ((i+1)*bin_w-2, 255-h),\
                      (int(180.0*i/bin_count), 255, 255), -1)
    img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
    cv2.imshow('hist', img)


# Create a window to capture mouse events
cv2.namedWindow('camshift')
cv2.setMouseCallback('camshift', onmouse, 0)
cv2.namedWindow('Control')

# Process each frame from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    # grab each frame
    image = frame.array
    vis = image.copy()
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array((0., 60., 32.)), np.array((180.,255.,255.)))

    # use a mouse to select a region to track
    if selection:
        x0, y0, x1, y1 = selection
        hsv_roi = hsv[y0:y1, x0:x1]
        mask_roi = mask[y0:y1, x0:x1]
        hist = cv2.calcHist([hsv_roi], [0], mask_roi, [16], [0,180])
        cv2.normalize(hist, hist, 0, 255, cv2.NORM_MINMAX)
        hist = hist.reshape(-1)
        show_hist(hist)
        vis_roi = vis[y0:y1, x0:x1]
        cv2.bitwise_not(vis_roi, vis_roi)
        vis[mask == 0] = 0

    if track_window and track_window[2] > 0 and track_window[3] > 0:
        selection = None
        prob = cv2.calcBackProject([hsv], [0], hist, [0,180], 1)
        prob &= mask
        term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)
        track_box, track_window = cv2.CamShift(prob, track_window, term_crit)

        if show_backproj:
            vis[:] = prob[...,np.newaxis]
        try:
            cv2.ellipse(vis, track_box, (0, 0, 255), 2)
        except:
            print(track_box)

        pts = cv2.boxPoints(track_box)
        mp = np.mean(pts, axis=0)
        center = tuple(mp)
        l = np.linalg.norm(pts[0] - pts[1])
        h = np.linalg.norm(pts[0] - pts[3])
        area = l * h
        print(area)
        x,y,w,h = track_window
        print(w*h)
        
        ctrl_img = cv2.line(vis, center, (160,120), (255,0,0), 2)


 
    # Draw horizontal and vertical line passing through centre of the image

    
    cv2.imshow('camshift', vis)

    ctrl_img = cv2.line(vis, (0, 120), (320, 120), (255,0,0), 2)
    ctrl_img = cv2.line(vis, (160, 240), (160, 0), (255,0,0), 2)
    
    cv2.imshow('Control', ctrl_img)
    
    ch = cv2.waitKey(5)

    # refresh the buffer
    rawCapture.truncate(0)
    
    if ch == 27:
        break
    if ch == ord('b'):
        show_backproj = not show_backproj
        
cv2.destroyAllWindows()
