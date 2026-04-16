"""
Camera calibration script using OpenCV.
"""
import numpy as np
import cv2
import glob
import os

# --- CONFIGURATION ---
CHECKERBOARD = (8, 6) # Inner corners
SQUARE_SIZE = 0.028   # Size of a single square in meters (28mm = 0.028m)
VISUALIZE = True      # Set to True to visualize corner detection

# Get the directory where this script is located
script_dir = os.path.expanduser('~/')
image_folder = os.path.join(script_dir, 'calibration_images', '*.jpg')

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE


objpoints = [] # 3d point in real world space
imgpoints  = [] # 2d points in image plane.
images = glob.glob(image_folder)

print(f"Looking for images in: {image_folder}")
print(f"Found {len(images)} images. Processing...")
if VISUALIZE:
    print("Visualization enabled. Press any key to advance, 'q' to skip remaining.")

if len(images) == 0:
    print("Error: No calibration images found! Please capture images first using capture_images.py")
    exit(1)

gray = None
skip_visualization = False
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret == True:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
        print(f"  Found corners in: {os.path.basename(fname)}")
        
        # Draw and display the corners
        if VISUALIZE and not skip_visualization:
            img_with_corners = cv2.drawChessboardCorners(img.copy(), CHECKERBOARD, corners2, ret)
            cv2.putText(img_with_corners, f"FOUND - {os.path.basename(fname)}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("Corner Detection", img_with_corners)
            key = cv2.waitKey(0) & 0xFF
            if key == ord('q'):
                skip_visualization = True
    else:
        print(f"  No corners found in: {os.path.basename(fname)}")
        
        # Show the image without corners
        if VISUALIZE and not skip_visualization:
            cv2.putText(img, f"NO CORNERS - {os.path.basename(fname)}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.imshow("Corner Detection", img)
            key = cv2.waitKey(0) & 0xFF
            if key == ord('q'):
                skip_visualization = True

if VISUALIZE:
    cv2.destroyAllWindows()

if len(objpoints) == 0:
    print("Error: No valid chessboard patterns found in any image!")
    print(f"Make sure your checkerboard has {CHECKERBOARD[0]}x{CHECKERBOARD[1]} inner corners.")
    exit(1)

print(f"\nUsing {len(objpoints)} valid images for calibration...")
print("Calculating Camera Matrix...")
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

print("\n--- APRILTAG PARAMETERS (Copy these to your main script) ---")
print(f"camera_params = ({mtx[0,0]}, {mtx[1,1]}, {mtx[0,2]}, {mtx[1,2]})")
