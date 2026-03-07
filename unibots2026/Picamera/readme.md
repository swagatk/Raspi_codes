# AprilTag Localization & Navigation Guide

We have standardized the resolution to 640x480 at 30 FPS. This is the mathematical "sweet spot" that provides enough pixels to see Ping Pong balls and AprilTags at 2 meters, without overloading your CPU threads.

## Software Requirements

Open your Raspberry Pi terminal and install the required vision libraries:

```
sudo apt update
sudo apt install python3-opencv
pip3 install pupil-apriltags numpy
```

(Note: `pupil-apriltags` is the fastest, most reliable Python library for AprilTags, heavily optimized for low-power ARM CPUs).


## Camera Calibration (The 640x480 Setup)

For AprilTags to give you physical distances in real-world meters, you must calibrate your PiCamera V3 at the exact resolution you will use for driving.

### Step 1: Print the Checkerboard

1. [Download](https://markhedleyjones.com/projects/calibration-checkerboard-collection) and print a standard OpenCV checkerboard on A4 size paper (9x7 squares, 30mm square size gives 8x6 inner corners).

2. Attach it to a completely flat, hard clipboard.

3. Measure the exact width of a single printed black square with a ruler (e.g., 25mm or 0.025m).

### Step 2: Capture calibration images

Use the file `capture_images.py` to capture images using the Picamera. Run the file on a terminal using the following command: 


```
python3 capture_images.py
```
Move the checkerboard around and press SPACE to take ~20 pictures from different angles, tilts, and distances.


### Step 3: Calculate the Camera Matrix

Use the file `calibrate.py` to calculate the camera matrix parameters. Update the SQUARE_SIZE to match your real-world measurement!

How to run it: 
```
python3 calibrate.py
```
Save the four numbers it prints out at the end.

## Tag Pose detection

Run the file `tag_pose_detection.py` to detect the tag pose include z-distance and x-offset.  The code includes the 640x480 forced resolution, Grayscale conversion, and the quad_decimate speed hack to ensure it leaves plenty of CPU power for your other parallel threads.

```
python3 tag_pose_detection.py
``` 

## Using Apriltag for navigation

TODO




