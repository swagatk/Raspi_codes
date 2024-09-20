# ROS2 Control Interface for Pirobot

## Dependencies:
* Tested on Raspberry Pi 4 with Raspberry Pi OS (Bookworm)
* Install ROS2 Iron 
* OpenCV 4.x

## Capabilities
* Teleoperation using arrow keys
* Viewing Picamera video from remote machine
* Service to start/stop autonomous navigation while avoiding obstacle

## Installing ROS2 Iron
* Follow the steps provided [here](https://github.com/Ar-Ray-code/rpi-bullseye-ros2) to install ROS2/Iron on Bookworm

## Installing Opencv
```
sudo apt install python3-opencv opencv-data
```
Make sure that you are able to `import cv2` inside a python console successfully
```
python3
Python 3.11.2 (main, Aug 26 2024, 07:20:54) [GCC 12.2.0] on linux
Type "help", "copyright", "credits" or "license" for more information.
>>> import cv2
>>> cv2.__version__
'4.6.0'
>>> 

```
## Steps to build your package
* Download the this repo in your home directory:
```
git clone https://github.com/swagatk/Raspi_codes.git
```

* Create an overlay workspace for you to build:
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src/
ros2 pkg create --build-type ament_python --license Apache-2.0 pirobot
```
* Copy the folder `Raspi_codes/ROS2/pirobot/` to `~/ros2_ws/src/pirobot/`.
```
copy -r ~/Raspi_codes/ROS2/pirobot ~/ros2_ws/src/pirobot/
```
* Build the package:
```
cd ~/ros2_ws/
colcon build --packages-select pirobot
```
* Once the build is completed without any errors. Source setup file to use the executables:
```
source install/local_setup.bash
```

* Run the following codes on different terminals:
```
ros2 run pirobot keypub
ros2 run pirobot motionsub
ros2 run pirobot imagepub
ros2 run pirobot imagesub
``` 

`keypub` and `imagesub` can run on a remote computer and `motionsub` and `imagepub` should run on the pirobot. 

* To run the avoid obstacle service

run the folllowing command on the pirobot:
```
ros2 run pirobot aoservice
```
run the following command on the remote machine with last term being one among the list ['True', 'true', '1', 'yes'] etc.
```
ros2 run pirobot aoclient True
```
For instance, to stop the robot motion, type:
```
ros2 run pirobot aoclient stop
```
I use this code to control Pirobot created using [CamJamEdukit3](https://camjam.me/?page_id=1035). 
