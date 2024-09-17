# ROS2 Control Interface for Pirobot

## Dependencies:
* Tested on Raspberry Pi 4 with Raspberry Pi OS (Bookworm)
* Install ROS2 Iron 

## Installing ROS2 Iron
* Follow the steps provided [here](https://github.com/Ar-Ray-code/rpi-bullseye-ros2) to install ROS2/Iron on Bookworm

## Steps
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
* Build the package:
```
cd ~/ros2_ws/
colcon build --packages-select pirobot
```
* Once the build is completed without any errors. Source setup file to use the executables:
```
source install/local_setup.bash
```

* Run the following codes on differential terminals:
``
ros2 run pirobot keypub
ros2 run pirobot motionsub
ros2 run pirobot imagepub
ros2 run pirobot imagesub
`` 
`keypub` and `imagesub` can run on a remote computer and `motionsub` and `imagepub' should run on the pirobot. 

I use this code to control Pirobot created using [CamJamEdukit3](https://camjam.me/?page_id=1035). 
