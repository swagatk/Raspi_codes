# RPI ROS Robot

## Install ROS Noetic on Raspbian Buster 
https://varhowto.com/install-ros-noetic-raspberry-pi-4/
## Install OpenCV 4.x on Raspbian Buster
* For RPi 4, follow instruction available at this site:

https://qengineering.eu/install-opencv-4.5-on-raspberry-pi-4.html


* For RPI 3, we need to install the available debian package (Also
    works for RPI 4)

`$ sudo apt-get install python-opencv python3-opencv`


## Installation Instruction
Clone this github repo in your home directory: 
`$ git clone https://github.com/swagatk/Raspi_codes.git`

It will create a folder called ‘Raspi_codes’ in your home directory.


Create catkin workspace in your home folder:
```
$ mkdir ~/catkin_ws/
$ cd ~/catkin_ws/
$ catkin_make
$ source devel/setup.bash
```

Create a catkin package `pirobot` inside your `catkin_ws` folder:
```
$ cd catkin_make/src/
$ catkin_create_pkg pirobot rospy roscpp std_msgs sensor_msgs
$ cd ~/catkin_ws/
$ catkin_make
$ source devel/setup.bash
```

Copy the files from GIT repo to the `pirobot` package:
```
$ roscd pirobot
$ cp -r ~/Raspi_codes/ROS/pirobot/scripts/ ./
$ cp -r ~/Raspi_codes/ROS/pirobot/launch/ ./ 
$ cd ~/catkin_ws/
$ catkin_make
$ source ./devel/setup.bash
```

The package is to be built both on the 'pirobot' and on a 'remote'
machine.  The command to execute the nodes are as follows:

On remote machine:
```
$ roslaunch pirobot remote.launch

```
On pirobot:

```
$ roslaunch pirobot pirobot.launch

```

Please refer to the presentation present in `doc` folder for more details on how to run various nodes. 
