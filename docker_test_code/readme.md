# Accessing RPI Hardware from Ubuntu Docker Container

## Pre-requisites
- Raspberry Pi OS 13 (Trixie)
- Docker
- ROS2 Jazzy on Ubuntu 24.04 Container

## Install docker
Install docker
```
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
```
Manage docker as non-root user
```
sudo groupadd docker
sudo usermod -aG docker $USER
```
Logout and log back in to see the effect

## Build ROS2 Jazzy docker image
Clone the ROS docker images git repo in your home directory:
```
git clone https://github.com/osrf/docker_images
```
Navigate to the folder `~/docker_images/ros/jazzy/ubuntu/noble/desktop`. 
Then run the following command:
```
docker build -t ros_docker .
```
Once completed, you can check the image built using the following command:
```
docker images
```
Create folder `docker` in your home directory. This will be used as the home directory inside the docker container:
```
mkdir ~/docker
```
Create a .bashrc file inside `~/docker` folder:
```
nano ~/docker/.bashrc
```
Add the following content
```
#---------------
# Force color prompt
force_color_prompt=yes
# Set a custom yellow prompt for Docker
if [ -n "$force_color_prompt" ]; then 
PS1='${debian_chroot:+($debian_chroot)}\[\033[01;33m\](ROS-DOCKER)\[\033[00m\] \u@\h:\w\$ ' 
else 
PS1='(ROS-DOCKER) \u@\h:\w\$ ' 
fi
# Source ROS 2 automatically (Optional)
source /opt/ros/jazzy/setup.bash
#----------
```
Press `Ctrl+x` and `y` to save and exit the file

Open a terminal and run the following command to allow docker to access host's xserver:
```
xhost +local:docker
```

## Run the docker container with the following command:
Run the following command in a separate terminal:
```
docker run -it --name=ros_jazzy --privileged --network host --user=$(id -u $USER):$(id -g $USER) --group-add video --group-add $(getent group gpio | cut -d: -f3) --env="DISPLAY" --workdir="/home/$USER" --volume="/home/pi/docker:/home/$USER" --volume="/etc/group:/etc/group:ro" --volume="/etc/passwd:/etc/passwd:ro" --volume="/etc/shadow:/etc/shadow:ro" --volume="/etc/sudoers.d:/etc/sudoers.d:ro" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --device=/dev/dri --group-add=$(getent group render | cut -d: -f3) --volume="/dev:/dev" ros_docker bash
```
This allows you to access hardware (GPIO pins, camera etc.) from the docker container. 

You should see something as shown below:
```
(ROS-DOCKER) pi@raspberrypi:~$ 
```

Test that you are able to see GUI by typing the following command:
```
ros2 run turtlesim turtlesim_node
```
## Accessing GPIO and Camera from Docker container:

Install some python packages inside the Ubuntu container: 
```
sudo apt install python3-venv
sudo apt install python3-opencv
sudo apt install v4l-utils
```
Create a virtual environment inside the docker container:
```
mkdir ~/.virtualenvs
cd ~/.virtualenvs
python3 -m venv gpio_env --system-site-packages
```
Activate the environment
```
source ~/virtualenvs/gpio_env/bin/activate
```
Install the following packages inside the virtual environment
```
(gpio_env) pip3 install rpi-lgpio
```
Now run the following command inside the `gpio_env` virtual environment

```
(gpio_env) (ROS-DOCKER) pi@raspberrypi:~/hardware_test$ python3 ./hardware_test.py 
```
You should see LED blinking if it is connected to the GPIO18 pin. 

### Accessing USB Camera
USB camera should start working straight way. Please run the following command:
```
python3 ./usb_camera_test.py
```

### Accessing PiCamera.
Run the following command on a separate terminal on the host raspbian OS:
```
# Run this on the Host (pi@raspberrypi)
rpicam-vid -t 0 --inline --listen -o tcp://0.0.0.0:5000 --width 640 --height 480
```
The above command starts streaming video on TCP port on the localhost which can now be accessed by a program running inside the docker container.

Run the following command inside the docker container (inside the virtual evironment)
```
(gpio_env) (ROS-DOCKER) pi@raspberrypi:~/hardware_test$ python3 ./picam_video.py 

```
You should be able to see video stream on an opencv window.
