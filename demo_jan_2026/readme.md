# Demo Instructions

## Download the package in your home folder:
Open a terminal and run the following commands:
```
cd ~/
git pull https://github.com/swagatk/Raspi_codes.git
```
Navigate to the folder inside `~/Raspi_codes/demo`:
```
cd ~/Raspi_codes/demo
```
## Obstacle avoidance
Open a terminal and run the following commands:
```
cd avoid_obstacle
python3 ./avoid_obstacle.py
```
## Joystick Control
Activate joystick virtual environment
```
source ~/.virtualenvs/joystickenv/bin/activate
```
Now execute the code inside this virtual environment. 
```
(joystickenv) cd ~/Raspi_codes/demo/joystick
(joystickenv) python3 ./camjamedukit3.py 
```
Now use left joystick to control the robot. Press `Analog` button to exit. Make sure the gamepad is powered and USB dongle is connected to the RPi. 

Once done, type the following command to exit the virtual environment.
```
deactivate
```
## Computer vision using Mediapipe
Activate the mediapipe virtual environment for this experiment
```
source ~/.virtualenvs/mediapipevenv/bin/activate
```
Run execute the scripts present in `mediapipe` folder one by one as shown below:
```
(mediapipevenv) cd ~/Raspi_codes/demo/mediapipe
python3 ./face_detect.py
python3 ./gesture_recog.py
python3 ./object_detect.py
```
