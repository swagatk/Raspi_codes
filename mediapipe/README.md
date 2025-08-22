# mediapipe examples with Picamera2

## Credits
Credit:[Google]( https://developers.googleblog.com/en/mediapipe-for-raspberry-pi-and-ios/)

## Examples
- Object Detection
- Gesture Recognition

## Dependencies
- Raspberry pi OS (bookworm)
- Mediapipe
- Opencv 4.x
- Pythoh 3.x

# Steps for installing

- Install opencv
```
sudo apt install python3-opencv opencv-data
```

- Create a virtual environment
```
python3 -m venv --system-site-packages ~/virtualenv
source ~/virtualenv/bin/activate
cd ~/virtualenv
```
- Install mediapipe inside the virtual environment
```
pip3 install mediapipe
```
- Download the repository
```
git clone https://github.com/swagatk/Raspi_codes.git
```
- Navigate the folder `~/virtualenv/Raspi_codes/mediapipe/examples` and execute the following commands:
``` 
sh setup.py
python3 face_detect.py
python3 gesture_recog.py
python3 object_detect.py
```
press 'q' on the image screen to exit. 

- Exit the virtual environment when finished
```
deactivate
```
