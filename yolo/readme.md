# Using YoloV8 Object Detection with Picamera2 and USB Camera

## Installation

Create a folder to download yolo models and create new files
```
mkdir ~/yolo_project
cd ~/yolo_project
```
Now create a virtual environment inside the above folder
```
python3 -m venv yolov8env --system-site-packages
```
Activate new environment
```
source yolov8env/bin/activate
```
install ultralytics within this virtual environment:
```
pip install ultralytics
```
Make sure that the opencv is installed on the main system
```
sudo apt install opencv-python
```
`picamera2` library should be installed by default. If not, you can install using the following command:
```
sudo apt install python3-picamera2
```
If everything goes fine, you should be able to run the following command without any error:
```
yolo predict model=yolov8n.pt source='https://ultralytics.com/images/bus.jpg'
```
This will download a file 'yolov8n.pt' in the current folder and save the output image 
with bounding box at `~/yolo_project/runs/detect/predict/`.

You can download other models as well
```
yolo detect predict model=yolo11n.pt
```
This will also save the output of prediction on test images at `~/yolo_project/runs/detect/predict2/`

Now you can run the codes provided here with an active `yolov8env` environment.

To deactivate the environment, run the following command

```
deactivate
```

