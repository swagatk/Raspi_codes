# Using YoloV8 Object Detection with Picamera2 and USB Camera

The code provided here supports both picamera and usb webcams.

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

## Selecting right Camera Device Index for USB camera

It is important to provide the right camera index for usb camera. You can know this by using the following command:
```
sudo apt install v4l-utils
```
```
v4l2-ctl --list-device
```
It will generate an output similar to the following:
```
rpi-hevc-dec (platform:rpi-hevc-dec):
	/dev/video19
	/dev/media1

HD 720P webcam: HD 720P webcam (usb-0000:01:00.0-1.4):
	/dev/video0
	/dev/video1
	/dev/media4

bcm2835-codec (vchiq:bcm2835-codec):
	/dev/media5
```
You should choose 0, 1 or 4 as the camera index in the file `usbcam_yolo.py`. 

## NCNN format
NCNN format is optimized for ARM architectures and provide faster inference compared to pytorch `.pt` models. You can convert a pytorch model into NCNN model by using the following command:

```
yolo export model=yolov8n.pt format=ncnn imgsz=320
```
It creates a folder named `yolov8n_ncnn_model`. 

When you are using ncnn model in one of the files above, please replace the following line

```
model = YOLO("/home/pi/yolo_project/yolov8n.pt")
```

to 
```
model = YOLO("/home/pi/yolo_project/yolov8n_ncnn_model")
```
Make sure that the `imgsz=320` matches with the image size of the captured frames.  I observed that on both cameras, the detection speed increases from 2 FPS to 7 FPS by using NCNN models.


## Speeding up with frame skip
we can increase the speed by skipping frames between detection. This is controlled by the user-defined parameter `SKIP_FRAMES`. With NCNN combined with `SKIP_FRAMES=3, it is possible to achieve FPS of about 50 (in picam) and about 100-120 for image size of 320x240. Last detected frames are displayed on the images until the new ones are found. The following two files implement this strategy:

* `usbcam_yolo_fs.py`
* `picam_yolo_fs.py`

## Numpy related errors
You may get numpy related errors as Ultralytics installed NumPy version 2.0+, but the Raspberry Pi system library (simplejpeg, used by picamera2) was built for NumPy 1.x. They are not compatible.

Uninstall numpy in your virtualenv:
```
pip uninstall numpy -y
```
Install compatible version
```
python3 ./picam_yolo_fs.py
```
