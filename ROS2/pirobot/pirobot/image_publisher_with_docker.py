"""
This version captures the frames from video stream produced by using the
following command:
rpicam-vid -t 0 --inline --listen -o tcp://0.0.0.0:5000 --width 640 --height 480

"""
#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
import numpy as np
import sys


# --- CONFIGURATION ---
# OPTION A: Use this for Raspberry Pi 5 Camera (Requires rpicam-vid on host)
SOURCE = "tcp://127.0.0.1:5000"

# OPTION B: Use this for USB Webcam (uncomment line below)
# SOURCE = 0   
# ---------------------

def initialize_camera():
    #print(f"Attempting to open video source: {SOURCE}")
    cap = cv2.VideoCapture(SOURCE)

    # If using USB Camera, force MJPG video for better compatibility
    if SOURCE == 0:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

    if not cap.isOpened():
        print("Error: Could not open video source.")
        print("1. If using Pi Camera: Is 'rpicam-vid' running on the host?")
        print("2. If using USB Camera: Did you map /dev/video0?")
        exit()
    return cap

    


def cv2_to_imgmsg(cv_image):
    img_msg = Image()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = "bgr8"
    img_msg.is_bigendian = 0
    img_msg.data = cv_image.tobytes()
    img_msg.step = len(img_msg.data) // img_msg.height
    return img_msg

class ImagePublisher(Node):
    def __init__(self, device):
        super().__init__('image_publisher')
        # DEFINING QoS PROFILE (Crucial for video)
        # Best Effort = Fire and forget (Fastest)
        # Volatile = Don't save old messages for new subscribers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.publisher_ = self.create_publisher(
            Image,
            '/pirobot/camera',
            qos_profile)

        timer_period = 0.033 # 30 FPS
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.camera = device

    def timer_callback(self):
        ret, frame = self.camera.read()
        if not ret:
            self.get_logger('image_publisher').warn("Failed to grab frame (Stream ended?)")
            return
        #frame = cv2.rotate(frame, cv2.ROTATE_180)
        #cv2.imshow('publisher', frame)
        msg = cv2_to_imgmsg(frame)
        self.publisher_.publish(msg)
        #self.get_logger('image_publisher').info('Sending data of size: "%d" bytes' % sys.getsizeof(msg.data))
        if cv2.waitKey(1) & 0xFF == ord('q'):
            raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    camera = initialize_camera()
    image_publisher = ImagePublisher(camera)
    try:
        rclpy.spin(image_publisher)
    except SystemExit:
        rclpy.logging.get_logger().info('Quitting.Done!')
        cv2.destroyAllWindows()
    image_publisher.destroy_node()
    rclpy.shutdown()
    camera.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
    
    
        
