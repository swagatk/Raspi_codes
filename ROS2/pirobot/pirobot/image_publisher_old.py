#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from picamera2 import Picamera2
from libcamera import Transform
import sys

# Initialize and Configure PiCamera
picam2 = Picamera2()
picam2.preview_configuration.size=(320,240)
picam2.preview_configuration.format = "RGB888"
picam2.preview_configuration.transform = Transform(hflip=0, vflip=1)
picam2.configure("preview")
picam2.start()


def cv2_to_imgmsg(cv_image):
    img_msg = Image()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = "bgr8"
    img_msg.is_bigendian = 0
    img_msg.data = cv_image.tostring()
    img_msg.step = len(img_msg.data) // img_msg.height
    return img_msg

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(
            Image,
            '/pirobot/camera',
            10)
        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        frame = picam2.capture_array()
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        #cv2.imshow('publisher', frame)
        msg = cv2_to_imgmsg(frame)
        self.publisher_.publish(msg)
        self.get_logger().info('Sending data of size: "%d" bytes' % sys.getsizeof(msg.data))
        if cv2.waitKey(1) & 0xFF == ord('q'):
            raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    try:
        rclpy.spin(image_publisher)
    except SystemExit:
        rclpy.logging.get_logger().info('Quitting.Done!')
        cv2.destroyAllWindows()
    image_publisher.destroy_node()
    rclpy.shutdown()
    picam2.stop()
    picam2.close()

if __name__ == '__main__':
    main()
    
    
        
