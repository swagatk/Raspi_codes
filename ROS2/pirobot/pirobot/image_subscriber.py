#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
import numpy as np
import sys

def imgmsg_to_cv2(img_msg):

    if img_msg.encoding != "bgr8":
        rospy.logerr("This node is hardcoded to 'bgr8' encoding.")
    dtype = np.dtype("uint8")   # hardcode to 8 bits
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3),
            dtype = dtype, buffer=img_msg.data)
    #if the byte order is different between the message and the system
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()
    return image_opencv


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        # QoS Must Match Publisher
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.subscription = self.create_subscription(
            Image,
            '/pirobot/camera',
            self.listener_callback,
            qos_profile) 
            
        #self.subscription

    def listener_callback(self, msg):
        #self.get_logger('image_subscriber').info('Received data of size: "%s" bytes' % sys.getsizeof(msg.data))
        cv2_img = imgmsg_to_cv2(msg)
        cv2.imshow("Subscriber", cv2_img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    try:
        rclpy.spin(image_subscriber)
    except SystemExit:
        rclpy.logging.get_logger('image_subscriber').info('Quitting. Done!')
        cv2.destroyAllWindows()
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
