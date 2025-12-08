#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
import numpy as np
import sys

def imgmsg_to_cv2(img_msg):
    dtype = np.dtype("uint8")
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3),
            dtype=dtype, buffer=img_msg.data)
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()
    return image_opencv

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        
        # USE BEST_EFFORT (Fire and Forget) to prevent network blocking
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
        
        self.current_frame = None

    def listener_callback(self, msg):
        # Only decode the image here, don't display it yet (keeps callback fast)
        self.current_frame = imgmsg_to_cv2(msg)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    
    # MAIN LOOP
    # Instead of rclpy.spin(), we loop manually to handle OpenCV better
    try:
        while rclpy.ok():
            # Process any waiting messages (non-blocking)
            rclpy.spin_once(image_subscriber, timeout_sec=0.001)
            
            # Display the frame if we have one
            if image_subscriber.current_frame is not None:
                cv2.imshow("Subscriber", image_subscriber.current_frame)
                
                # Clear it so we don't re-display the same frame unnecessarily
                # (Optional, depends on preference)
                # image_subscriber.current_frame = None 
            
            # Check for Quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    except KeyboardInterrupt:
        pass
    finally:
        image_subscriber.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
