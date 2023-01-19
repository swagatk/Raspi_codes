#!/usr/bin/env python
## Simple ROS image subscriber

import rospy
from sensor_msgs.msg import Image
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

def image_callback(img_msg):
    rospy.loginfo(rospy.get_caller_id() + ': Receiving Image data of size: %s', len(img_msg.data))

    cv2_img = imgmsg_to_cv2(img_msg)
    cv2.imshow('Subscriber', cv2_img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.destroyAllWindows()

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('imagesub', anonymous=False)

    rospy.Subscriber('/pirobot/camera', Image, image_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
