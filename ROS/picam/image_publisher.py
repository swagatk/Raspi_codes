#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
print("Camera Opened Successfully: ", cap.isOpened())
#bridge = CvBridge()

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_FPS, 30)


def cv2_to_imgmsg(cv_image):
    img_msg = Image()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = "bgr8"
    img_msg.is_bigendian = 0
    img_msg.data = cv_image.tostring()
    img_msg.step = len(img_msg.data) // img_msg.height
    return img_msg
                    
    

def talker():
    pub = rospy.Publisher('/pirobot/camera', Image, queue_size=1)
    rospy.init_node('imagepub', anonymous=False)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        # roate the frame. it is inverted in my case
        rframe = cv2.rotate(frame, cv2.ROTATE_180)
        if not ret:
            break
        msg = cv2_to_imgmsg(rframe)
        #msg = np.frombuffer(rframe.data, dtype=np.uint8).reshape(rframe.height, rframe.width, -1)
        pub.publish(msg)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        if rospy.is_shutdown():
            cap.release()
            #cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        cap.release()
