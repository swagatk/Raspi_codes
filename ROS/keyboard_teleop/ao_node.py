#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from avoid_obstacle import ao_main
def move():
    pub = rospy.Publisher('status', String, queue_size=10)
    rospy.init_node('avoid_obstacle', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = ao_main()
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        rospy.loginfo('Keyboard Interrupt')
