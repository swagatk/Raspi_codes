#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import key_press as kp

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        cmd_str = kp.fetch_command()
        rospy.loginfo(cmd_str)
        pub.publish(cmd_str)
        rate.sleep()


if __name__ == '__main__':
    try:
        # initialize  Keyboard Interface
        kp.init()
        talker()
    except rospy.ROSInterruptException:
        kp.stop()
