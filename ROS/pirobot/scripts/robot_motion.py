#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import motor_control as mc

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ": %s", data.data)
    cmd_str = data.data
    mc.exec_cmd(cmd_str)



def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('chatter', String, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
