#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import recognise_direction_of_finger as dr

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        cmd_str = dr.fetch_command()  # Pass the captured frame to fetch_command()
        rospy.loginfo(cmd_str)
        pub.publish(cmd_str)  # Publish the direction as a ROS message
        rate.sleep()

if __name__ == '__main__':
    try:
        # initialize gesture recognition Interface
        dr.init()
        talker()
    except rospy.ROSInterruptException:
        dr.stop()
