"""
Controlling robot by detecting hand poses
"""
#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pirobot import recognise_finger_direction as rfd
import sys

class HandRecognPublisher(Node):
    def __init__(self):
        super().__init__('handrecog_publisher')
        self.publisher_ = self.create_publisher(
            String,
            '/pirobot/command',
            10)
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = rfd.fetch_command()
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        if msg.data == 'EXIT':
            raise SystemExit

def main(args=None):
    rfd.init()
    rclpy.init(args=args)
    handpub = HandRecognPublisher()
    try:
        rclpy.spin(handpub)
    except SystemExit:
        rfd.stop()
    handpub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
