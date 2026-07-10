#!/usr/bin/env python3

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard_node')

        self.declare_parameter('topic', '/pirobot2/cmd_vel')
        self.declare_parameter('speed_1', 0.10)
        self.declare_parameter('speed_2', 0.20)
        self.declare_parameter('speed_3', 0.30)
        self.declare_parameter('angular_scale', 1.0)
        self.declare_parameter('idle_timeout_sec', 0.4)

        self.topic = str(self.get_parameter('topic').value)
        self.speed_map = {
            '1': float(self.get_parameter('speed_1').value),
            '2': float(self.get_parameter('speed_2').value),
            '3': float(self.get_parameter('speed_3').value),
        }
        self.angular_scale = float(self.get_parameter('angular_scale').value)
        self.idle_timeout_sec = float(self.get_parameter('idle_timeout_sec').value)

        self.current_speed_key = '2'
        self.current_speed = self.speed_map[self.current_speed_key]
        self.last_motion_time = self.get_clock().now()
        self.sent_stop = False

        self.publisher = self.create_publisher(Twist, self.topic, 20)

        self.get_logger().info(
            'Keyboard control: w=forward, s=backward, a=left, b=right, 1/2/3=speed, '
            'space=stop, q=quit'
        )
        self.get_logger().info(
            f'Publishing Twist to {self.topic}. Default speed profile: {self.current_speed_key} ({self.current_speed:.2f} m/s)'
        )

    def _publish_twist(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.publisher.publish(msg)

    def _apply_motion_key(self, key):
        if key == 'w':
            self._publish_twist(self.current_speed, 0.0)
            self.sent_stop = False
            return True
        if key == 's':
            self._publish_twist(-self.current_speed, 0.0)
            self.sent_stop = False
            return True
        if key == 'a':
            self._publish_twist(0.0, self.current_speed * self.angular_scale)
            self.sent_stop = False
            return True
        if key == 'b':
            self._publish_twist(0.0, -self.current_speed * self.angular_scale)
            self.sent_stop = False
            return True
        if key == ' ':
            self._publish_twist(0.0, 0.0)
            self.sent_stop = True
            return True
        return False

    def _apply_speed_key(self, key):
        if key not in self.speed_map:
            return False
        self.current_speed_key = key
        self.current_speed = self.speed_map[key]
        self.get_logger().info(
            f'Speed set to profile {key}: {self.current_speed:.2f} m/s'
        )
        return True


def read_key(timeout_sec=0.05):
    readable, _, _ = select.select([sys.stdin], [], [], timeout_sec)
    if readable:
        return sys.stdin.read(1)
    return None


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()

    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    try:
        while rclpy.ok():
            key = read_key()
            now = node.get_clock().now()

            if key == 'q':
                node._publish_twist(0.0, 0.0)
                break

            key_handled = False
            if key is not None:
                key_handled = node._apply_speed_key(key)
                if not key_handled:
                    key_handled = node._apply_motion_key(key)

                if key_handled and key in ('w', 's', 'a', 'b'):
                    node.last_motion_time = now

            elapsed = (now - node.last_motion_time).nanoseconds / 1e9
            if elapsed >= node.idle_timeout_sec and not node.sent_stop:
                node._publish_twist(0.0, 0.0)
                node.sent_stop = True

            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_twist(0.0, 0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
