#!/usr/bin/env python3

import atexit
import select
import signal
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
            'Keyboard control: w=forward, s=backward, a=left, d=right (b also works), 1/2/3=speed, '
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
        if key in ('d', 'b'):
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

    keyboard_stream = sys.stdin
    keyboard_file = None
    old_settings = None
    restored = False
    previous_sigint = None
    previous_sigterm = None

    def restore_terminal():
        nonlocal restored
        if restored:
            return
        if old_settings is not None:
            try:
                termios.tcsetattr(keyboard_stream, termios.TCSADRAIN, old_settings)
            except Exception:
                pass
        if keyboard_file is not None:
            try:
                keyboard_file.close()
            except Exception:
                pass
        restored = True

    def on_signal(signum, _frame):
        restore_terminal()
        raise KeyboardInterrupt()

    # ros2 launch may not attach a TTY to stdin; use the controlling terminal directly.
    if not keyboard_stream.isatty():
        try:
            keyboard_file = open('/dev/tty', 'r')
            keyboard_stream = keyboard_file
        except OSError:
            node.get_logger().error(
                'Keyboard teleop requires a TTY. Run this node from an interactive terminal.'
            )
            node.destroy_node()
            rclpy.shutdown()
            return

    old_settings = termios.tcgetattr(keyboard_stream)
    tty.setcbreak(keyboard_stream.fileno())
    atexit.register(restore_terminal)
    previous_sigint = signal.getsignal(signal.SIGINT)
    previous_sigterm = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    try:
        while rclpy.ok():
            key = None
            readable, _, _ = select.select([keyboard_stream], [], [], 0.05)
            if readable:
                key = keyboard_stream.read(1)
            now = node.get_clock().now()

            if key == 'q':
                node._publish_twist(0.0, 0.0)
                break

            key_handled = False
            if key is not None:
                key_handled = node._apply_speed_key(key)
                if not key_handled:
                    key_handled = node._apply_motion_key(key)

                if key_handled and key in ('w', 's', 'a', 'd', 'b'):
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
        restore_terminal()
        if previous_sigint is not None:
            signal.signal(signal.SIGINT, previous_sigint)
        if previous_sigterm is not None:
            signal.signal(signal.SIGTERM, previous_sigterm)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
