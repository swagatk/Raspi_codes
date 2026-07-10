#!/usr/bin/env python3

import importlib
import os
import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def load_serial_module():
    try:
        return importlib.import_module('serial')
    except ImportError:
        virtual_env = os.environ.get('VIRTUAL_ENV')
        if virtual_env:
            version = f'python{sys.version_info.major}.{sys.version_info.minor}'
            site_packages = os.path.join(virtual_env, 'lib', version, 'site-packages')
            if os.path.isdir(site_packages) and site_packages not in sys.path:
                sys.path.insert(0, site_packages)
                return importlib.import_module('serial')
        raise


class MotionSubscriber(Node):
    def __init__(self):
        super().__init__('motion_subscriber_node')

        self.declare_parameter('topic', '/pirobot2/cmd_vel')
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('serial_timeout', 0.1)
        self.declare_parameter('deadband', 0.02)
        self.declare_parameter('speed_1_max', 0.12)
        self.declare_parameter('speed_2_max', 0.22)
        self.declare_parameter('default_speed_command', '2')

        self.topic = str(self.get_parameter('topic').value)
        self.serial_port = str(self.get_parameter('serial_port').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.serial_timeout = float(self.get_parameter('serial_timeout').value)
        self.deadband = float(self.get_parameter('deadband').value)
        self.speed_1_max = float(self.get_parameter('speed_1_max').value)
        self.speed_2_max = float(self.get_parameter('speed_2_max').value)
        self.default_speed_command = str(self.get_parameter('default_speed_command').value)

        serial_module = load_serial_module()
        self.ser = serial_module.Serial(
            port=self.serial_port,
            baudrate=self.baudrate,
            timeout=self.serial_timeout,
        )

        self.current_motion_command = None
        self.current_speed_command = None

        self.subscription = self.create_subscription(Twist, self.topic, self.cmd_vel_callback, 20)
        self.get_logger().info(
            f'Listening on {self.topic} and sending commands to {self.serial_port} @ {self.baudrate}'
        )

        if self.default_speed_command in ('1', '2', '3'):
            self._send_serial(self.default_speed_command)
            self.current_speed_command = self.default_speed_command

    def _send_serial(self, payload):
        try:
            self.ser.write(payload.encode('ascii'))
            self.ser.flush()
        except Exception as exc:
            self.get_logger().error(f'Failed to write to serial: {exc}')

    def _select_speed_command(self, linear_x, angular_z):
        mag = max(abs(linear_x), abs(angular_z))
        if mag <= 0.0:
            return self.current_speed_command
        if mag <= self.speed_1_max:
            return '1'
        if mag <= self.speed_2_max:
            return '2'
        return '3'

    def _select_motion_command(self, linear_x, angular_z):
        if linear_x > self.deadband:
            return 'F'
        if linear_x < -self.deadband:
            return 'B'
        if angular_z > self.deadband:
            return 'L'
        if angular_z < -self.deadband:
            return 'R'
        return 'S'

    def cmd_vel_callback(self, msg):
        linear_x = float(msg.linear.x)
        angular_z = float(msg.angular.z)

        speed_command = self._select_speed_command(linear_x, angular_z)
        if speed_command in ('1', '2', '3') and speed_command != self.current_speed_command:
            self._send_serial(speed_command)
            self.current_speed_command = speed_command

        motion_command = self._select_motion_command(linear_x, angular_z)
        if motion_command != self.current_motion_command:
            self._send_serial(motion_command)
            self.current_motion_command = motion_command

    def destroy_node(self):
        try:
            self._send_serial('S')
        except Exception:
            pass
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MotionSubscriber()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ImportError as exc:
        print(
            'Python package "pyserial" is not available. Install command: '
            'python3 -m pip install pyserial'
        )
        print(exc)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
