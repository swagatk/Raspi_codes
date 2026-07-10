#!/usr/bin/env python3

import importlib
import os
import sys
from threading import Lock

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


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


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        self.declare_parameter('cmd_vel_topic', '/pirobot2/cmd_vel')
        self.declare_parameter('ultrasonic_topic', '/pirobot2/ultrasonic')
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('serial_timeout', 0.1)
        self.declare_parameter('poll_hz', 50.0)
        self.declare_parameter('command_keepalive_sec', 0.5)
        self.declare_parameter('deadband', 0.02)
        self.declare_parameter('speed_1_max', 0.12)
        self.declare_parameter('speed_2_max', 0.22)
        self.declare_parameter('default_speed_command', 2)

        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.ultrasonic_topic = str(self.get_parameter('ultrasonic_topic').value)
        self.serial_port = str(self.get_parameter('serial_port').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.serial_timeout = float(self.get_parameter('serial_timeout').value)
        self.poll_hz = float(self.get_parameter('poll_hz').value)
        self.command_keepalive_sec = float(self.get_parameter('command_keepalive_sec').value)
        self.deadband = float(self.get_parameter('deadband').value)
        self.speed_1_max = float(self.get_parameter('speed_1_max').value)
        self.speed_2_max = float(self.get_parameter('speed_2_max').value)
        self.default_speed_command = str(int(self.get_parameter('default_speed_command').value))

        serial_module = load_serial_module()
        self.ser = serial_module.Serial(
            port=self.serial_port,
            baudrate=self.baudrate,
            timeout=self.serial_timeout,
        )
        self.serial_lock = Lock()

        self.current_motion_command = None
        self.current_speed_command = None
        self.ultrasonic_publish_count = 0

        self.cmd_vel_sub = self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_callback, 20)
        self.ultrasonic_pub = self.create_publisher(Int32MultiArray, self.ultrasonic_topic, 20)
        self.poll_timer = self.create_timer(1.0 / self.poll_hz, self.poll_serial)
        self.keepalive_timer = self.create_timer(self.command_keepalive_sec, self.send_keepalive)

        if self.default_speed_command in ('1', '2', '3'):
            self._send_serial(self.default_speed_command)
            self.current_speed_command = self.default_speed_command

        self.get_logger().info(
            f'Serial bridge active on {self.serial_port} @ {self.baudrate}. '
            f'cmd_vel={self.cmd_vel_topic}, ultrasonic={self.ultrasonic_topic}'
        )

    def _send_serial(self, payload):
        try:
            with self.serial_lock:
                self.ser.write(payload.encode('ascii'))
                self.ser.flush()
        except Exception as exc:
            self.get_logger().error(f'Failed to write serial payload {payload!r}: {exc}')

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

    def _parse_distance_line(self, line):
        # Arduino format: D,left,center,right
        if not line.startswith('D,'):
            return None

        parts = line.split(',')
        if len(parts) != 4:
            return None

        try:
            left = int(parts[1])
            center = int(parts[2])
            right = int(parts[3])
        except ValueError:
            return None

        return [left, center, right]

    def send_keepalive(self):
        if self.current_motion_command is None:
            return
        self._send_serial(self.current_motion_command)

    def poll_serial(self):
        try:
            with self.serial_lock:
                while self.ser.in_waiting > 0:
                    raw = self.ser.readline().decode('ascii', errors='ignore').strip()
                    if not raw:
                        continue

                    distances = self._parse_distance_line(raw)
                    if distances is None:
                        continue

                    msg = Int32MultiArray()
                    msg.data = distances
                    self.ultrasonic_pub.publish(msg)

                    self.ultrasonic_publish_count += 1
                    if self.ultrasonic_publish_count % 50 == 0:
                        self.get_logger().info(
                            f'ultrasonic cm [left, center, right] = {distances}'
                        )
        except Exception as exc:
            self.get_logger().error(f'Serial poll failed: {exc}')

    def destroy_node(self):
        try:
            self._send_serial('S')
        except Exception:
            pass
        try:
            with self.serial_lock:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SerialBridge()
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
