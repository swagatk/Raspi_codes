#!/usr/bin/env python3

import importlib
import os
import sys

import rclpy
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


class UltrasonicPublisher(Node):
    def __init__(self):
        super().__init__('ultrasonic_publisher_node')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('serial_timeout', 0.1)
        self.declare_parameter('topic', '/pirobot2/ultrasonic')
        self.declare_parameter('poll_hz', 50.0)

        self.serial_port = str(self.get_parameter('serial_port').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.serial_timeout = float(self.get_parameter('serial_timeout').value)
        self.topic = str(self.get_parameter('topic').value)
        self.poll_hz = float(self.get_parameter('poll_hz').value)

        serial_module = load_serial_module()
        self.ser = serial_module.Serial(
            port=self.serial_port,
            baudrate=self.baudrate,
            timeout=self.serial_timeout,
        )

        self.publisher = self.create_publisher(Int32MultiArray, self.topic, 20)
        self.publish_count = 0

        self.timer = self.create_timer(1.0 / self.poll_hz, self.timer_callback)
        self.get_logger().info(
            f'Reading ultrasonic values from {self.serial_port} and publishing to {self.topic}'
        )

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

    def timer_callback(self):
        try:
            while self.ser.in_waiting > 0:
                raw = self.ser.readline().decode('ascii', errors='ignore').strip()
                if not raw:
                    continue

                distances = self._parse_distance_line(raw)
                if distances is None:
                    continue

                msg = Int32MultiArray()
                msg.data = distances
                self.publisher.publish(msg)

                self.publish_count += 1
                if self.publish_count % 50 == 0:
                    self.get_logger().info(
                        f'ultrasonic cm [left, center, right] = {distances}'
                    )
        except Exception as exc:
            self.get_logger().error(f'Serial read failed: {exc}')

    def destroy_node(self):
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = UltrasonicPublisher()
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
