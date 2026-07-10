#!/usr/bin/env python3

import importlib
import math
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def load_rplidar_class():
    try:
        return importlib.import_module('rplidar').RPLidar
    except ImportError:
        virtual_env = os.environ.get('VIRTUAL_ENV')
        if virtual_env:
            version = f'python{sys.version_info.major}.{sys.version_info.minor}'
            site_packages = os.path.join(virtual_env, 'lib', version, 'site-packages')
            if os.path.isdir(site_packages) and site_packages not in sys.path:
                sys.path.insert(0, site_packages)
                return importlib.import_module('rplidar').RPLidar
        raise


class LidarPublisher(Node):
    def __init__(self):
        super().__init__('lidar_publisher_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'laser')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('scan_resolution', 360)
        self.declare_parameter('range_min_m', 0.15)
        self.declare_parameter('range_max_m', 16.0)

        self.port = self.get_parameter('port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.scan_resolution = int(self.get_parameter('scan_resolution').value)
        self.range_min_m = float(self.get_parameter('range_min_m').value)
        self.range_max_m = float(self.get_parameter('range_max_m').value)

        self.publisher = self.create_publisher(LaserScan, self.scan_topic, qos_profile_sensor_data)
        self.stop_event = threading.Event()
        self.lidar = None
        self.last_scan_time = None

        try:
            self.rplidar_class = load_rplidar_class()
        except ImportError as exc:
            raise RuntimeError(
                'Python package "rplidar" is not available for this interpreter. '
                f'Interpreter: {sys.executable}. '
                'If using a virtualenv, activate it before launch or rebuild after activation. '
                'Install command: python3 -m pip install rplidar'
            ) from exc

        self.reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.reader_thread.start()

    def _read_loop(self):
        try:
            self.lidar = self.rplidar_class(self.port, baudrate=self.baudrate)
            self.lidar.start_motor()
            time.sleep(0.5)

            info = self.lidar.get_info()
            health = self.lidar.get_health()
            self.get_logger().info(
                f'Connected to {self.port} | model={info.get("model")} firmware={info.get("firmware")} '
                f'hardware={info.get("hardware")} health={health}'
            )

            for scan in self.lidar.iter_scans():
                if self.stop_event.is_set():
                    break
                msg = self._scan_to_message(scan)
                self.publisher.publish(msg)
        except Exception as exc:
            self.get_logger().error(f'RPLidar read loop stopped: {exc}')
        finally:
            self._shutdown_lidar()

    def _scan_to_message(self, scan):
        point_count = max(1, self.scan_resolution)
        angle_increment = (2.0 * math.pi) / point_count
        ranges = [float('inf')] * point_count
        intensities = [0.0] * point_count

        for quality, angle_deg, distance_mm in scan:
            if distance_mm <= 0:
                continue

            distance_m = distance_mm / 1000.0
            if distance_m < self.range_min_m or distance_m > self.range_max_m:
                continue

            wrapped_angle_deg = angle_deg % 360.0
            index = min(point_count - 1, int(wrapped_angle_deg / 360.0 * point_count))
            current_range = ranges[index]

            if math.isinf(current_range) or distance_m < current_range:
                ranges[index] = distance_m
                intensities[index] = float(quality)

        now = self.get_clock().now().to_msg()
        monotonic_now = time.monotonic()
        if self.last_scan_time is None:
            scan_time = 0.0
        else:
            scan_time = monotonic_now - self.last_scan_time
        self.last_scan_time = monotonic_now

        msg = LaserScan()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi - angle_increment
        msg.angle_increment = angle_increment
        msg.time_increment = scan_time / point_count if scan_time > 0.0 else 0.0
        msg.scan_time = scan_time
        msg.range_min = self.range_min_m
        msg.range_max = self.range_max_m
        msg.ranges = ranges
        msg.intensities = intensities
        return msg

    def _shutdown_lidar(self):
        if self.lidar is None:
            return
        try:
            self.lidar.stop()
        except Exception:
            pass
        try:
            self.lidar.stop_motor()
        except Exception:
            pass
        try:
            self.lidar.disconnect()
        except Exception:
            pass
        self.lidar = None

    def destroy_node(self):
        self.stop_event.set()
        if self.reader_thread.is_alive():
            self.reader_thread.join(timeout=2.0)
        self._shutdown_lidar()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LidarPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        print(exc)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()