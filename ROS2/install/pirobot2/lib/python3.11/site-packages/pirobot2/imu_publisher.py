#!/usr/bin/env python3

import importlib
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

ACCEL_SCALE_LSB_PER_G = 16384.0
GYRO_SCALE_LSB_PER_DPS = 131.0
GRAVITY = 9.80665


def load_smbus_class():
    for module_name in ('smbus2', 'smbus'):
        try:
            return importlib.import_module(module_name).SMBus
        except ImportError:
            continue

    virtual_env = os.environ.get('VIRTUAL_ENV')
    if virtual_env:
        version = f'python{sys.version_info.major}.{sys.version_info.minor}'
        site_packages = os.path.join(virtual_env, 'lib', version, 'site-packages')
        if os.path.isdir(site_packages) and site_packages not in sys.path:
            sys.path.insert(0, site_packages)
            for module_name in ('smbus2', 'smbus'):
                try:
                    return importlib.import_module(module_name).SMBus
                except ImportError:
                    continue

    raise ImportError(
        'Neither smbus2 nor smbus is available. Install one of them, for example: '
        'python3 -m pip install smbus2'
    )


class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher_node')

        self.declare_parameter('bus_num', 1)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('topic', '/imu/data_raw')
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('accel_alpha', 0.25)
        self.declare_parameter('gyro_alpha', 0.20)
        self.declare_parameter('complementary_alpha', 0.98)
        self.declare_parameter('calibration_samples', 100)

        self.bus_num = int(self.get_parameter('bus_num').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.topic = self.get_parameter('topic').value
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.accel_alpha = float(self.get_parameter('accel_alpha').value)
        self.gyro_alpha = float(self.get_parameter('gyro_alpha').value)
        self.complementary_alpha = float(self.get_parameter('complementary_alpha').value)
        self.calibration_samples = int(self.get_parameter('calibration_samples').value)

        self.publisher = self.create_publisher(Imu, self.topic, 10)
        self.SMBus = load_smbus_class()
        self.bus = self.SMBus(self.bus_num)
        self._initialize_sensor()

        self.accel_filtered = None
        self.gyro_filtered = None
        self.gyro_bias = [0.0, 0.0, 0.0]
        self.calibration_count = 0
        self.calibration_sum = [0.0, 0.0, 0.0]
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.last_time = time.monotonic()
        self.sample_count = 0

        self.timer = self.create_timer(1.0 / self.rate_hz, self.timer_callback)
        self.get_logger().info(
            f'Publishing filtered MPU6050 data on {self.topic} using I2C bus {self.bus_num}'
        )

    def _initialize_sensor(self):
        self.bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0x00)
        self.bus.write_byte_data(MPU6050_ADDR, SMPLRT_DIV, 19)
        self.bus.write_byte_data(MPU6050_ADDR, CONFIG, 0x03)
        self.bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG, 0x00)
        self.bus.write_byte_data(MPU6050_ADDR, ACCEL_CONFIG, 0x00)
        time.sleep(0.1)

    def read_word_2c(self, reg):
        high = self.bus.read_byte_data(MPU6050_ADDR, reg)
        low = self.bus.read_byte_data(MPU6050_ADDR, reg + 1)
        value = (high << 8) | low
        if value >= 0x8000:
            value = -((65535 - value) + 1)
        return value

    def read_raw_measurements(self):
        ax = self.read_word_2c(ACCEL_XOUT_H)
        ay = self.read_word_2c(ACCEL_XOUT_H + 2)
        az = self.read_word_2c(ACCEL_XOUT_H + 4)
        gx = self.read_word_2c(GYRO_XOUT_H)
        gy = self.read_word_2c(GYRO_XOUT_H + 2)
        gz = self.read_word_2c(GYRO_XOUT_H + 4)
        return ax, ay, az, gx, gy, gz

    def _low_pass(self, previous, current, alpha):
        if previous is None:
            return list(current)
        return [alpha * current[index] + (1.0 - alpha) * previous[index] for index in range(len(current))]

    def _quaternion_from_euler(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw

    def _publish_calibration_progress(self):
        if self.calibration_count == self.calibration_samples:
            self.gyro_bias = [value / float(self.calibration_samples) for value in self.calibration_sum]
            self.get_logger().info(
                f'Gyro bias calibrated: x={self.gyro_bias[0]:.2f}, '
                f'y={self.gyro_bias[1]:.2f}, z={self.gyro_bias[2]:.2f}'
            )

    def timer_callback(self):
        try:
            ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw = self.read_raw_measurements()
        except OSError as exc:
            self.get_logger().error(f'I2C read failed: {exc}')
            return

        if self.calibration_count < self.calibration_samples:
            self.calibration_sum[0] += gx_raw
            self.calibration_sum[1] += gy_raw
            self.calibration_sum[2] += gz_raw
            self.calibration_count += 1
            self._publish_calibration_progress()
            return

        ax_g = ax_raw / ACCEL_SCALE_LSB_PER_G
        ay_g = ay_raw / ACCEL_SCALE_LSB_PER_G
        az_g = az_raw / ACCEL_SCALE_LSB_PER_G
        gx_dps = (gx_raw - self.gyro_bias[0]) / GYRO_SCALE_LSB_PER_DPS
        gy_dps = (gy_raw - self.gyro_bias[1]) / GYRO_SCALE_LSB_PER_DPS
        gz_dps = (gz_raw - self.gyro_bias[2]) / GYRO_SCALE_LSB_PER_DPS

        accel_vector = [ax_g * GRAVITY, ay_g * GRAVITY, az_g * GRAVITY]
        gyro_vector = [math.radians(gx_dps), math.radians(gy_dps), math.radians(gz_dps)]

        self.accel_filtered = self._low_pass(self.accel_filtered, accel_vector, self.accel_alpha)
        self.gyro_filtered = self._low_pass(self.gyro_filtered, gyro_vector, self.gyro_alpha)

        now = time.monotonic()
        dt = now - self.last_time
        self.last_time = now
        if dt <= 0.0:
            dt = 1.0 / self.rate_hz

        ax_f, ay_f, az_f = self.accel_filtered
        gx_f, gy_f, gz_f = self.gyro_filtered

        roll_acc = math.atan2(ay_f, az_f)
        pitch_acc = math.atan2(-ax_f, math.sqrt(ay_f * ay_f + az_f * az_f))

        self.roll = self.complementary_alpha * (self.roll + gx_f * dt) + (1.0 - self.complementary_alpha) * roll_acc
        self.pitch = self.complementary_alpha * (self.pitch + gy_f * dt) + (1.0 - self.complementary_alpha) * pitch_acc
        self.yaw = self.yaw + gz_f * dt

        qx, qy, qz, qw = self._quaternion_from_euler(self.roll, self.pitch, self.yaw)

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        msg.angular_velocity.x = gx_f
        msg.angular_velocity.y = gy_f
        msg.angular_velocity.z = gz_f
        msg.linear_acceleration.x = ax_f
        msg.linear_acceleration.y = ay_f
        msg.linear_acceleration.z = az_f
        msg.orientation_covariance = [0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.05]
        msg.angular_velocity_covariance = [0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.02]
        msg.linear_acceleration_covariance = [0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.05]
        self.publisher.publish(msg)

        self.sample_count += 1
        if self.sample_count % int(self.rate_hz) == 0:
            self.get_logger().info(
                f'roll={math.degrees(self.roll):.1f} deg, pitch={math.degrees(self.pitch):.1f} deg, '
                f'yaw={math.degrees(self.yaw):.1f} deg'
            )

    def destroy_node(self):
        try:
            self.bus.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = IMUPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ImportError as exc:
        print(exc)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()