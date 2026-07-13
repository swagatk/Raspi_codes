#!/usr/bin/env python3

from collections import deque
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import TransformBroadcaster


def quaternion_from_yaw(yaw):
    half = 0.5 * yaw
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class ScanImuSlamNode(Node):
    def __init__(self):
        super().__init__('scan_imu_slam_node')

        self.declare_parameter('scan_topic', '/pirobot2/scan')
        self.declare_parameter('imu_topic', '/pirobot2/imu/data_raw')
        self.declare_parameter('map_topic', '/pirobot2/map')
        self.declare_parameter('pose_topic', '/pirobot2/slam_pose')
        self.declare_parameter('path_topic', '/pirobot2/slam_path')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('width', 800)
        self.declare_parameter('height', 800)
        self.declare_parameter('origin_x', -20.0)
        self.declare_parameter('origin_y', -20.0)
        self.declare_parameter('free_update', -0.35)
        self.declare_parameter('occupied_update', 0.90)
        self.declare_parameter('min_log_odds', -4.0)
        self.declare_parameter('max_log_odds', 4.0)
        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('accel_deadband', 0.10)
        self.declare_parameter('scan_match_enabled', True)
        self.declare_parameter('scan_match_window_m', 0.25)
        self.declare_parameter('scan_match_step_m', 0.05)
        self.declare_parameter('scan_match_min_points', 20)
        self.declare_parameter('scan_match_prior_weight', 0.20)
        self.declare_parameter('yaw_filter_window', 5)
        self.declare_parameter('zupt_accel_threshold', 0.15)
        self.declare_parameter('zupt_gyro_threshold', 0.05)
        self.declare_parameter('zupt_min_samples', 8)
        self.declare_parameter('publish_tf', True)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.resolution = float(self.get_parameter('resolution').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.origin_x = float(self.get_parameter('origin_x').value)
        self.origin_y = float(self.get_parameter('origin_y').value)
        self.free_update = float(self.get_parameter('free_update').value)
        self.occupied_update = float(self.get_parameter('occupied_update').value)
        self.min_log_odds = float(self.get_parameter('min_log_odds').value)
        self.max_log_odds = float(self.get_parameter('max_log_odds').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.accel_deadband = float(self.get_parameter('accel_deadband').value)
        self.scan_match_enabled = bool(self.get_parameter('scan_match_enabled').value)
        self.scan_match_window_m = float(self.get_parameter('scan_match_window_m').value)
        self.scan_match_step_m = float(self.get_parameter('scan_match_step_m').value)
        self.scan_match_min_points = int(self.get_parameter('scan_match_min_points').value)
        self.scan_match_prior_weight = float(self.get_parameter('scan_match_prior_weight').value)
        self.yaw_filter_window = max(1, int(self.get_parameter('yaw_filter_window').value))
        self.zupt_accel_threshold = float(self.get_parameter('zupt_accel_threshold').value)
        self.zupt_gyro_threshold = float(self.get_parameter('zupt_gyro_threshold').value)
        self.zupt_min_samples = int(self.get_parameter('zupt_min_samples').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.log_odds = [0.0] * (self.width * self.height)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_imu_time = None
        self.stationary_count = 0
        self.is_stationary = False
        self.map_initialized = False
        self.yaw_sin_buffer = deque(maxlen=self.yaw_filter_window)
        self.yaw_cos_buffer = deque(maxlen=self.yaw_filter_window)

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.map_frame
        self.last_path_x = 0.0
        self.last_path_y = 0.0
        self.last_path_yaw = 0.0

        self.map_publisher = self.create_publisher(OccupancyGrid, self.map_topic, 1)
        self.pose_publisher = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.path_publisher = self.create_publisher(Path, self.path_topic, 10)

        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Imu, self.imu_topic, self.imu_callback, qos_profile_sensor_data)
        self.create_timer(1.0 / self.publish_rate_hz, self.publish_outputs)

        self.get_logger().info(
            f'Scan+IMU SLAM node started. scan={self.scan_topic}, imu={self.imu_topic}, map={self.map_topic}'
        )
        if self.scan_match_enabled:
            self.get_logger().info(
                f'Scan matching enabled. window={self.scan_match_window_m:.2f} m, step={self.scan_match_step_m:.2f} m'
            )
        else:
            self.get_logger().warn('Scan matching is disabled; pose XY will remain fixed unless changed externally.')
        self.get_logger().info(f'Yaw moving-average filter window: {self.yaw_filter_window} samples')

    def _update_filtered_yaw(self, raw_yaw):
        self.yaw_sin_buffer.append(math.sin(raw_yaw))
        self.yaw_cos_buffer.append(math.cos(raw_yaw))

        mean_sin = sum(self.yaw_sin_buffer) / len(self.yaw_sin_buffer)
        mean_cos = sum(self.yaw_cos_buffer) / len(self.yaw_cos_buffer)
        return math.atan2(mean_sin, mean_cos)

    def imu_callback(self, msg):
        stamp = msg.header.stamp
        current_time = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if current_time <= 0.0:
            current_time = self.get_clock().now().nanoseconds * 1e-9

        if self.last_imu_time is None:
            self.last_imu_time = current_time
            raw_yaw = yaw_from_quaternion(msg.orientation)
            self.yaw = self._update_filtered_yaw(raw_yaw)
            return

        dt = current_time - self.last_imu_time
        self.last_imu_time = current_time
        if dt <= 0.0 or dt > 0.5:
            return

        raw_yaw = yaw_from_quaternion(msg.orientation)
        self.yaw = self._update_filtered_yaw(raw_yaw)

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        if abs(ax) < self.accel_deadband:
            ax = 0.0
        if abs(ay) < self.accel_deadband:
            ay = 0.0

        accel_norm = math.sqrt(ax * ax + ay * ay)
        gyro_norm = math.sqrt(
            msg.angular_velocity.x * msg.angular_velocity.x
            + msg.angular_velocity.y * msg.angular_velocity.y
            + msg.angular_velocity.z * msg.angular_velocity.z
        )
        if accel_norm < self.zupt_accel_threshold and gyro_norm < self.zupt_gyro_threshold:
            self.stationary_count += 1
        else:
            self.stationary_count = 0

        self.is_stationary = self.stationary_count >= self.zupt_min_samples

    def scan_callback(self, msg):
        points = []
        angle = msg.angle_min
        for distance in msg.ranges:
            if math.isfinite(distance) and msg.range_min <= distance <= msg.range_max:
                points.append((distance, angle))
            angle += msg.angle_increment

        if self.scan_match_enabled and not self.is_stationary:
            self.update_pose_from_scan_matching(points)

        for distance, beam_angle in points:
            end_x = self.x + distance * math.cos(self.yaw + beam_angle)
            end_y = self.y + distance * math.sin(self.yaw + beam_angle)
            self.update_ray(self.x, self.y, end_x, end_y)

        if points:
            self.map_initialized = True

        self.update_path(msg.header.stamp)

    def update_path(self, stamp):
        dx = self.x - self.last_path_x
        dy = self.y - self.last_path_y
        dyaw = abs(math.atan2(math.sin(self.yaw - self.last_path_yaw), math.cos(self.yaw - self.last_path_yaw)))

        if dx * dx + dy * dy >= 0.01 or dyaw >= math.radians(3.0):
            pose = self.build_pose_stamped(stamp)
            self.path_msg.poses.append(pose)
            self.last_path_x = self.x
            self.last_path_y = self.y
            self.last_path_yaw = self.yaw

    def update_pose_from_scan_matching(self, points):
        if not self.map_initialized:
            return

        if len(points) < self.scan_match_min_points:
            return

        best_x = self.x
        best_y = self.y
        best_score = float('-inf')

        window = max(0.0, self.scan_match_window_m)
        step = max(0.01, self.scan_match_step_m)
        max_steps = int(window / step)

        for ix in range(-max_steps, max_steps + 1):
            candidate_x = self.x + ix * step
            for iy in range(-max_steps, max_steps + 1):
                candidate_y = self.y + iy * step
                score = self.score_scan_pose(points, candidate_x, candidate_y)

                # Small motion prior helps avoid jumping between equivalent local optima.
                offset_sq = (candidate_x - self.x) * (candidate_x - self.x) + (candidate_y - self.y) * (candidate_y - self.y)
                score -= self.scan_match_prior_weight * offset_sq

                if score > best_score:
                    best_score = score
                    best_x = candidate_x
                    best_y = candidate_y

        self.x = best_x
        self.y = best_y

    def score_scan_pose(self, points, pose_x, pose_y):
        score = 0.0
        hit_count = 0
        for distance, beam_angle in points:
            end_x = pose_x + distance * math.cos(self.yaw + beam_angle)
            end_y = pose_y + distance * math.sin(self.yaw + beam_angle)
            gx, gy = self.world_to_grid(end_x, end_y)
            if not self.in_bounds(gx, gy):
                continue

            idx = gy * self.width + gx
            value = self.log_odds[idx]

            if value > 0.0:
                score += value
                hit_count += 1
            elif value < 0.0:
                score += 0.2 * value

        if hit_count == 0:
            return -1e6
        return score

    def world_to_grid(self, wx, wy):
        gx = int((wx - self.origin_x) / self.resolution)
        gy = int((wy - self.origin_y) / self.resolution)
        return gx, gy

    def in_bounds(self, gx, gy):
        return 0 <= gx < self.width and 0 <= gy < self.height

    def set_log_odds(self, gx, gy, delta):
        if not self.in_bounds(gx, gy):
            return
        idx = gy * self.width + gx
        value = self.log_odds[idx] + delta
        if value > self.max_log_odds:
            value = self.max_log_odds
        elif value < self.min_log_odds:
            value = self.min_log_odds
        self.log_odds[idx] = value

    def bresenham(self, x0, y0, x1, y1):
        points = []
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        x = x0
        y = y0

        while True:
            points.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x += sx
            if e2 <= dx:
                err += dx
                y += sy
        return points

    def update_ray(self, start_x, start_y, end_x, end_y):
        gx0, gy0 = self.world_to_grid(start_x, start_y)
        gx1, gy1 = self.world_to_grid(end_x, end_y)

        if not self.in_bounds(gx0, gy0):
            return

        cells = self.bresenham(gx0, gy0, gx1, gy1)
        if not cells:
            return

        for cell_x, cell_y in cells[:-1]:
            self.set_log_odds(cell_x, cell_y, self.free_update)

        occ_x, occ_y = cells[-1]
        self.set_log_odds(occ_x, occ_y, self.occupied_update)

    def build_pose_stamped(self, stamp):
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.0
        pose.pose.orientation = quaternion_from_yaw(self.yaw)
        return pose

    def publish_outputs(self):
        now = self.get_clock().now().to_msg()

        map_msg = OccupancyGrid()
        map_msg.header.stamp = now
        map_msg.header.frame_id = self.map_frame
        map_msg.info.map_load_time = now
        map_msg.info.resolution = self.resolution
        map_msg.info.width = self.width
        map_msg.info.height = self.height
        map_msg.info.origin.position.x = self.origin_x
        map_msg.info.origin.position.y = self.origin_y
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0

        map_data = []
        for value in self.log_odds:
            if abs(value) < 0.05:
                map_data.append(-1)
                continue
            probability = 1.0 - 1.0 / (1.0 + math.exp(value))
            occupancy = int(max(0.0, min(100.0, round(probability * 100.0))))
            map_data.append(occupancy)
        map_msg.data = map_data
        self.map_publisher.publish(map_msg)

        pose_msg = self.build_pose_stamped(now)
        self.pose_publisher.publish(pose_msg)

        self.path_msg.header.stamp = now
        self.path_publisher.publish(self.path_msg)

        if self.tf_broadcaster is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = now
            tf_msg.header.frame_id = self.map_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation = quaternion_from_yaw(self.yaw)
            self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanImuSlamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()