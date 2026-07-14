#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


class CmdVelOdometryNode(Node):
    def __init__(self):
        super().__init__('cmdvel_odom_node')

        self.declare_parameter('cmd_vel_topic', '/pirobot2/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('cmd_timeout_sec', 0.7)
        self.declare_parameter('linear_scale', 1.0)
        self.declare_parameter('angular_scale', 1.0)
        self.declare_parameter('imu_topic', '/pirobot2/imu/data_raw')
        self.declare_parameter('use_imu_yaw', True)
        self.declare_parameter('imu_yaw_weight', 0.85)
        self.declare_parameter('imu_timeout_sec', 0.4)

        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.cmd_timeout_sec = float(self.get_parameter('cmd_timeout_sec').value)
        self.linear_scale = float(self.get_parameter('linear_scale').value)
        self.angular_scale = float(self.get_parameter('angular_scale').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.use_imu_yaw = bool(self.get_parameter('use_imu_yaw').value)
        self.imu_yaw_weight = max(0.0, min(1.0, float(self.get_parameter('imu_yaw_weight').value)))
        self.imu_timeout_sec = float(self.get_parameter('imu_timeout_sec').value)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.wz = 0.0
        self.imu_wz = 0.0
        self.imu_yaw = None

        self.last_cmd_time = None
        self.last_imu_time = None
        self.last_update_time = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 20)
        self.tf_pub = TransformBroadcaster(self) if self.publish_tf else None

        self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_callback, 20)
        self.create_subscription(Imu, self.imu_topic, self.imu_callback, qos_profile_sensor_data)
        self.create_timer(1.0 / max(1.0, self.publish_rate_hz), self.update)

        self.get_logger().info(
            f'cmd_vel odometry active. cmd_vel={self.cmd_vel_topic}, odom={self.odom_topic}, '
            f'imu={self.imu_topic}, frames={self.odom_frame}->{self.base_frame}'
        )

    def cmd_vel_callback(self, msg):
        self.vx = float(msg.linear.x)
        self.wz = float(msg.angular.z)
        self.last_cmd_time = self.get_clock().now()

    def imu_callback(self, msg):
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.imu_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.imu_wz = float(msg.angular_velocity.z)
        self.last_imu_time = self.get_clock().now()

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1e-9
        self.last_update_time = now

        if dt <= 0.0 or dt > 0.5:
            return

        if self.last_cmd_time is not None:
            age = (now - self.last_cmd_time).nanoseconds * 1e-9
            if age > self.cmd_timeout_sec:
                self.vx = 0.0
                self.wz = 0.0

        cmd_wz = self.wz * self.angular_scale
        yaw_pred = self.yaw + cmd_wz * dt

        imu_fresh = False
        if self.last_imu_time is not None:
            imu_age = (now - self.last_imu_time).nanoseconds * 1e-9
            imu_fresh = imu_age <= self.imu_timeout_sec

        if self.use_imu_yaw and imu_fresh and self.imu_yaw is not None:
            yaw_error = math.atan2(
                math.sin(self.imu_yaw - yaw_pred),
                math.cos(self.imu_yaw - yaw_pred),
            )
            self.yaw = yaw_pred + self.imu_yaw_weight * yaw_error
        else:
            self.yaw = yaw_pred

        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        vx_scaled = self.vx * self.linear_scale
        self.x += vx_scaled * math.cos(self.yaw) * dt
        self.y += vx_scaled * math.sin(self.yaw) * dt

        qz = math.sin(0.5 * self.yaw)
        qw = math.cos(0.5 * self.yaw)
        stamp = now.to_msg()

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        odom_msg.twist.twist.linear.x = vx_scaled
        odom_msg.twist.twist.angular.z = self.imu_wz if (self.use_imu_yaw and imu_fresh) else cmd_wz

        self.odom_pub.publish(odom_msg)

        if self.tf_pub is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw
            self.tf_pub.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
