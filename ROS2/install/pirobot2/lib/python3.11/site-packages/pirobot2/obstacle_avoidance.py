#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        self.declare_parameter('ultrasonic_topic', '/pirobot2/ultrasonic')
        self.declare_parameter('cmd_vel_topic', '/pirobot2/cmd_vel')
        self.declare_parameter('control_hz', 10.0)
        self.declare_parameter('sensor_timeout_sec', 0.6)
        self.declare_parameter('obstacle_distance_cm', 35)
        self.declare_parameter('danger_distance_cm', 20)
        self.declare_parameter('side_clearance_cm', 18)
        self.declare_parameter('forward_speed', 0.18)
        self.declare_parameter('turn_speed', 0.22)
        self.declare_parameter('reverse_speed', 0.10)

        self.ultrasonic_topic = str(self.get_parameter('ultrasonic_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.control_hz = float(self.get_parameter('control_hz').value)
        self.sensor_timeout_sec = float(self.get_parameter('sensor_timeout_sec').value)
        self.obstacle_distance_cm = int(self.get_parameter('obstacle_distance_cm').value)
        self.danger_distance_cm = int(self.get_parameter('danger_distance_cm').value)
        self.side_clearance_cm = int(self.get_parameter('side_clearance_cm').value)
        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.reverse_speed = float(self.get_parameter('reverse_speed').value)

        self.latest_distances = None
        self.last_sensor_time = self.get_clock().now()

        self.ultrasonic_sub = self.create_subscription(
            Int32MultiArray,
            self.ultrasonic_topic,
            self.ultrasonic_callback,
            20,
        )
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 20)
        self.control_timer = self.create_timer(1.0 / self.control_hz, self.control_loop)

        self.get_logger().info(
            f'Obstacle avoidance active. Subscribing {self.ultrasonic_topic}, publishing {self.cmd_vel_topic}'
        )

    def ultrasonic_callback(self, msg):
        if len(msg.data) < 3:
            return

        # Input format: [left, center, right] in centimeters.
        left = int(msg.data[0])
        center = int(msg.data[1])
        right = int(msg.data[2])

        # Treat invalid/zero readings as no echo (far away) to avoid false stops.
        left = left if left > 0 else 999
        center = center if center > 0 else 999
        right = right if right > 0 else 999

        self.latest_distances = (left, center, right)
        self.last_sensor_time = self.get_clock().now()

    def _publish_cmd(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)

    def _is_sensor_stale(self):
        age_sec = (self.get_clock().now() - self.last_sensor_time).nanoseconds / 1e9
        return age_sec > self.sensor_timeout_sec

    def control_loop(self):
        if self.latest_distances is None or self._is_sensor_stale():
            self._publish_cmd(0.0, 0.0)
            return

        left, center, right = self.latest_distances

        if center <= self.danger_distance_cm:
            if left <= self.side_clearance_cm and right <= self.side_clearance_cm:
                # Boxed in: reverse and bias turn to the side with more space.
                turn_sign = 1.0 if left >= right else -1.0
                self._publish_cmd(-self.reverse_speed, turn_sign * self.turn_speed)
                return

            turn_sign = 1.0 if left >= right else -1.0
            self._publish_cmd(0.0, turn_sign * self.turn_speed)
            return

        if center <= self.obstacle_distance_cm:
            turn_sign = 1.0 if left >= right else -1.0
            self._publish_cmd(0.0, turn_sign * self.turn_speed)
            return

        if left <= self.side_clearance_cm and right > left:
            self._publish_cmd(0.0, -self.turn_speed)
            return

        if right <= self.side_clearance_cm and left > right:
            self._publish_cmd(0.0, self.turn_speed)
            return

        self._publish_cmd(self.forward_speed, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ObstacleAvoidance()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node._publish_cmd(0.0, 0.0)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
