import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
# from cv_bridge import CvBridge # Removed dependency
import cv2
import sys
import numpy as np

class QoSCamSubscriber(Node):
    def __init__(self, reliability_mode, durability_mode):
        super().__init__('qos_cam_sub')
        # self.bridge = CvBridge() # Not needed

        # 1. MATCHING QoS PROFILE
        qos_profile = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10)
        
        if reliability_mode == 'reliable':
            self.get_logger().info('SUBSCRIBING: RELIABLE')
            qos_profile.reliability = ReliabilityPolicy.RELIABLE
        else:
            self.get_logger().info('SUBSCRIBING: BEST_EFFORT')
            qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        if durability_mode == 'transient':
            qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        else:
            qos_profile.durability = DurabilityPolicy.VOLATILE

        self.subscription = self.create_subscription(
            Image,
            'class_video_feed',
            self.listener_callback,
            qos_profile)

    def listener_callback(self, msg):
        try:
            # 1. Convert ROS Image to OpenCV manually (No cv_bridge)
            # Create numpy array from the bytes
            # Note: buffer checks might be needed in prod, but fine for lab
            dtype = np.uint8
            n_channels = 3
            if msg.encoding == 'bgr8':
                n_channels = 3
            
            # Reconstruct image from bytes
            frame = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, n_channels)
            
            # 2. Calculate Latency
            # Current Client Time - Image Header Time
            now = self.get_clock().now()
            msg_time = rclpy.time.Time.from_msg(msg.header.stamp)
            latency = (now - msg_time).nanoseconds / 1e9 # to seconds

            # 3. Display Latency on Screen
            color = (0, 255, 0) if latency < 0.2 else (0, 0, 255) # Green if fast, Red if laggy
            text = f"Latency: {latency:.3f}s"
            cv2.putText(frame, text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

            cv2.imshow("Student View", frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Frame Error: {e}')

def main():
    if len(sys.argv) < 3:
        print("Usage: python3 cam_sub.py [reliable|best_effort] [volatile|transient]")
        return

    rclpy.init()
    node = QoSCamSubscriber(sys.argv[1], sys.argv[2])
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
