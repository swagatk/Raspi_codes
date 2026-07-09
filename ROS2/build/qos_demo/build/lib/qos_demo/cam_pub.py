import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
# from cv_bridge import CvBridge # Removed dependency
import cv2
import sys
import time
from picamera2 import Picamera2
import numpy as np

class QoSCamPublisher(Node):
    def __init__(self, reliability_mode, durability_mode):
        super().__init__('qos_cam_pub')
        
        # 1. SETUP DYNAMIC QoS PROFILE
        # depth=10: Keep last 10 frames in buffer if network is slow
        qos_profile = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10)
        
        # Configure Reliability
        if reliability_mode == 'reliable':
            self.get_logger().info('>>> MODE: RELIABLE (Guaranteed Delivery, Higher Latency)')
            qos_profile.reliability = ReliabilityPolicy.RELIABLE
        else:
            self.get_logger().info('>>> MODE: BEST_EFFORT (Drop frames, Low Latency)')
            qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        # Configure Durability (For "Late Joining" demo)
        if durability_mode == 'transient':
            self.get_logger().info('>>> DURABILITY: TRANSIENT_LOCAL (New subs see old msg)')
            qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        else:
            qos_profile.durability = DurabilityPolicy.VOLATILE

        self.publisher_ = self.create_publisher(Image, 'class_video_feed', qos_profile)
        
        # 2. SETUP PICAMERA2 (Bookworm Standard)
        self.picam2 = Picamera2()
        
        # CORRECTED CONFIGURATION METHOD (Matches your working test script):
        self.picam2.preview_configuration.size = (320, 240)
        self.picam2.preview_configuration.format = "BGR888" 
        self.picam2.start()

        self.timer = self.create_timer(0.1, self.timer_callback) # 10 FPS

    def timer_callback(self):
        try:
            # Capture frame
            frame = self.picam2.capture_array()
            
            # Timestamp for visual latency check
            current_time = time.strftime("%H:%M:%S", time.localtime())
            cv2.putText(frame, f"SvrTime: {current_time}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Add a moving bar to visualize dropped frames/stutter
            self.draw_moving_bar(frame)

            # --- MANUAL CONVERSION (No cv_bridge) ---
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_frame'
            
            height, width, channels = frame.shape
            msg.height = height
            msg.width = width
            msg.encoding = 'bgr8'
            msg.is_bigendian = 0
            msg.step = width * channels # Bytes per row
            msg.data = frame.tobytes()
            # ----------------------------------------
            
            self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Camera Error: {e}')

    def draw_moving_bar(self, frame):
        # Visual indicator to see jitter
        sec = int(time.time() * 10) % 320
        cv2.line(frame, (sec, 200), (sec, 240), (0, 0, 255), 5)

def main():
    if len(sys.argv) < 3:
        print("Usage: python3 cam_pub.py [reliable|best_effort] [volatile|transient]")
        return

    rclpy.init()
    node = QoSCamPublisher(sys.argv[1], sys.argv[2])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.picam2.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
