#publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloWorldPublisher(Node):
	def __init__(self):
		super().__init__('hello_world_publisher')
		# Create a publisher that publishes String messages to the 'topic' topic
		self.publisher_ = self.create_publisher(String, 'topic', 10)
		# Set a timer to trigger the callback every 1.0 second
		timer_period = 1.0
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.i = 0
	def timer_callback(self):
		msg = String()
		msg.data = 'Hello World: %d' % self.i
		# Publish the message
		self.publisher_.publish(msg)
		# Log the message to the console
		self.get_logger().info('Publishing: "%s"' % msg.data)
		self.i += 1



def main(args=None):
		rclpy.init(args=args)
		hello_world_publisher = HelloWorldPublisher()
		try:
			rclpy.spin(hello_world_publisher)
		except KeyboardInterrupt:
			pass
		finally:
			# Destroy the node explicitly
			hello_world_publisher.destroy_node()
			rclpy.shutdown()
			
if __name__ == '__main__':
	main()
