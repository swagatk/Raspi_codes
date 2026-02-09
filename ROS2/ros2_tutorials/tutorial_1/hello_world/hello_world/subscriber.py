#subscriber.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloWorldSubscriber(Node):
	def __init__(self):
		super().__init__('hello_world_subscriber')
		# Create a subscription to the 'topic' topic
		self.subscription = self.create_subscription(
			String,
			'topic',
			self.listener_callback,
			10)
		self.subscription # prevent unused variable warning
	def listener_callback(self, msg):
		# Log the received message to the console
		self.get_logger().info('I heard: "%s"' % msg.data)
		
def main(args=None):
	rclpy.init(args=args)
	hello_world_subscriber = HelloWorldSubscriber()
	try:
		rclpy.spin(hello_world_subscriber)
	except KeyboardInterrupt:
		pass
	finally:
		# Destroy the node explicitly
		hello_world_subscriber.destroy_node()
		rclpy.shutdown()
		
if __name__ == '__main__':
	main()
