from std_srvs.srv import Trigger, SetBool
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pirobot import avoid_obstacle as ao

class AvoidObstacleService(Node):
    def __init__(self):
        super().__init__('avoid_obstacle_service')
        self.srv = self.create_service(
            SetBool,
            '/pirobot/avoid_obstacle',
            self.avoid_obstacle_callback)
        self.publisher_ = self.create_publisher(
            String,
            'pirobot/status',
            10)
        self.timer=None
            

    def avoid_obstacle_callback(self, request, response):
        #request # request must be specified even if it is not used
        print('request data:', request.data)
        if request.data:
            self.get_logger().info('Received Request to start autonomous navigation ...')
            response.success=True
            response.message="Navigating & avoiding obstacle"
            timer_period=0.1 # seconds
            self.timer = self.create_timer(
                timer_period,
                self.publish_robot_status_callback)

        else:
            self.get_logger().info('Received Request to stop autonomous navigation')
            response.success=True
            response.message="Stopping autonomous navigation"
            ao.halt() # stop motors
            if self.timer is not None:
                self.timer.destroy() # stop publishing
        return response

    def publish_robot_status_callback(self):
        msg = String()
        msg.data = ao.main()
        self.publisher_.publish(msg)
        self.get_logger().info('Status: "%s"' % msg.data)
        
        

def main(args=None):
    rclpy.init(args=args)
    ao.initialize_sensor()
    ao_service = AvoidObstacleService()
    rclpy.spin(ao_service)
    ao_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
