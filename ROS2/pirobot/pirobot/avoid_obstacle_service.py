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
        self.declare_parameter('status', 0)
            

    def avoid_obstacle_callback(self, request, response):
        if request.data:
            self.get_logger().info('Received Request to start autonomous motion ...')
            current_status = self.get_parameter('status').get_parameter_value().integer_value
            if current_status == 0:
                response.success=True
                response.message="Navigating & avoiding obstacle"
                timer_period=0.1 # seconds
                self.timer = self.create_timer(
                    timer_period,
                    self.publish_robot_status_callback)
                # toggle the robot status
                new_status = rclpy.parameter.Parameter('status',
                                    rclpy.Parameter.Type.INTEGER, 1)
                all_new_params = [new_status]
                self.set_parameters(all_new_params)
                self.get_logger().info('Robot is now moving')
            else:
                response.success=False
                response.message="Robot is already running"
                self.get_logger().info('Robot Status: "%d"' % current_status)
                
        else:
            self.get_logger().info('Received Request to stop autonomous motion')
            current_status = self.get_parameter('status').get_parameter_value().integer_value
            if current_status == 1:
                response.success=True
                response.message="Stopping autonomous navigation"
                ao.halt() # stop motors
                if self.timer is not None:
                    self.timer.destroy() # stop publishing
                self.get_logger().info('Robot has stopped moving')
                # toggle robot status
                new_status = rclpy.parameter.Parameter('status',
                            rclpy.Parameter.Type.INTEGER, 0)
                all_new_params = [new_status]
                self.set_parameters(all_new_params)
            else:
                response.success=False
                response.message="Robot is not running. Nothing to do"
                self.get_logger().info('Robot Status: "%d"' % current_status)
                self.get_logger().info('Waiting for Client command to start moving')

        return response

    def publish_robot_status_callback(self):
        msg = String()
        msg.data = ao.main()
        self.publisher_.publish(msg)
        self.get_logger().info('Status: "%s"' % msg.data)
        
        

def main(args=None):
    rclpy.init(args=args)
    ao.initialize()
    ao_service = AvoidObstacleService()
    rclpy.spin(ao_service)
    ao_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
