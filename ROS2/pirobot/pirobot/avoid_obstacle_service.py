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
        self.is_running = False
        self.timer=None

            

    def avoid_obstacle_callback(self, request, response):
        if request.data:
            self.get_logger().info('Received Request to start autonomous motion ...')
    
            if not self.is_running:
                response.success=True
                response.message="Navigating & avoiding obstacle"
                timer_period=0.1 # seconds
                self.timer = self.create_timer(
                    timer_period,
                    self.motion_callback)
                # toggle the robot status
                self.is_running = True
            else:
                response.success=False
                response.message="Robot is already running. Nothing to do."
                
        else:
            self.get_logger().info('Received Request to stop autonomous motion')
            if self.is_running:
                response.success=True
                response.message="Stopping autonomous navigation"
                ao.halt()
                self.is_running = False
                if self.timer is not None:
                    self.timer.destroy() # stop publishing
                self.get_logger().info('Robot has stopped moving')
            else:
                response.success=False
                response.message="Robot is not running. Nothing to do"
                self.get_logger().info('Waiting for Client command to start moving')
        return response

    def motion_callback(self):
        msg = ao.main()
        self.get_logger().info(msg)
            
        
        

def main(args=None):
    rclpy.init(args=args)
    ao.initialize()
    ao_service = AvoidObstacleService()
    rclpy.spin(ao_service)
    ao_service.destroy_node()
    ao.cleanup()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
