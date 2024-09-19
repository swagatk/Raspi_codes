# calls the avoid_obstacle_service
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger
import sys

class AvoidObstacleClient(Node):
    def __init__(self):
        super().__init__('avoid_obstacle_client')
        self.client = self.create_client(
                            SetBool,
                             '/pirobot/avoid_obstacle')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service to become available ...')
        #self.req = Trigger.Request()
        self.req = SetBool.Request()

    def send_request(self, d:bool):
        self.req.data = d
        #print('req.data:', self.req.data)
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    input_string = sys.argv[1]
    if input_string.lower() in ['start', 'true', 'True', '1', 't', 'y', 'yes', 'yup']:
        input_bool = bool(True)
    else:
        input_bool = bool(False)
    print('Input argument 2:', input_bool)
    ao_client = AvoidObstacleClient()
    response = ao_client.send_request(input_bool)
    ao_client.get_logger().info(
        f'Response from server:\
            \n\tSuccess: {response.success}\
            \n\tMessage: {response.message}')

    ao_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    

                         
