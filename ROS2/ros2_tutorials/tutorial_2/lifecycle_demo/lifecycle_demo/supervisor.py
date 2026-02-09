import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
import time

class Supervisor(Node):

    def __init__(self):
        super().__init__('supervisor_node')
        self.client = self.create_client(ChangeState, '/lc_talker/change_state')

    def change_state(self, transition_id):
        self.get_logger().info(f'SUPERVISOR: Requesting transition {transition_id}...')
        
        # Wait for service
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /lc_talker/change_state service...')

        req = ChangeState.Request()
        req.transition.id = transition_id
        
        # Send Request
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'SUPERVISOR: Transition Result: {future.result().success}')
        else:
            self.get_logger().error('SUPERVISOR: Service call failed')

def main():
    rclpy.init()
    supervisor = Supervisor()

    # Step 1: Configure
    time.sleep(2) # Wait a moment to see the initial state
    supervisor.change_state(Transition.TRANSITION_CONFIGURE)
    
    # Step 2: Activate
    time.sleep(2)
    supervisor.change_state(Transition.TRANSITION_ACTIVATE)

    # Step 3: Run for 5 seconds then Deactivate
    time.sleep(5)
    supervisor.change_state(Transition.TRANSITION_DEACTIVATE)
    
    # Step 4: Cleanup
    time.sleep(2)
    supervisor.change_state(Transition.TRANSITION_CLEANUP)
    
    # Step 5: Shutdown (The missing piece)
    # This moves the node from 'Unconfigured' to 'Finalized'
    time.sleep(2)
    supervisor.get_logger().info('SUPERVISOR: Requesting Shutdown...')
    supervisor.change_state(Transition.TRANSITION_UNCONFIGURED_SHUTDOWN)

    supervisor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
