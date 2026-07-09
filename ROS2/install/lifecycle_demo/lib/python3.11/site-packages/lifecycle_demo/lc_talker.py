import rclpy
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from std_msgs.msg import String
import time

class LifecycleTalker(Node):

    def __init__(self):
        super().__init__('lc_talker')
        self.get_logger().info('INSTRUCTOR: Node created (Unconfigured). I am silent.')
        self._pub = None
        self._timer = None

    # TRANSITION 1: Configure
    # Allocates memory, sets up publishers/timers, but DOES NOT start them.
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('INSTRUCTOR: Configuring... creating publisher.')
        self._pub = self.create_lifecycle_publisher(String, 'chatter', 10)
        self._timer = self.create_timer(1.0, self.publish_message)
        self._timer.cancel() # Pause timer immediately
        return TransitionCallbackReturn.SUCCESS

    # TRANSITION 2: Activate
    # Actually starts the work.
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('INSTRUCTOR: Activating... I will now start publishing!')
        super().on_activate(state) # Crucial: Enables the lifecycle publisher
        self._timer.reset() # Unpause timer
        return TransitionCallbackReturn.SUCCESS

    # TRANSITION 3: Deactivate
    # Pauses the work but keeps the setup.
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('INSTRUCTOR: Deactivating... Stopping publication.')
        self._timer.cancel()
        super().on_deactivate(state)
        return TransitionCallbackReturn.SUCCESS

    # TRANSITION 4: Cleanup
    # Destroys the publisher/timer to free memory.
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('INSTRUCTOR: Cleaning up... Destroying publisher.')
        self.destroy_publisher(self._pub)
        self.destroy_timer(self._timer)
        return TransitionCallbackReturn.SUCCESS
        
    # TRANSITION 5: Shutdown
    # This is called when the node is being destroyed
    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('INSTRUCTOR: Shutting down... Node is now Finalized.')
        return TransitionCallbackReturn.SUCCESS

    def publish_message(self):
        msg = String()
        msg.data = "Hello World: Lifecycle is Active!"
        if self._pub.is_activated:
            self._pub.publish(msg)
            self.get_logger().info(f'Publishing: "{msg.data}"')

def main():
    rclpy.init()
    node = LifecycleTalker()
    
    try:
        # This loop runs forever until you press Ctrl+C
        rclpy.spin(node)
    except KeyboardInterrupt:
        # This block runs when you press Ctrl+C
        pass
    except ExternalShutdownException:
        pass
    finally:
        # This block runs no matter how the node stops
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
