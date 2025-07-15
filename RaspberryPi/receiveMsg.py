import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TwistListener(Node):
    def __init__(self):
        super().__init__('twist_listener')
        self.subscription = self.create_subscription(
            String,
            'twist_selection',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received twist: {msg.data}')

def main():
    rclpy.init()
    node = TwistListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
