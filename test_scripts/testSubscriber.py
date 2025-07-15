import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TestNode(Node):
    def __init__(self):
        super().__init__('test_listener')
        self.subscription = self.create_subscription(
            String,
            'PI_command',
            self.listener_callback,
            10
        )
        self.get_logger().info("✅ Subscriber created on /PI_command")

    def listener_callback(self, msg):
        self.get_logger().info(f"📥 Received message: {msg.data}")

def main():
    rclpy.init()
    node = TestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
