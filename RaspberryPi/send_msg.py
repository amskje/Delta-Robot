import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time


class TwistPublisher(Node):

    def __init__(self):
        super().__init__('twist_publisher')
        self.publisher_ = self.create_publisher(String, 'twist_selection', 10)

    def send_twist(self, twist_name: str):
        msg = String()
        msg.data = twist_name
        self.publisher_.publish(msg)
        self.get_logger().info(f"Sent twist: {msg.data}")

def run_publisher():
    #Start ROS 2
    rclpy.init()
    twist_publisher =TwistPublisher()

    ros_thread = threading.Thread(target=rclpy.spin, args=(twist_publisher,), daemon=True)
    ros_thread.start()

    i = 0

    for i in range(100):

        #send test message
        time.sleep(1.0)#give time to init
        twist_publisher.send_twist("hei, dette er en test")

    #give time to send, and then shutdown
    time.sleep(1.0)
    twist_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    run_publisher()