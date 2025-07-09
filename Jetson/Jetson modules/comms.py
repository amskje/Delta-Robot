import rclpy
from std_msgs.msg import String
import serial
import time

# Constants (adjust as needed)
ROS_TOPIC = 'PI_command'
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
ACK_TIMEOUT = 5  # seconds


class ROSComm:
    def __init__(self, topic=ROS_TOPIC, node_name='Jetson'):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(node_name)
        self.publisher = self.node.create_publisher(String, topic, 10)
        self.subscriber = self.node.create_subscription(
            String,
            topic,
            self._receive_callback,
            10
        )
        self._latest_msg = None
        self.node.get_logger().info(f"ROS initialized on topic '{topic}'")

    def _receive_callback(self, msg: String):
        self.node.get_logger().info(f"Received: {msg.data}")
        self._latest_msg = msg.data

    def send_message(self, message: str):
        msg = String()
        msg.data = message
        self.publisher.publish(msg)
        self.node.get_logger().info(f"Published: {msg.data}")

    def get_latest_message(self) -> str:
        return str(self._latest_msg)

    def clear_message(self):
        self._latest_msg = None

    def spin_once(self, timeout_sec=0.1):
        rclpy.spin_once(self.node, timeout_sec=timeout_sec)

    def shutdown(self):
        self.node.destroy_node()
        rclpy.shutdown()


class SerialComm:
    def __init__(self, port=SERIAL_PORT, baudrate=BAUD_RATE):
        self.conn = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)
        print(f"[Serial] Connected to {port}.")

    def send_message(self, message: str):
        full_msg = message.strip() + '\n'
        self.conn.write(full_msg.encode())
        print(f"[Serial] Sent: {full_msg.strip()}")

    def wait_for_ack(self, expected_ack='OK', timeout=ACK_TIMEOUT):
        start = time.time()
        while time.time() - start < timeout:
            if self.conn.in_waiting > 0:
                line = self.conn.readline().decode().strip()
                if line == expected_ack:
                    print("[Serial] ACK received.")
                    return True
        print("[Serial] ACK timeout.")
        return False

    def close(self):
        self.conn.close()