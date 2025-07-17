import rclpy
from std_msgs.msg import String
import serial
import time
from dataclasses import dataclass

@dataclass
class CommunicationsConfig:
    ROS_TOPIC: str = 'PI_command'
    SERIAL_PORT: str = '/dev/ttyACM0'
    BAUD_RATE: int = 57600 # Check if matches with Arduino
    ACK_TIMEOUT: int = 60  # Seconds

def config() -> CommunicationsConfig:
    return CommunicationsConfig()



class ROSComm:
    def __init__(self, topic=config().ROS_TOPIC, node_name='Jetson'):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(node_name)
        self.publisher = self.node.create_publisher(String, topic, 10)
        self.subscriber = self.node.create_subscription(
            String,
            topic,
            self.receive_callback,
            10
        )
        self._latest_msg = None
        self.node.get_logger().info(f"ROS initialized on topic '{topic}'")

    def receive_callback(self, msg: String):
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
    def __init__(self, port=config().SERIAL_PORT, baudrate=config().BAUD_RATE, timeout=1):
        self.conn = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(2) # Wait for Arduino reset
        print(f"[Serial] Connected to {port}.")

    def send_message(self, message: str):
        full_msg = message.strip() + '\n'
        self.conn.write(full_msg.encode())
        print(f"[Serial] Sent: {full_msg.strip()}")

    def wait_for_ack(self, expected_ack='OK', timeout=config().ACK_TIMEOUT):
        start = time.time()
        while time.time() - start < timeout:
            if self.conn.in_waiting > 0:
                line = self.conn.readline().decode().strip()
                if line == expected_ack:
                    print("[Serial] ACK received.")
                    return True
        print("[Serial] ACK timeout.")
        return False
    
    def read_line(self):
        return self.conn.readline().decode().strip()

    def in_waiting(self):
        return self.conn.in_waiting
    
    def read_serial_responses(serial_comm):
        while serial_comm.in_waiting():
            try:
                line = serial_comm.read_line()
                if line:
                    print(f"[Arduino]: {line}")
            except UnicodeDecodeError:
                pass


    def close(self):
        self.conn.close()


# Arduino sync
def wait_for_arduino_ready(ser):
    print("Waiting for Arduino to finish setup...")
    deadline = time.time() + 12
    while time.time() < deadline:
        try:
            line = ser.readline().decode().strip()
            if line:
                print(f"[Arduino]: {line}")
                if "Finished setup" in line:
                    print("Arduino is ready.\n")
                    return True
        except:
            pass
    print("Timed out waiting for Arduino.")
    return False

# Send angles to Arduino
def send_to_arduino(ser, angles_list):
    ser.reset_input_buffer()
    print("Sending angles to Arduino...")
    ser.write(b"POSITION\n")
    ser.flush()
    time.sleep(0.1)

    for idx, angle_set in enumerate(angles_list):
        t1, t2, t3 = angle_set
        cmd = f"ANGLES {int(t1)},{int(t2)},{int(t3)}\n"
        ser.write(cmd.encode())
        ser.flush()
        time.sleep(0.05)

    ser.write(b"GO\n")
    ser.flush()
    print("Movement command sent.\n")