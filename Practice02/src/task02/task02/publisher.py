import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class Publisher(Node):
    def __init__(self):
        super().__init__('publisher')

        self.declare_parameter('topic_name', '/spgc/receiver')
        self.declare_parameter('text', 'Hello, ROS2!')

        self.publisher = None

    def start(self):
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.text = self.get_parameter('text').get_parameter_value().string_value

        self.publisher = self.create_publisher(String, topic_name, 10)
        self.get_logger().info(f"Publisher started on topic '{topic_name}' with text '{self.text}'")

        time.sleep(0.05)

        msg = String()
        msg.data = self.text
        self.publisher.publish(msg)
        self.get_logger().info(f'Published message: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = Publisher()
    rclpy.spin_once(node, timeout_sec=0.1)
    node.start()
    node.destroy_node()
    rclpy.shutdown()
