import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Publisher(Node):
    def __init__(self):
        super().__init__('publisher')

        self.declare_parameter('topic_name', '/spgc/receiver')
        self.declare_parameter('text', 'Hello, ROS2!')

        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.text = self.get_parameter('text').get_parameter_value().string_value

        self.publisher = self.create_publisher(String, topic_name, 10)
        self.get_logger().info(f"Publisher started on topic '{topic_name}' with default text '{self.text}'") 
        self.add_on_set_parameters_callback(self.parameters_callback)   
        self.timer = self.create_timer(1.0, self.publish_message)

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'text':
                self.text = param.value
                self.get_logger().info(f'Parameter text updated to: "{self.text}"')
                if self.publish_timer:
                    self.publish_and_exit()
        return rclpy.node.SetParametersResult(successful=True)
    
    def publish_message(self):
        msg = String()
        msg.data = self.text
        self.publisher.publish(msg)
        self.get_logger().info(f'Published message: "{msg.data}"')
        

def main(args=None):
    rclpy.init(args=args)
    node = Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
