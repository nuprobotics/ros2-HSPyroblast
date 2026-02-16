import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class TriggerService(Node):
    def __init__(self):
        super().__init__('trigger_service_node')

        self.declare_parameter('service_name', '/trigger_service')
        self.declare_parameter('default_string', 'No service available')

        self.service_name = self.get_parameter('service_name').get_parameter_value().string_value
        self.default_string = self.get_parameter('default_string').get_parameter_value().string_value
        self.stored_string = self.default_string

        self.call_external_service()

        self.srv = self.create_service(Trigger, self.service_name, self.service_callback)
        self.get_logger().info(f"Service '{self.service_name}' is ready.")

    def call_external_service(self):
        client = self.create_client(Trigger, '/spgc/trigger')
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("/spgc/trigger service not available, using default string")
            self.stored_string = self.default_string
            return

        req = Trigger.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.stored_string = future.result().message
            self.get_logger().info(f"Stored string from /spgc/trigger: '{self.stored_string}'")
        else:
            self.get_logger().warn("Failed to call /spgc/trigger, using default string")
            self.stored_string = self.default_string

    def service_callback(self, request, response):
        response.success = True
        response.message = self.stored_string
        return response

        

def main(args=None):
    rclpy.init(args=args)
    node = TriggerService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
