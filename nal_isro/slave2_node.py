import rclpy
from rclpy.node import Node
from isro_msgs.srv import Compute

class Slave2Node(Node):
    def __init__(self):
        super().__init__('slave2_node')
        self.srv = self.create_service(Compute, 'compute2', self.compute_callback)
    
    def compute_callback(self, request, response):
        response.result = request.a + request.b
        self.get_logger().info(f'Incoming request\na: {request.a}, b: {request.b}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = Slave2Node()
    rclpy.spin(node)
    rclpy.shutdown()
