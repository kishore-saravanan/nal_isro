import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from isro_msgs.msg import ObjectData
from isro_msgs.srv import StringSrv
from rclpy.duration import Duration
from rclpy.clock import Clock

class Slave5Node(Node):
    def __init__(self):
        super().__init__('slave5_node')
        self.srv = self.create_service(StringSrv, 'compute5', self.compute_callback)
        self.subscription = self.create_subscription(
            ObjectData,
            'object_info',
            self.object_info_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.moved_publisher = self.create_publisher(Bool, 'moved_bool', 10)
        timer_period = 0.01  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        self.start_exec = False
        self.start_time = None  # To store the time when execution starts
        self.goal_reached = None
        self.target_depth = 2.0  # 15 cm
        self.linear_speed = -0.5  # m/s
        self.angular_speed = 0.5  # rad/s

    
    def compute_callback(self, request, response):
        self.get_logger().info(f'Incoming request\na: {request.input}')
        self.start_exec = True
        self.start_time = self.get_clock().now()  # Record the start time
        response.result = True
        rate = self.create_rate(2, self.get_clock())
        return response
    
    # def timer_callback(self):
    #     if self.start_exec:
    #         current_time = self.get_clock().now()
    #         elapsed_time = current_time - self.start_time
    #         if elapsed_time < Duration(seconds=5.0):
    #             msg = Twist()
    #             msg.linear.x = 1.0
    #             msg.linear.y = 0.0
    #             msg.linear.z = 0.0
    #             msg.angular.x = 0.0
    #             msg.angular.y = 0.0
    #             msg.angular.z = 0.5
    #             self.publisher.publish(msg)
    #             self.get_logger().info(f'Published cmd_vel: linear=({msg.linear.x}, {msg.linear.y}, {msg.linear.z}) angular=({msg.angular.x}, {msg.angular.y}, {msg.angular.z})')
    #         else:
    #             self.start_exec = False
    #             self.get_logger().info('Stopped publishing cmd_vel after 5 seconds')

    def object_info_callback(self, msg):
        if self.start_exec:
            self.get_logger().info(f'Received object info: {msg.name} at ({msg.center_x}, {msg.center_y}), depth: {msg.depth}m')
            
            # Calculate control commands
            twist = Twist()

            if msg.depth < self.target_depth:
                # Move forward
                twist.linear.x = self.linear_speed
            else:
                # Stop
                twist.linear.x = 0.0

            # Assuming center_x is the horizontal pixel position of the object in the image frame
            # Adjust angular velocity to center the object
            # You may need to adjust the following logic based on your camera's field of view and resolution
            # if msg.center_x < 300:  # Assuming 640x480 resolution, center_x < 320 means object is to the left
            #     twist.angular.z = self.angular_speed
            # elif msg.center_x > 340:  # center_x > 320 means object is to the right
            #     twist.angular.z = -self.angular_speed
            # else:
            #     twist.angular.z = 0.0

            if msg.depth > self.target_depth:
                self.goal_reached = True
                self.start_exec = False
                bool_msg = Bool()
                bool_msg.data = True
                self.moved_publisher.publish(bool_msg)

            self.publisher.publish(twist)
            
        else:
            print("Not moving")

def main(args=None):
    rclpy.init(args=args)
    node = Slave5Node()
    rclpy.spin(node)
    rclpy.shutdown()
