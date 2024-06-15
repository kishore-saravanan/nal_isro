import rclpy
from rclpy.node import Node
from isro_msgs.srv import StringSrv
from geometry_msgs.msg import Twist
import csv
import os
import time

class Slave6Node(Node):
    def __init__(self):
        super().__init__('slave6_node')
        self.srv = self.create_service(StringSrv, 'move_waypoint_2', self.compute_callback)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('CmdVelPublisher node has been started.')
    
    def compute_callback(self, request, response):
        self.get_logger().info(f'Incoming request\na: {request.input}')
        self.read_and_publish_cmd_vel_data()
        response.result = True
        return response
    
    def read_and_publish_cmd_vel_data(self):
        csv_file = 'data/cmd_vel_data_2.csv'
        if not os.path.exists(csv_file):
            self.get_logger().error(f'CSV file {csv_file} does not exist.')
            return

        with open(csv_file, 'r') as file:
            reader = csv.DictReader(file)
            previous_timestamp_sec = None
            previous_timestamp_nanosec = None

            for row in reader:
                timestamp_sec = int(row['timestamp_sec'])
                timestamp_nanosec = int(row['timestamp_nanosec'])
                linear_x = float(row['linear_x'])
                linear_y = float(row['linear_y'])
                linear_z = float(row['linear_z'])
                angular_x = float(row['angular_x'])
                angular_y = float(row['angular_y'])
                angular_z = float(row['angular_z'])

                if previous_timestamp_sec is not None and previous_timestamp_nanosec is not None:
                    # Calculate time to wait before publishing this message
                    current_time = timestamp_sec + timestamp_nanosec / 1e9
                    previous_time = previous_timestamp_sec + previous_timestamp_nanosec / 1e9
                    time_to_wait = current_time - previous_time
                    time.sleep(time_to_wait)

                # Create and publish the Twist message
                msg = Twist()
                msg.linear.x = linear_x
                msg.linear.y = linear_y
                msg.linear.z = linear_z
                msg.angular.x = angular_x
                msg.angular.y = angular_y
                msg.angular.z = angular_z
                self.publisher.publish(msg)
                self.get_logger().info(f'Published cmd_vel: linear=({linear_x}, {linear_y}, {linear_z}) angular=({angular_x}, {angular_y}, {angular_z})')

                previous_timestamp_sec = timestamp_sec
                previous_timestamp_nanosec = timestamp_nanosec

def main(args=None):
    rclpy.init(args=args)
    node = Slave6Node()
    rclpy.spin(node)
    rclpy.shutdown()
