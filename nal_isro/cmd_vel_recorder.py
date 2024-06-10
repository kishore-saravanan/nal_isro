import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.clock import Clock
import csv
import os

class CmdVelRecorder(Node):
    def __init__(self):
        super().__init__('cmd_vel_recorder')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.cmd_vel_data = []
        self.get_logger().info('CmdVelRecorder node has been started.')

    def cmd_vel_callback(self, msg):
        timestamp = self.get_clock().now().to_msg()
        self.cmd_vel_data.append((timestamp, msg))
        self.get_logger().info(f'Received cmd_vel at {timestamp.sec}.{timestamp.nanosec}: linear={msg.linear} angular={msg.angular}')

    def save_cmd_vel_data(self):
        # Create a directory for storing the data if it doesn't exist
        os.makedirs('data', exist_ok=True)
        
        with open('data/cmd_vel_data.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['timestamp_sec', 'timestamp_nanosec', 'linear_x', 'linear_y', 'linear_z', 'angular_x', 'angular_y', 'angular_z'])
            for timestamp, msg in self.cmd_vel_data:
                writer.writerow([
                    timestamp.sec,
                    timestamp.nanosec,
                    msg.linear.x,
                    msg.linear.y,
                    msg.linear.z,
                    msg.angular.x,
                    msg.angular.y,
                    msg.angular.z
                ])
        self.get_logger().info('cmd_vel data saved to data/cmd_vel_data.csv')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_cmd_vel_data()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
