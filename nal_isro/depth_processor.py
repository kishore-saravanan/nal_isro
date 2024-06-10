import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from isro_msgs.msg import ObjectData

class DepthProcessor(Node):

    def __init__(self):
        super().__init__('depth_processor')
        self.subscription = self.create_subscription(
            String,
            '/object_data_str',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher = self.create_publisher(ObjectData, 'object_info', 10)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        input_str = msg.data
        components = input_str.split(',')
        name = components[0]
        start_x = round(float(components[1]))
        start_y = round(float(components[2]))
        width = round(float(components[3]))
        height = round(float(components[4]))
        confidence = float(components[5])

        # Print the variables to verify
        # print(f"name = {name}")
        # print(f"start_x = {start_x}")
        # print(f"start_y = {start_y}")
        # print(f"width = {width}")
        # print(f"height = {height}")
        # print(f"confidence = {confidence}")

        object_msg = ObjectData()
        object_msg.name = name
        object_msg.center_x = round(start_x + (width/2))
        object_msg.center_y = round(start_y + (height/2))
        object_msg.depth = 1.0

        self.publisher.publish(object_msg)


def main(args=None):
    rclpy.init(args=args)

    string_subscriber = DepthProcessor()

    rclpy.spin(string_subscriber)

    string_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
