import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from isro_msgs.msg import ObjectData
import cv2
from cv_bridge import CvBridge
import math

class DepthProcessor(Node):

    def __init__(self):
        super().__init__('depth_processor')
        self.subscription = self.create_subscription(
            String,
            '/object_data_string',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.depth_subscription = self.create_subscription(
            Image,
            '/zed/zed/depth/depth_registered',  # Adjust this topic name as needed
            self.depth_image_callback,
            10)
        self.depth_subscription  # prevent unused variable warning
        self.br = CvBridge()
        self.publisher = self.create_publisher(ObjectData, 'object_info', 10)
        self.depth_image = None

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

        if name == "container" or name =="sample":
            object_msg = ObjectData()
            object_msg.name = name
            object_msg.center_x = round(start_x + (width/2))
            object_msg.center_y = round(start_y + (height/2))

            if self.depth_image is not None:
                depth_value = self.depth_image[object_msg.center_y, object_msg.center_x]
                if not math.isnan(depth_value):
                    print("Received depth image.")
                    object_msg.depth = self.depth_image[object_msg.center_y, object_msg.center_x]
                    self.publisher.publish(object_msg)
                else:
                    print("Depth value is invalid")
            else:
                print("Not received depth image or depth value is invalid")

        

    def depth_image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        self.depth_image = cv2.resize(self.br.imgmsg_to_cv2(msg, desired_encoding='passthrough'), (640, 640))

        # # Define the coordinate you want to check the depth for
        # u = 320  # x-coordinate
        # v = 240  # y-coordinate

        # # Extract depth value at (u, v)
        # depth_value = depth_image[v, u]

        # self.get_logger().info(f'Depth at coordinate ({u}, {v}): {depth_value} meters')


def main(args=None):
    rclpy.init(args=args)

    string_subscriber = DepthProcessor()

    rclpy.spin(string_subscriber)

    string_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
