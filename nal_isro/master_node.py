import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from isro_msgs.srv import Compute, StringSrv

class MasterNode(Node):
    def __init__(self):
        super().__init__('master_node')
        self.subscription = self.create_subscription(
            Bool,
            'moved_bool',
            self.moved_callback,
            10
        )
        self.client1 = self.create_client(StringSrv, 'move_waypoint')
        self.client2 = self.create_client(Compute, 'compute2')
        self.client3 = self.create_client(StringSrv, 'compute3')
        self.moved = False
        timer_period = 0.01  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.start_task_1() # Commanded Waypoint Navigation
        #self.start_task_2() # Arm pickup
        self.start_task_3() # Move towards container
        while not self.moved:
             rclpy.spin_once(self)
             print("Not moved")
        print("Moved")

#############################################################################################
    
    def start_task_1(self): # Commanded Waypoint Navigations
        str_request = StringSrv.Request()
        str_request.input = "Start"

        while not self.client1.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service compute1 to be available...')
        
        future1 = self.client1.call_async(str_request)

        rclpy.spin_until_future_complete(self, future1)
        if future1.result() is not None:
            self.get_logger().info(f'Result from compute1: {future1.result().result}')
        else:
            self.get_logger().error('Failed to call service compute1')


#############################################################################################

    def start_task_2(self):  # Arm pickup
        request = Compute.Request()
        request.a = 2
        request.b = 3
        str_request = StringSrv.Request()
        str_request.input = "Start"

        while not self.client2.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service compute2 to be available...')

        future2 = self.client2.call_async(request)

        rclpy.spin_until_future_complete(self, future2)
        if future2.result() is not None:
            self.get_logger().info(f'Result from compute2: {future2.result().result}')
        else:
            self.get_logger().error('Failed to call service compute2')

#############################################################################################

    def start_task_3(self): # Move towards container
        str_request = StringSrv.Request()
        str_request.input = "Start" 

        while not self.client3.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service compute3 to be available...')

        future3 = self.client3.call_async(str_request)

        rclpy.spin_until_future_complete(self, future3)
        if future3.result() is not None:
            self.get_logger().info(f'Result from compute3: {future3.result().result}')
        else:
            self.get_logger().error('Failed to call service compute3')

#############################################################################################

    def moved_callback(self, msg):
        if msg.data == True:
            self.moved = True
            print("In moved callback")

def main(args=None):
    rclpy.init(args=args)
    node = MasterNode()
    rclpy.spin(node)
    rclpy.shutdown()
