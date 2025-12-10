import rclpy
from rclpy.node import Node
from std_msgs.msg import String

except_point = ""

class StringRelayNode(Node):
    def __init__(self):
        super().__init__('string_relay_node')
        self.publisher_ = self.create_publisher(String, '/robot_event', 10)
        self.subscription = self.create_subscription(
            String,
            '/robot_status',
            self.listener_callback,
            10)
        self.subscription = "" # prevent unused variable warning
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = self.subscription
        self.publisher_.publish(msg)

    def listener_callback(self, msg):
        global except_point
        except_point = msg.data 
        self.get_logger().info(f'Received: {except_point}')
        if except_point == 'waiting':
            self.subscription = 'arrived'
        else:
            self.subscription = 'moving'

def main(args=None):
    rclpy.init(args=args)
    node = StringRelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
