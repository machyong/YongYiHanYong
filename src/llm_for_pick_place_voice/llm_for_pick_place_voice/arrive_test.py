import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ArriveTest(Node):
    def __init__(self):
        super().__init__("arrive_test")
        self.sub = self.create_subscription(
            String,
            "robot_event",
            self.callback,
            10,
        )
        self.get_logger().info("robot arrive test node started. Waiting for events...")

    def callback(self, msg: String):
        self.get_logger().info(f"[robot_event] {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = ArriveTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()