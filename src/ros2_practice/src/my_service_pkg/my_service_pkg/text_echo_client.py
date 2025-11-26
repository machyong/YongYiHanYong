import sys
import rclpy
from rclpy.node import Node
from my_interfaces.srv import SendText


class TextEchoClient(Node):
    def __init__(self):
        super().__init__('text_echo_client')
        self.client = self.create_client(SendText, 'send_text')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서비스 서버 기다리는 중...')

        self.request = SendText.Request()

    def send_request(self, text: str):
        self.request.text = text
        self.get_logger().info(f'요청 보내는 중: "{text}"')
        self.future = self.client.call_async(self.request)


def main(args=None):
    rclpy.init(args=args)

    text = sys.argv[1] if len(sys.argv) >= 2 else "Hello, Service!"
    node = TextEchoClient()
    node.send_request(text)

    while rclpy.ok():
        rclpy.spin_once(node)
        if node.future.done():
            try:
                response = node.future.result()
            except Exception as e:
                node.get_logger().error(f"Service call failed: {e}")
            else:
                node.get_logger().info(f'Result: "{response.response}"')
            break

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
