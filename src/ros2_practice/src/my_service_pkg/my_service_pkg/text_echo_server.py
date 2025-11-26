import rclpy
from rclpy.node import Node
from my_interfaces.srv import SendText


class TextEchoServer(Node):
    def __init__(self):
        super().__init__('text_echo_server')
        self.srv = self.create_service(
            SendText,        #  새 타입
            'send_text',     # 서비스 이름
            self.callback
        )
        self.get_logger().info('Text Echo Service Ready: send_text')

    def callback(self, request, response):
        self.get_logger().info(f"Request text: {request.text}")

        if request.text == "야":
            response.response = "호"
        elif request.text == "집" or request.text == "집에":
            response.response = "가고 싶다."
        else:
            response.response = request.text   # 그대로 돌려주기
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TextEchoServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
