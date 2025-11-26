import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서비스 서버 기다리는 중...')

        self.request = AddTwoInts.Request()

    def send_request(self, a: int, b: int):
        self.request.a = a
        self.request.b = b
        self.get_logger().info(f'요청 보내는 중: a={a}, b={b}')
        self.future = self.client.call_async(self.request)

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) >= 3:
        a = int(sys.argv[1])
        b = int(sys.argv[2])
    
    else: 
        a = 7
        b = 8
    
    node = AddTwoIntClient()
    node.send_request(a, b)

    while rclpy.ok():
        rclpy.spin_once(node)
        if node.future.done():
            try:
                response = node.future.result()
            except Exception as e:
                node.get_logger().error(f'Service call failed: {e}')
            else:
                node.get_logger().info(f'Result: {a} + {b} = {response.sum}')
            break
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()