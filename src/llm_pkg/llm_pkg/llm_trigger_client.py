import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class LLMTriggerClient(Node):
    def __init__(self):
        super().__init__('llm_trigger_client')

        # Trigger 서비스 클라이언트
        self.cli = self.create_client(Trigger, 'llm_trigger')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("서비스 대기 중...")

        self.req = Trigger.Request()

    def send_trigger(self):
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    node = LLMTriggerClient()

    # 그냥 트리거만 보냄!
    print("\nTrigger 호출 중...\n")
    res = node.send_trigger()

    if res:
        print(f"[서비스 응답] success={res.success}, message={res.message}")
    else:
        print("[오류] 서비스 응답 없음")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
