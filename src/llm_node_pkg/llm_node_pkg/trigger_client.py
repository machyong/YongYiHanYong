import rclpy
from rclpy.node import Node
# from llm_node_pkg.srv import TriggerLLM
from std_srvs.srv import Trigger

class TriggerLLMClient(Node):
    def __init__(self):
        # llm_trigger_client 노드 생성
        super().__init__('llm_trigger_client')
        # llm_trigger 서비스 생성
        self.cli = self.create_client(Trigger, 'llm_trigger')
        # 서비스 기다리기
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("서비스 기다리는 중...")
        self.req = Trigger.Request()
    # 서버에 서비스 요청 보내기
    def send_prompt(self):
    # def send_prompt(self, text):
        # req = Trigger.Request()
        # req.prompt = text
        # return self.cli.call_async(req)
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


# trigger_client.py 실행
def main(args=None):
    rclpy.init(args=args)
    node = TriggerLLMClient()

    
    print("\nTrigger 호출\n")
    res = node.send_prompt()

    if res:
        print(f"[서비스 응답] success={res.success}, message={res.message}")
    else:
        print("[오류] 서비스 응답 없음")

    node.destroy_node()
    rclpy.shutdown()
    # future = node.send_prompt("안녕? 서비스 테스트 중이야!")
    # rclpy.spin_until_future_complete(node, future)

    # print("서비스 응답:", future.result().answer)

    # node.destroy_node()
    # rclpy.shutdown()


if __name__ == "__main__":
    main()