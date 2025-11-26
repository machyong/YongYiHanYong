import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class LLMTriggerClient(Node):
    def __init__(self):
        super().__init__('llm_trigger_client')

        # Trigger 서비스 클라이언트
        self.cli = self.create_client(Trigger, 'get_keyword')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("서비스 대기 중...")

        self.req = Trigger.Request()

        self.call_service()
        
    def call_service(self):
        """비동기로 서비스 호출"""
        self.get_logger().info("서비스 호출 중...")
        future = self.cli.call_async(self.req)
        future.add_done_callback(self.service_callback)
    
    def service_callback(self, future):
        """서비스 응답 콜백"""
        try:
            response = future.result()
            self.get_logger().info(f"[서비스 응답] success={response.success}, message={response.message}")
        except Exception as e:
            self.get_logger().error(f"[오류] 서비스 호출 실패: {e}")
        self.call_service()

def main(args=None):
    rclpy.init(args=args)
    node = LLMTriggerClient()

    print("\nTrigger 클라이언트 시작...\n")
    node.call_service()
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()