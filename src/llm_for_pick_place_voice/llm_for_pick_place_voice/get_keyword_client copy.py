import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
# 자체 서비스 인터페이스 생성 모듈
# from llm_node_pkg.srv import TriggerLLM



############ get_keyword_client (클라이언트) Node ############
class GetKeywordClient(Node):
    def __init__(self):
        super().__init__('get_keyword_client')
        
        # get_keyword_client 생성 (get_keyword 서비스 받기)
        self.cli = self.create_client(Trigger, 'get_keyword')
        # get_keyword_server 준비 대기
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for get_keyword_server...")
    
        # service 요청 객체 생성 및 service 호출
        self.req = Trigger.Request()
        self.call_service()


    ### server에 service 요청
    def call_service(self):
        self.get_logger().info("Calling service...")
        # 비동기 호출
        future = self.cli.call_async(self.req)
        future.add_done_callback(self.service_callback)
        self.get_logger().info("Service called, waiting for response(wakeupword, stt)...")
    
    ### service 응답 콜백
    def service_callback(self, future):
        try:
            res = future.result()

            # object나 action이 None인 경우 경고
            tables = []
            actions = []
            
            if res.message:
                table_action_pairs = res.message.split()
                for pair in table_action_pairs:
                    if ":" in pair:
                        table, action = pair.split(":")
                        tables.append(table)
                        actions.append(action)
            
            if tables and actions:
                objects_str = " ".join(tables)
                actions_str = " ".join(actions)
            else:
                objects_str = "NONE"
                actions_str = "NONE"

            if not tables or tables == []:
                self.get_logger().warn("Warning: No tables detected (object is empty)")
            if not actions or all(a == "NONE" for a in actions):
                self.get_logger().warn("Warning: Action is NONE (not clearly specified)")

            # service 응답 로그
            self.get_logger().info(
                f"[Service response] success={res.success}, object={objects_str}, action={actions_str}"
            )
            
            # service 재호출
            self.call_service()
            self.get_logger().info("Next Service waiting...")

        except Exception as e:
            self.get_logger().error(f"Error: Service call failed: {e}")


### get_keyword_client.py 실행
def main(args=None):
    rclpy.init(args=args)
    node = GetKeywordClient()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()