import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import time


class GetKeywordClient(Node):
    """
    get_keyword 서비스를 호출하고 응답을 다른 노드로 전달하는 클라이언트
    
    흐름: 
    1. get_keyword 서비스 호출
    2. 서버가 wakeup word 감지 → STT → 키워드 추출
    3. 응답 받음 (table:action 형식)
    4. 다른 노드로 전달 (TODO)
    5. 반복
    """
    def __init__(self):
        super().__init__('get_keyword_client')
        
        self.cli = self.create_client(Trigger, 'get_keyword')
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for get_keyword service...")
        
        self.req = Trigger.Request()
        self.latest_keyword = None
        
        self.get_logger().info("GetKeywordClient initialized.")
        self.call_service()
    
    def call_service(self):
        """get_keyword 서비스 호출"""
        self.get_logger().info("Calling get_keyword service...")
        future = self.cli.call_async(self.req)
        future.add_done_callback(self.service_callback)
    
    def service_callback(self, future):
        """get_keyword 서비스 응답 처리"""
        try:
            res = future.result()
            
            tables = []
            actions = []
            
            if res.message:
                table_action_pairs = res.message.split()
                for pair in table_action_pairs:
                    if ":" in pair:
                        table, action = pair.split(":")
                        tables.append(table)
                        actions.append(action)
            
            objects_str = " ".join(tables) if tables else "NONE"
            actions_str = " ".join(actions) if actions else "NONE"
            
            if not tables:
                self.get_logger().warn("Warning: No tables detected")
            if not actions or all(a == "NONE" for a in actions):
                self.get_logger().warn("Warning: Action is NONE")
            
            self.get_logger().info(
                f"[Response] success={res.success}, tables={objects_str}, actions={actions_str}"
            )
            
            self.latest_keyword = {
                "tables": tables,
                "actions": actions,
                "message": res.message
            }
            
            if res.success and tables and actions:
                self.forward_to_other_node(tables, actions)
            
            # 다음 서비스 호출
            self.call_service()
            
        except Exception as e:
            self.get_logger().error(f"Error: Service call failed: {e}")
            time.sleep(1.0)
            self.call_service()
    
    def forward_to_other_node(self, tables, actions):
        """
        받은 키워드를 로봇 노드로 전달 (TODO: 구현 필요)
        
        예시:
        req = YourServiceType.Request()
        req.tables = tables
        req.actions = actions
        future = self.your_client.call_async(req)
        future.add_done_callback(self.forward_response_callback)
        """
        self.get_logger().info(f"TODO: Forward to other node - tables={tables}, actions={actions}")
    
    def forward_response_callback(self, future):
        """로봇 노드 응답 처리"""
        try:
            response = future.result()
            self.get_logger().info(f"Forward response: {response}")
        except Exception as e:
            self.get_logger().error(f"Forward failed: {e}")
def main(args=None):
    rclpy.init(args=args)
    node = GetKeywordClient()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
