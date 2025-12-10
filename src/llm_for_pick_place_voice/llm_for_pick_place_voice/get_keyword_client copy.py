import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from yolo_depth_interfaces.srv import MoveToPoint
from interface_pkg.srv import GetTargetPoint, StringToString
from geometry_msgs.msg import Point
from std_msgs.msg import String
import json
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
        
        self.cli = self.create_client(Trigger, '/get_keyword')
        self.get_target_point_client = self.create_client(Trigger, '/get_target_point')
        self.move_to_point_client = self.create_client(StringToString, '/move_to_point')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for get_keyword service...")
        while not self.get_target_point_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for get_target_point service...")
        while not self.move_to_point_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for move_to_point service...")
        
        # target_object_class 토픽 구독
        self.class_subscriber = self.create_subscription(
            String,
            '/target_object_class',
            self.class_callback,
            10
        )
        self.latest_class = None  # 최근 감지된 클래스
        
        self.req = Trigger.Request()
        self.latest_keyword = None
        
        self.get_logger().info("GetKeywordClient initialized.")
    
    def class_callback(self, msg):
        """/target_object_class 토픽 콜백"""
        self.latest_class = msg.data
        self.get_logger().debug(f"Received class: {self.latest_class}")
    
    def process_keyword(self):
        """get_keyword 서비스 호출 및 응답 처리"""
        self.get_logger().info("Calling get_keyword service...")
        get_keyword_future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, get_keyword_future)

        try:
            res = get_keyword_future.result()

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
            
            if self.latest_class != "None":
                # get_target_point 서비스를 반복적으로 호출
                # "None"이 올 때까지 계속 시도
                while rclpy.ok():
                    
                    # get_target_point 서비스 호출
                    target_point_response = self.call_get_target_point()
                    if target_point_response is not None:
                        self.get_logger().info(f"Got target point response, waiting for class detection...")
                        
                        # "None"이 올 때까지 대기
                        timeout = time.time() + 5.0  # 5초 타임아웃
                        while rclpy.ok() and time.time() < timeout:
                            if self.latest_class == "None":
                                self.get_logger().info("Received None class, exiting loop")
                                break
                            time.sleep(0.1)
                        
                        # "None"을 받으면 반복 종료
                        if self.latest_class == "None":
                            break
                        
                        # 아직 데이터가 있으면 1초 대기 후 재시도
                        self.get_logger().info("Class detection still active, retrying in 1 second...")
                        time.sleep(1.0)
                    else:
                        # 서비스 호출 실패 시 대기 후 재시도
                        self.get_logger().warn("Failed to get target point, retrying...")
                        time.sleep(1.0)
            
        except Exception as e:
            self.get_logger().error(f"Error calling get_keyword service: {e}")
    
    def call_get_target_point(self):
        """/get_target_point 서비스 호출 및 응답 반환"""
        self.get_logger().info("Calling /get_target_point service...")
        get_target_point_future = self.get_target_point_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, get_target_point_future)

        try:
            res = get_target_point_future.result()
            # Point를 MoveToPoint 서비스로 전달 (res가 Point 타입이 아니면 적절히 수정 필요)
            self.forward_point_to_move_to_point(res)
            return res
        except Exception as e:
            self.get_logger().error(f"Error calling /get_target_point service: {e}")
            return None
    
    def call_trigger_action(self):
        """trigger_action 서비스 호출 및 응답 반환"""
        self.get_logger().info("Calling trigger_action service...")
        trigger_action_future = self.trigger_action_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, trigger_action_future)

        try:
            res = trigger_action_future.result()
            if res.success:
                self.get_logger().info(f"trigger_action service successfully triggered. Response: {res.message}")

                # 응답 메시지를 다른 노드로 전달
                self.forward_to_other_node(res.message)
                
                # 응답값 반환 (None인지 확인하기 위해)
                return res.message if res.message else None
            else:
                self.get_logger().warn(f"trigger_action service call failed. Response: {res.message}")
                return None
        except Exception as e:
            self.get_logger().error(f"Error calling trigger_action service: {e}")
            return None

    def forward_point_to_move_to_point(self, point):
        """/get_target_point 서비스로부터 받은 Point를 /move_to_point 서비스로 전달"""
        self.get_logger().info(f"Forwarding point to /move_to_point service: ({getattr(point, 'x', None)}, {getattr(point, 'y', None)}, {getattr(point, 'z', None)})")
        try:
            # StringToString 서비스 요청 생성 (Point를 String으로 변환해서 전달)
            req = StringToString.Request()
            if hasattr(point, 'x') and hasattr(point, 'y') and hasattr(point, 'z'):
                req.input = f"{point.x},{point.y},{point.z}"
            else:
                req.input = str(point)
            future = self.move_to_point_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            self.get_logger().info(f"Move to point response - Success: {getattr(response, 'success', None)}, Message: {getattr(response, 'message', None)}")
        except Exception as e:
            self.get_logger().error(f"Error calling /move_to_point service: {e}")

    def forward_to_other_node(self, message):
        """trigger_action 응답 메시지(JSON)를 /move_to_point 서비스로 전달 (레거시 코드)"""
        self.get_logger().info(f"Forwarding message to /move_to_point service: {message}")
        try:
            # JSON 파싱
            data = json.loads(message)
            # Point 객체 생성
            point = Point()
            point.x = float(data.get('x', 0.0))
            point.y = float(data.get('y', 0.0))
            point.z = float(data.get('z', 0.0))
            self.get_logger().info(f"Created Point - x: {point.x}, y: {point.y}, z: {point.z}")
            # /move_to_point로 전달
            self.forward_point_to_move_to_point(point)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON parsing error: {e}")
        except Exception as e:
            self.get_logger().error(f"Error calling /move_to_point service: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = GetKeywordClient()

    try:
        while rclpy.ok():
            node.process_keyword()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
