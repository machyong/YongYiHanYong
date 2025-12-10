import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from interface_pkg.srv import StringToString
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
    4. (필요 시) /get_target_point 호출 → 좌표 string 수신
    5. 받은 string을 /move_to_point(StringToString) 서비스로 전달
    6. 반복
    """
    def __init__(self):
        super().__init__('get_keyword_client')

        # -------------------------------
        # 서비스 클라이언트
        # -------------------------------
        # 1) get_keyword : Trigger
        self.cli = self.create_client(Trigger, 'get_keyword')

        # 2) get_target_point : Trigger
        #    → res.message 에 좌표 string 이 들어 있다고 가정
        self.get_target_point_client = self.create_client(Trigger, '/get_target_point')

        # 3) move_to_point : StringToString
        self.move_to_point_client = self.create_client(StringToString, '/move_to_point')

        self.publisher_ = self.create_publisher(String, '/robot_status', 10)

        self.robot_status = "waiting"
        self.timer = self.create_timer(0.5, self.robot_status_publisher)
        # 서비스 대기
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for get_keyword service...")
        while not self.get_target_point_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for get_target_point service...")
        while not self.move_to_point_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for move_to_point service...")
        # -------------------------------
        # /target_object_class 토픽 구독
        # -------------------------------
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
    def robot_status_publisher(self):
        msg = String()
        msg.data = self.robot_status
        self.publisher_.publish(msg)
    def class_callback(self, msg: String):
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

            # latest_class 가 "None" 이 아니라면 → 실제 물체가 감지되고 있다고 판단
            if self.latest_class != "None":
                self.robot_status = "moving"
                # get_target_point 서비스를 반복 호출하면서
                # /target_object_class 가 "None" 이 될 때까지 반복
                while rclpy.ok():

                    # 🔹 executor에게 콜백 처리 기회 제공
                    rclpy.spin_once(self, timeout_sec=0.1)

                    # 카메라 토픽이 None이면 즉시 종료
                    if self.latest_class == "None":
                        self.get_logger().info("Received None class, exiting loop")
                        break

                    # /get_target_point 서비스 호출
                    target_str = self.call_get_target_point()

                    if target_str is None:
                        self.get_logger().warn(
                            "Failed to get target point, will retry on next cycle"
                        )
                        continue

                    self.get_logger().info(
                        f"Got target point string from /get_target_point: {target_str}"
                    )

                    # /move_to_point 서비스로 전달
                    self.forward_string_to_move_to_point(target_str)

                    # 다음 반복으로 바로 넘어감
                    # (latest_class 변화 여부는 spin_once에서 처리됨)
            self.robot_status = "waiting"

        except Exception as e:
            self.get_logger().error(f"Error calling get_keyword service: {e}")

    def call_get_target_point(self):
        """/get_target_point 서비스 호출 및 응답 message(string) 반환"""
        self.get_logger().info("Calling /get_target_point service...")
        future = self.get_target_point_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)

        try:
            res = future.result()
            # Trigger.Response: { bool success, string message }
            if res.success and res.message:
                self.get_logger().info(
                    f"/get_target_point response: success={res.success}, message={res.message}"
                )
                return res.message  # 좌표 string (예: "x,y,z" 또는 JSON)
            else:
                self.get_logger().warn(
                    f"/get_target_point returned empty or failed: success={res.success}, message='{res.message}'"
                )
                return None
        except Exception as e:
            self.get_logger().error(f"Error calling /get_target_point service: {e}")
            return None

    def forward_string_to_move_to_point(self, coord_str: str):
        """
        /get_target_point 에서 받은 좌표 string 을
        /move_to_point(StringToString) 서비스로 그대로 전달
        """
        self.get_logger().info(
            f"Forwarding string to /move_to_point service: '{coord_str}'"
        )
        try:
            req = StringToString.Request()
            req.input = coord_str

            future = self.move_to_point_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()

            # StringToString 정의에 따라 필드만 로깅 (예: {string input, string output} 이라면)
            self.get_logger().info(
                f"MoveToPoint response: {response}"
            )
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
