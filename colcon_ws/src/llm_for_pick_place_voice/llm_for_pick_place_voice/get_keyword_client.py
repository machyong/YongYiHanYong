import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from interface_pkg.srv import StringToString
from std_msgs.msg import String
import json
import time


class GetKeywordClient(Node):
    """
    음성 → 키워드 → 타겟 좌표 → RL 이동 → 일반 이동
    """

    def __init__(self):
        super().__init__('get_keyword_client')

        # -------------------------------
        # 서비스 클라이언트 정의
        # -------------------------------
        self.cli = self.create_client(Trigger, 'get_keyword')
        self.get_target_point_client = self.create_client(Trigger, '/get_target_point')

        # RL 이동 서비스
        self.move_to_point_rl_client = self.create_client(StringToString, '/move_to_point_rl')

        # 테이블 이동 서비스
        self.move_to_point_client = self.create_client(StringToString, '/move_to_point')

        # RL 이동 서비스2
        self.move_to_box_rl_client = self.create_client(StringToString, '/move_to_box_rl')

        # 수거함 이동 서비스
        self.move_to_box_client = self.create_client(StringToString, '/move_to_box')

        # 상태 퍼블리셔
        self.publisher_ = self.create_publisher(String, '/robot_status', 10)
        self.robot_status = "waiting"
        self.timer = self.create_timer(0.5, self.robot_status_publisher)

        #로봇 리턴 퍼블리셔
        self.robot_command_pub = self.create_publisher(String, '/robot_command', 10)
        
        # 서비스 대기
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for get_keyword service...")

        while not self.get_target_point_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for get_target_point service...")

        while not self.move_to_point_rl_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /move_to_point_rl service...")

        while not self.move_to_point_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /move_to_point service...")

        # 클래스 구독
        self.class_subscriber = self.create_subscription(
            String,
            '/target_object_class',
            self.class_callback,
            10,
        )

        self.latest_class = None
        self.req = Trigger.Request()
        self.latest_keyword = None

        self.get_logger().info("GetKeywordClient initialized.")

    def robot_status_publisher(self):
        msg = String()
        msg.data = self.robot_status
        self.publisher_.publish(msg)

    def class_callback(self, msg: String):
        self.latest_class = msg.data

    def process_keyword(self):
        """get_keyword 서비스 호출"""
        self.get_logger().info("Calling get_keyword service...")
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)

        try:
            res = future.result()

            tables = []
            actions = []
            if res.message:
                items = res.message.split()
                for pair in items:
                    if ":" in pair:
                        t, a = pair.split(":")
                        tables.append(t)
                        actions.append(a)

            self.latest_keyword = {
                "tables": tables,
                "actions": actions,
                "message": res.message,
            }

            if self.latest_class != "None":
                self.robot_status = "moving"

                while rclpy.ok():
                    rclpy.spin_once(self, timeout_sec=0.1)

                    if self.latest_class == "None":
                        self.get_logger().info("Received None class → stop loop")
                        break

                    coord_str = self.call_get_target_point()
                    if coord_str is None:
                        continue

                    self.forward_string_to_move_to_point(coord_str)

            self.robot_status = "returning"

            cmd = String()
            cmd.data = json.dumps({
                "type": "keyword",
                "tables": ["table0"],
                "command": "return",
                "text": "퇴식구로 이동합니다",
            }, ensure_ascii=False)

            self.robot_command_pub.publish(cmd)
            self.get_logger().info("Published robot_command: return_to_home")

        except Exception as e:
            self.get_logger().error(f"Error calling get_keyword: {e}")

    def call_get_target_point(self):
        self.get_logger().info("Calling /get_target_point...")
        future = self.get_target_point_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)

        try:
            res = future.result()
            if res.success and res.message:
                return res.message
            else:
                return None
        except Exception as e:
            self.get_logger().error(f"Error in /get_target_point: {e}")
            return None

    def forward_string_to_move_to_point(self, coord_str: str):
        """
        1) /move_to_point_rl(coord_str)
        2) RL 응답 그대로 → /move_to_point(forward_str)
        3) /move_to_box_rl → RL 결과 → /move_to_box
        """

        # ============================================
        # STEP 1 — RL 서비스 호출 (move_to_point_rl)
        # ============================================
        self.get_logger().info(f"[1] Sending to /move_to_point_rl: '{coord_str}'")

        try:
            rl_req = StringToString.Request()
            rl_req.input = coord_str

            future_rl = self.move_to_point_rl_client.call_async(rl_req)
            rclpy.spin_until_future_complete(self, future_rl)
            res_rl = future_rl.result()

            self.get_logger().info(f"[RL Response] {res_rl}")

            if hasattr(res_rl, "output") and res_rl.output:
                forward_str = res_rl.output
            elif hasattr(res_rl, "message") and res_rl.message:
                forward_str = res_rl.message
            else:
                forward_str = coord_str

        except Exception as e:
            self.get_logger().error(f"Error in RL call: {e}")
            return

        # ============================================
        # STEP 2 — RL 결과 → move_to_point 전달
        # ============================================
        self.get_logger().info(f"[2] Forwarding to /move_to_point: '{forward_str}'")

        try:
            req_final = StringToString.Request()
            req_final.input = forward_str

            future_final = self.move_to_point_client.call_async(req_final)
            rclpy.spin_until_future_complete(self, future_final)

            res_final = future_final.result()

            if hasattr(res_final, "output") and res_final.output:
                second_str = res_final.output
            elif hasattr(res_final, "message") and res_final.message:
                second_str = res_final.message
            else:
                second_str = coord_str

        except Exception as e:
            self.get_logger().error(f"Error calling /move_to_point: {e}")
            return

        # ============================================
        # STEP 3 — RL 서비스 호출 (move_to_box_rl)
        # ============================================
        self.get_logger().info(f"[3] Sending to /move_to_box_rl: '{second_str}'")

        try:
            rl_box = StringToString.Request()
            rl_box.input = second_str

            future_box_rl = self.move_to_box_rl_client.call_async(rl_box)
            rclpy.spin_until_future_complete(self, future_box_rl)

            res_box_rl = future_box_rl.result()
            self.get_logger().info(f"[RL Box Response] {res_box_rl}")

            if hasattr(res_box_rl, "output") and res_box_rl.output:
                third_str = res_box_rl.output
            elif hasattr(res_box_rl, "message") and res_box_rl.message:
                third_str = res_box_rl.message
            else:
                third_str = second_str

        except Exception as e:
            self.get_logger().error(f"Error in /move_to_box_rl: {e}")
            return

        # ============================================
        # STEP 4 — RL 결과 → move_to_box 호출 (수정됨)
        # ============================================
        self.get_logger().info(f"[4] Forwarding to /move_to_box: '{third_str}'")

        try:
            req_box = StringToString.Request()
            req_box.input = third_str

            future_box = self.move_to_box_client.call_async(req_box)
            rclpy.spin_until_future_complete(self, future_box)

            res_box_final = future_box.result()
            self.get_logger().info(f"[MoveToBox Response] {res_box_final}")

        except Exception as e:
            self.get_logger().error(f"Error calling /move_to_box: {e}")


        


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
