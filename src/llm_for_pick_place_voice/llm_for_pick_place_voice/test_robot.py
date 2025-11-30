import rclpy
from rclpy.node import Node
from interface_pkg.srv import StringToString
import json
import math
import time
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class TestRobotNode(Node):
    """
    forward_service 서버: 다른 노드에서 전달받은 메시지(JSON 형식의 좌표)를 수신하고
    로봇을 해당 좌표로 움직인 후 '수신 완료'로 응답
    """
    def __init__(self):
        super().__init__('test_robot_node')
        
        # forward_service 서버 생성
        self.srv = self.create_service(StringToString, 'forward_service', self.handle_forward_service)
        self.get_logger().info('TestRobotNode is ready. Waiting for forward_service requests...')
        
        # 로봇 제어용 Action 클라이언트
        self.action_name = "/dsr_moveit_controller/follow_joint_trajectory"
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.action_name
        )
        
        # 관절 이름 (Doosan 로봇)
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
        self.get_logger().info(f"Action server: {self.action_name}")
    
    def handle_forward_service(self, request, response):
        """forward_service 요청 처리"""
        self.get_logger().info(f"Received message from forward_service: {request.input}")
        
        try:
            # JSON 파싱
            data = json.loads(request.input)
            
            self.get_logger().info(f"Parsed data: {data}")
            self.get_logger().info(f"Object class: {data['class']}")
            self.get_logger().info(f"Target position: x={data['x']}, y={data['y']}, z={data['z']}")
            self.get_logger().info(f"Angle: {data['angle_deg']}°")
            
            # 좌표로 로봇 이동
            x = float(data['x'])
            y = float(data['y'])
            z = float(data['z'])
            angle_deg = float(data['angle_deg'])
            
            # 간단한 역기구학: 좌표를 관절 각도로 변환
            joint_values = self._cartesian_to_joint(x, y, z, angle_deg)
            
            if joint_values is not None:
                # 로봇 이동
                self.get_logger().info(f"Moving robot to: {joint_values}")
                success = self.move_to_joint_goal(joint_values, duration=3.0)
                
                if success:
                    # 이동 완료 대기
                    time.sleep(4)
                    response.success = True
                    response.output = '수신 완료'
                    self.get_logger().info(f"Sent response: {response.output}")
                else:
                    response.success = False
                    response.output = '로봇 이동 실패'
            else:
                response.success = False
                response.output = '역기구학 계산 실패'
        
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON parsing error: {e}")
            response.success = False
            response.output = 'JSON 파싱 오류'
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            response.success = False
            response.output = f'오류: {str(e)}'
        
        return response
    
    def _cartesian_to_joint(self, x, y, z, angle_deg):
        """
        카르테시안 좌표를 관절 각도로 변환 (6-DOF 역기구학)
        
        Args:
            x, y, z: 위치 (미터)
            angle_deg: 각도 (도)
        
        Returns:
            list: 관절 각도 [rad]
        """
        try:
            # DH 파라미터 (Doosan 로봇 M0609)
            # 실제 값은 로봇 스펙에 맞게 조정 필요
            L1 = 0.15   # Link 1 길이
            L2 = 0.27   # Link 2 길이
            L3 = 0.27   # Link 3 길이
            L4 = 0.20   # Link 4 길이 (wrist)
            L5 = 0.10   # Link 5 길이
            L6 = 0.08   # Link 6 길이 (end-effector)
            
            # 각도 변환
            angle_rad = math.radians(angle_deg)
            
            # ============================================
            # Joint 1: Base rotation (xy 평면에서의 회전)
            # ============================================
            joint_1 = math.atan2(y, x)
            
            # ============================================
            # Joint 2, 3, 4: Arm kinematics
            # ============================================
            
            # Base에서 end-effector까지의 거리 (xy 평면)
            rho = math.sqrt(x**2 + y**2)
            
            # End-effector의 높이를 고려한 거리
            # End-effector까지의 거리 = sqrt(rho^2 + z^2)
            distance_3d = math.sqrt(rho**2 + z**2)
            
            # Wrist center까지의 거리 (L6만큼 뒤로)
            distance_wrist = distance_3d - L6
            
            # Z축 방향 각도 (pitch)
            alpha = math.atan2(z, rho)
            
            if distance_wrist <= 0:
                self.get_logger().warn("Target too close to base")
                return None
            
            # Law of cosines: L2와 L3 + L4로 역기구학 계산
            L23 = L2 + L3
            
            # cos(elbow_angle) = (L2^2 + (L3+L4)^2 - distance_wrist^2) / (2 * L2 * (L3+L4))
            cos_elbow = (L2**2 + L23**2 - distance_wrist**2) / (2 * L2 * L23 + 1e-6)
            
            # 범위 제한 [-1, 1]
            cos_elbow = max(-1, min(1, cos_elbow))
            elbow_angle = math.acos(cos_elbow)
            
            # Joint 2와 Joint 3 계산
            # shoulder_angle = atan2(z, rho) - atan2(L23*sin(elbow), L2 + L23*cos(elbow))
            sin_elbow = math.sin(elbow_angle)
            cos_elbow_val = math.cos(elbow_angle)
            
            numerator = L23 * sin_elbow
            denominator = L2 + L23 * cos_elbow_val
            
            shoulder_offset = math.atan2(numerator, denominator)
            
            joint_2 = alpha - shoulder_offset
            joint_3 = elbow_angle - math.pi  # Elbow joint 각도
            
            # ============================================
            # Joint 4, 5, 6: Wrist orientation
            # ============================================
            
            # 엔드이펙터가 수직으로 내려가도록 설정
            # x축 회전 = π (180도) → 엔드이펙터 수직 아래로
            # y축 회전 = 0
            # z축 회전 = angle_deg (카메라 각도)
            
            # 오일러 각도로 쿼터니언 계산
            roll = math.pi  # x축 회전 180도 (수직)
            pitch = 0.0  # y축 회전 0도
            yaw = angle_rad  # z축 회전 (각도 정보 활용)
            
            # 쿼터니언 계산 (ZYX 오일러)
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            cp = math.cos(pitch * 0.5)
            sp = math.sin(pitch * 0.5)
            cr = math.cos(roll * 0.5)
            sr = math.sin(roll * 0.5)
            
            qx = sr * cp * cy - cr * sp * sy
            qy = cr * sp * cy + sr * cp * sy
            qz = cr * cp * sy - sr * sp * cy
            qw = cr * cp * cy + sr * sp * sy
            
            # 쿼터니언에서 조인트 4, 5, 6 계산
            # 간단한 근사: 쿼터니언 성분을 직접 사용
            joint_4 = roll  # Roll (x축)
            joint_5 = pitch  # Pitch (y축)
            joint_6 = yaw   # Yaw (z축)
            
            # ============================================
            # 조인트 각도 정규화 (±π 범위)
            # ============================================
            joint_values = [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
            
            # 각도를 정규화
            for i in range(len(joint_values)):
                # ±π 범위로 조정
                while joint_values[i] > math.pi:
                    joint_values[i] -= 2 * math.pi
                while joint_values[i] < -math.pi:
                    joint_values[i] += 2 * math.pi
            
            self.get_logger().info(f"Distance to target: {distance_3d:.4f} m")
            self.get_logger().info(f"Distance to wrist center: {distance_wrist:.4f} m")
            self.get_logger().info(f"Alpha (Z angle): {math.degrees(alpha):.2f}°")
            self.get_logger().info(f"Elbow angle: {math.degrees(elbow_angle):.2f}°")
            self.get_logger().info(f"Calculated joint angles (rad): {[f'{v:.4f}' for v in joint_values]}")
            self.get_logger().info(f"Calculated joint angles (deg): {[f'{math.degrees(v):.2f}°' for v in joint_values]}")
            
            return joint_values
        
        except Exception as e:
            self.get_logger().error(f"Error in cartesian_to_joint: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    def move_to_joint_goal(self, joint_values, duration=5.0):
        """
        관절 목표 위치로 이동
        
        Args:
            joint_values (list): 관절 각도 (라디안)
            duration (float): 이동 시간 (초)
        
        Returns:
            bool: 성공 여부
        """
        try:
            self.get_logger().info(f"Moving to joint goal: {[f'{v:.4f}' for v in joint_values]}")
            self.get_logger().info(f"Duration: {duration}s")
            
            # 궤적 생성
            trajectory = self._create_trajectory(joint_values, duration)
            
            # Action 전송
            if not self._action_client.wait_for_server(timeout_sec=10):
                self.get_logger().error(f"Action server not available: {self.action_name}")
                return False
            
            # Goal 생성
            goal_msg = FollowJointTrajectory.Goal()
            goal_msg.trajectory = trajectory
            
            # Goal 전송
            self.get_logger().info("Sending trajectory goal...")
            self._send_goal_future = self._action_client.send_goal_async(goal_msg)
            self._send_goal_future.add_done_callback(self._goal_response_callback)
            
            return True
        
        except Exception as e:
            self.get_logger().error(f"Error in move_to_joint_goal: {e}")
            return False
    
    def _create_trajectory(self, joint_values, duration):
        """
        JointTrajectory 메시지 생성
        """
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        # 궤적 포인트 생성
        point = JointTrajectoryPoint()
        point.positions = joint_values
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        
        trajectory.points = [point]
        
        return trajectory
    
    def _goal_response_callback(self, future):
        """
        Goal 응답 콜백
        """
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Goal rejected by action server")
                return
            
            self.get_logger().info("✓ Goal accepted by action server")
            
            # 결과 대기
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self._get_result_callback)
        except Exception as e:
            self.get_logger().error(f"Error in goal response: {e}")
    
    def _get_result_callback(self, future):
        """
        결과 콜백
        """
        try:
            result = future.result().result
            self.get_logger().info("✓ Motion completed successfully")
        except Exception as e:
            self.get_logger().error(f"Motion failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TestRobotNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
