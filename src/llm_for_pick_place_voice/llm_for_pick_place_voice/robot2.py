"""여기에 두산 api를 써도 되고 moveit을 써도 됩니다."""
"""DOASN API를 사용한 예시로 서비스를 사용한 알고리즘으로 작성한 코드입니다."""

import rclpy
from rclpy.node import Node
from llm_for_pick_place_voice.srv import KeywordCommand
import ast
import math

# Doosan Robotics API imports
try:
    import DRCF
    from DRCF import *
except ImportError:
    print("Warning: Doosan Robotics API (DRCF) not found. Using mock implementation.")


class DoosanRobotAPIController(Node):
    def __init__(self):
        super().__init__('doosan_robot_api_controller')
        
        # 서비스 서버 생성 - get_keyword_client로부터 서비스 요청 받음
        self.srv = self.create_service(
            KeywordCommand,
            'robot_control_service',
            self.service_callback
        )
        
        # Doosan Robot 연결 설정
        self.robot_id = "dsr01"
        self.robot_model = "e0509"
        self.robot_ip = "192.168.137.100"  # 실제 로봇 IP로 변경 필요
        
        # 로봇 초기화
        self.initialize_robot()
        
        self.get_logger().info('Doosan Robot API Controller initialized')
        self.get_logger().info(f'Service ready: /robot_control_service')
    
    def initialize_robot(self):
        """Doosan 로봇 초기화"""
        try:
            # 로봇 연결
            self.get_logger().info(f'Connecting to robot at {self.robot_ip}...')
            
            # Doosan API 초기화
            # SetSafetyLevel(SAFETY_LEVEL)
            # SetRobotMode(ROBOT_MODE_AUTONOMOUS)
            
            self.get_logger().info('Robot initialized successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize robot: {e}')
    
    def service_callback(self, request, response):
        """get_keyword_client로부터 서비스 요청 처리"""
        try:
            # String 서비스 데이터를 파싱
            command_dict = ast.literal_eval(request.command)
            
            self.get_logger().info(f'Received service request: {command_dict}')
            
            # 데이터 추출
            object_name = command_dict.get('object', '')
            x = command_dict.get('x', 0)
            y = command_dict.get('y', 0)
            z = command_dict.get('z', 0)
            angle = command_dict.get('angle', 0)
            
            # 로봇 이동 실행
            success = self.move_to_position(x, y, z, angle, object_name)
            
            # 응답 설정
            response.success = success
            if success:
                response.message = f'Successfully moved to pick {object_name}'
            else:
                response.message = f'Failed to move to pick {object_name}'
            
        except Exception as e:
            self.get_logger().error(f'Failed to process service request: {e}')
            response.success = False
            response.message = f'Error: {str(e)}'
        
        return response
    
    def move_to_position(self, x, y, z, angle, object_name):
        """Doosan API를 사용하여 로봇을 목표 위치로 이동"""
        try:
            self.get_logger().info(
                f'Moving to pick {object_name} at position: x={x}, y={y}, z={z}, angle={angle}'
            )
            
            # cm를 mm로 변환 (Doosan API는 mm 단위 사용)
            x_mm = x * 10.0
            y_mm = y * 10.0
            z_mm = z * 10.0
            
            # 각도를 라디안으로 변환
            angle_rad = math.radians(angle)
            
            # 목표 위치 설정 (X, Y, Z, Rx, Ry, Rz)
            # Doosan 좌표계: [x, y, z, rx, ry, rz]
            target_pos = [
                x_mm,      # X 위치 (mm)
                y_mm,      # Y 위치 (mm)
                z_mm,      # Z 위치 (mm)
                0.0,       # Roll (Rx)
                0.0,       # Pitch (Ry)
                angle_rad  # Yaw (Rz) - Z축 회전
            ]
            
            self.get_logger().info(f'Target position: {target_pos}')
            
            # ===== Doosan API 명령 사용 예시 =====
            
            # 1. 직선 이동 (MoveL)
            # MoveL(target_pos, vel=[50, 50], acc=[100, 100])
            
            # 2. 관절 이동 (MoveJ)
            # MoveJ(target_pos, vel=50, acc=100)

            
            # 4. Pick & Place 시퀀스
            self.pick_and_place_sequence(target_pos, object_name)
            
            self.get_logger().info('Motion completed successfully!')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to move robot: {e}')
            return False
    
    def pick_and_place_sequence(self, target_pos, object_name):
        """Pick and Place 시퀀스 실행"""
        try:
            # 안전 높이 설정
            safe_height_offset = 100.0  # mm
            
            # 1. 접근 위치로 이동 (물체 위 안전 높이)
            approach_pos = target_pos.copy()
            approach_pos[2] += safe_height_offset
            
            self.get_logger().info(f'Step 1: Moving to approach position')
            # MoveL(approach_pos, vel=[50, 50], acc=[100, 100])
            
            # 2. 그리퍼 열기
            self.get_logger().info(f'Step 2: Opening gripper')
            # SetDigitalOutput(1, True)  # 그리퍼 열기 신호
            # Wait(0.5)  # 0.5초 대기
            
            # 3. 물체 위치로 하강
            self.get_logger().info(f'Step 3: Descending to object')
            # MoveL(target_pos, vel=[30, 30], acc=[80, 80])
            
            # 4. 그리퍼 닫기 (물체 잡기)
            self.get_logger().info(f'Step 4: Grasping {object_name}')
            # SetDigitalOutput(1, False)  # 그리퍼 닫기 신호
            # Wait(0.5)  # 0.5초 대기
            
            # 5. 안전 높이로 상승
            self.get_logger().info(f'Step 5: Lifting object')
            # MoveL(approach_pos, vel=[30, 30], acc=[80, 80])
            
            # 6. 배치 위치로 이동 (예시: 원래 위치에서 200mm 오른쪽)
            place_pos = approach_pos.copy()
            place_pos[0] += 200.0  # X축으로 200mm 이동
            
            self.get_logger().info(f'Step 6: Moving to place position')
            # MoveL(place_pos, vel=[50, 50], acc=[100, 100])
            
            # 7. 하강
            place_pos[2] -= safe_height_offset
            self.get_logger().info(f'Step 7: Descending to place')
            # MoveL(place_pos, vel=[30, 30], acc=[80, 80])
            
            # 8. 그리퍼 열기 (물체 놓기)
            self.get_logger().info(f'Step 8: Releasing {object_name}')
            # SetDigitalOutput(1, True)  # 그리퍼 열기 신호
            # Wait(0.5)  # 0.5초 대기
            
            # 9. 안전 높이로 상승
            place_pos[2] += safe_height_offset
            self.get_logger().info(f'Step 9: Retracting')
            # MoveL(place_pos, vel=[30, 30], acc=[80, 80])
            
            # 10. 홈 위치로 복귀 (옵션)
            self.get_logger().info(f'Step 10: Returning to home')
            # MoveJHome()
            
            self.get_logger().info(f'Pick and place sequence completed for {object_name}')
            
        except Exception as e:
            self.get_logger().error(f'Pick and place sequence failed: {e}')
            raise
    
    def move_to_home(self):
        """홈 위치로 이동"""
        try:
            self.get_logger().info('Moving to home position')
            # MoveJHome()
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to move to home: {e}')
            return False
    
    def stop_robot(self):
        """로봇 정지"""
        try:
            self.get_logger().info('Stopping robot')
            # Stop()
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to stop robot: {e}')
            return False


def main(args=None):
    rclpy.init(args=args)
    
    robot_controller = DoosanRobotAPIController()
    
    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        robot_controller.get_logger().info('Shutting down...')
        robot_controller.stop_robot()
    finally:
        robot_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
