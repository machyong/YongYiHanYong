#!/usr/bin/env python3
"""
MoveIt Joint 값 테스트 코드
특정 joint 각도로 로봇을 이동시켜 MoveIt 동작을 확인합니다.
"""

import rclpy
from rclpy.node import Node
from moveit_py import MoveItPy
from moveit_py.planning import PlanningComponent
import sys


class MoveItJointTestNode(Node):
    def __init__(self):
        super().__init__('moveit_joint_test_node')
        self.get_logger().info('MoveIt Joint 테스트 노드 초기화 중...')
        
        try:
            # MoveIt 초기화
            self.moveit = MoveItPy(node_name="moveit_joint_test")
            self.get_logger().info('✓ MoveIt 초기화 성공')
            
            # Robot Model 확인
            self.robot_model = self.moveit.get_robot_model()
            joint_model_groups = self.robot_model.get_joint_model_group_names()
            self.get_logger().info(f'✓ 사용 가능한 Joint Model Groups: {joint_model_groups}')
            
            # 기본 그룹 설정
            if joint_model_groups:
                self.group_name = joint_model_groups[0]
                self.get_logger().info(f'✓ 선택된 그룹: {self.group_name}')
                
                # Planning Component 생성
                self.planning_component = PlanningComponent(
                    self.group_name, 
                    self.moveit
                )
                self.get_logger().info('✓ Planning Component 생성 성공')
                
                # Joint 정보 확인
                self.joint_names = self.robot_model.get_joint_model_group(
                    self.group_name
                ).get_active_joint_model_names()
                self.get_logger().info(f'✓ 활성 Joint 목록: {self.joint_names}')
                self.get_logger().info(f'✓ Joint 개수: {len(self.joint_names)}')
                
                # 현재 Joint 값 출력
                self.print_current_joint_values()
                
                self.get_logger().info('\n=== MoveIt 초기화 완료 ===\n')
                
            else:
                self.get_logger().error('✗ Joint Model Group을 찾을 수 없습니다.')
                sys.exit(1)
                
        except Exception as e:
            self.get_logger().error(f'✗ MoveIt 초기화 실패: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            sys.exit(1)
    
    def print_current_joint_values(self):
        """현재 Joint 값 출력"""
        try:
            current_state = self.moveit.get_planning_scene().current_state
            self.get_logger().info('\n--- 현재 Joint 값 ---')
            for i, joint_name in enumerate(self.joint_names):
                joint_value = current_state.get_joint_positions(joint_name)
                if joint_value:
                    self.get_logger().info(f'{joint_name}: {joint_value[0]:.4f} rad ({joint_value[0] * 57.2958:.2f}°)')
            self.get_logger().info('-------------------\n')
        except Exception as e:
            self.get_logger().warn(f'현재 Joint 값 확인 실패: {str(e)}')
    
    def test_move_to_joint_values(self, joint_values):
        """특정 Joint 값으로 이동 테스트"""
        try:
            self.get_logger().info('\n=== Joint 값으로 이동 테스트 시작 ===')
            
            # Joint 값 검증
            if len(joint_values) != len(self.joint_names):
                self.get_logger().error(
                    f'✗ Joint 값 개수 불일치: 입력={len(joint_values)}, 필요={len(self.joint_names)}'
                )
                return False
            
            # 목표 Joint 값 출력
            self.get_logger().info('\n--- 목표 Joint 값 ---')
            for i, (joint_name, value) in enumerate(zip(self.joint_names, joint_values)):
                self.get_logger().info(f'{joint_name}: {value:.4f} rad ({value * 57.2958:.2f}°)')
            self.get_logger().info('-------------------\n')
            
            # 현재 상태를 시작 상태로 설정
            self.planning_component.set_start_state_to_current_state()
            
            # 목표 Joint 값 설정
            robot_state = self.moveit.get_planning_scene().current_state
            robot_state.joint_positions = dict(zip(self.joint_names, joint_values))
            
            self.planning_component.set_goal_state(robot_state=robot_state)
            
            # 경로 계획
            self.get_logger().info('경로 계획 중...')
            plan_result = self.planning_component.plan()
            
            if plan_result:
                self.get_logger().info('✓ 경로 계획 생성 성공')
                
                # 계획 실행
                self.get_logger().info('계획된 경로 실행 중...')
                execute_result = self.planning_component.execute()
                
                if execute_result:
                    self.get_logger().info('✓ 목표 Joint 값으로 이동 완료!')
                    
                    # 이동 후 현재 값 확인
                    import time
                    time.sleep(0.5)
                    self.print_current_joint_values()
                    return True
                else:
                    self.get_logger().error('✗ 경로 실행 실패')
                    return False
            else:
                self.get_logger().error('✗ 경로 계획 생성 실패')
                return False
                
        except Exception as e:
            self.get_logger().error(f'✗ Joint 이동 테스트 중 오류: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            return False
    
    def test_home_position(self):
        """Home 포지션으로 이동 (모든 Joint를 0으로)"""
        home_joints = [0.0] * len(self.joint_names)
        self.get_logger().info('Home 포지션으로 이동 (모든 Joint = 0)')
        return self.test_move_to_joint_values(home_joints)


def main(args=None):
    rclpy.init(args=args)
    
    print("\n" + "="*60)
    print("MoveIt Joint 값 테스트")
    print("="*60 + "\n")
    
    try:
        node = MoveItJointTestNode()
        
        # ========================================
        # 여기에서 목표 Joint 값을 설정하세요
        # ========================================
        
        # 예시 1: Home 포지션 (모든 Joint = 0)
        # target_joint_values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # 예시 2: 커스텀 Joint 값 (라디안 단위)
        # 6축 로봇 예시
        target_joint_values = [
            0.0,      # Joint 1
            -0.785,   # Joint 2 (-45도)
            1.571,    # Joint 3 (90도)
            0.0,      # Joint 4
            0.785,    # Joint 5 (45도)
            0.0       # Joint 6
        ]
        
        # 참고: 각도를 라디안으로 변환
        # 라디안 = 도 * (π / 180) = 도 * 0.0174533
        # 예: 45도 = 0.785 라디안, 90도 = 1.571 라디안
        
        print("\n목표 Joint 값으로 이동을 시작합니다...")
        print("주의: 로봇이 실제로 움직입니다!\n")
        
        # Joint 개수 자동 조정
        if len(target_joint_values) != len(node.joint_names):
            print(f"경고: Joint 값 개수를 {len(node.joint_names)}개로 조정합니다.")
            target_joint_values = target_joint_values[:len(node.joint_names)]
            if len(target_joint_values) < len(node.joint_names):
                target_joint_values += [0.0] * (len(node.joint_names) - len(target_joint_values))
        
        # 이동 테스트 실행
        success = node.test_move_to_joint_values(target_joint_values)
        
        if success:
            print("\n" + "="*60)
            print("테스트 성공!")
            print("="*60 + "\n")
        else:
            print("\n" + "="*60)
            print("테스트 실패")
            print("="*60 + "\n")
        
        # Home 포지션으로 복귀 테스트 (옵션)
        # print("\n5초 후 Home 포지션으로 복귀합니다...")
        # import time
        # time.sleep(5)
        # node.test_home_position()
        
    except KeyboardInterrupt:
        print("\n테스트 중단됨")
    except Exception as e:
        print(f"\n오류 발생: {str(e)}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
