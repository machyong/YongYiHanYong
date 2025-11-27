#!/usr/bin/env python3
"""
MoveIt 동작 확인을 위한 테스트 코드
로봇의 MoveIt 연결 상태와 기본 동작을 테스트합니다.
"""

import rclpy
from rclpy.node import Node
from moveit_py import MoveItPy
from moveit_py.planning import PlanningComponent
import sys


class MoveItTestNode(Node):
    def __init__(self):
        super().__init__('moveit_test_node')
        self.get_logger().info('MoveIt 테스트 노드 초기화 중...')
        
        try:
            # MoveIt 초기화
            self.moveit = MoveItPy(node_name="moveit_test")
            self.get_logger().info('✓ MoveIt 초기화 성공')
            
            # Planning Scene 확인
            self.planning_scene = self.moveit.get_planning_scene()
            self.get_logger().info('✓ Planning Scene 접근 가능')
            
            # Robot Model 확인
            self.robot_model = self.moveit.get_robot_model()
            joint_model_groups = self.robot_model.get_joint_model_group_names()
            self.get_logger().info(f'✓ 사용 가능한 Joint Model Groups: {joint_model_groups}')
            
            # 기본 그룹 설정 (일반적으로 "manipulator" 또는 "arm" 사용)
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
                joint_names = self.robot_model.get_joint_model_group(self.group_name).get_active_joint_model_names()
                self.get_logger().info(f'✓ 활성 Joint 목록: {joint_names}')
                
                # 현재 상태 확인
                current_state = self.moveit.get_planning_scene().current_state
                self.get_logger().info('✓ 현재 로봇 상태 확인 가능')
                
                self.get_logger().info('\n=== MoveIt 동작 확인 완료 ===')
                self.get_logger().info('모든 테스트 통과! MoveIt이 정상적으로 작동 중입니다.')
                
            else:
                self.get_logger().error('✗ Joint Model Group을 찾을 수 없습니다.')
                
        except Exception as e:
            self.get_logger().error(f'✗ MoveIt 초기화 실패: {str(e)}')
            self.get_logger().error('MoveIt 설정을 확인하세요.')
            sys.exit(1)
    
    def test_plan_to_pose(self):
        """특정 좌표로 이동 테스트"""
        try:
            self.get_logger().info('\n=== 목표 좌표로 경로 계획 및 실행 테스트 ===')
            
            # 목표 좌표 설정 (x, y, z, roll, pitch, yaw)
            target_x = 0.3  # 미터
            target_y = 0.0  # 미터
            target_z = 0.4  # 미터
            target_roll = 0.0   # 라디안
            target_pitch = 0.0  # 라디안
            target_yaw = 0.0    # 라디안
            
            self.get_logger().info(f'목표 좌표: x={target_x}, y={target_y}, z={target_z}')
            self.get_logger().info(f'목표 자세: roll={target_roll}, pitch={target_pitch}, yaw={target_yaw}')
            
            # 현재 상태를 시작 상태로 설정
            self.planning_component.set_start_state_to_current_state()
            
            # 목표 포즈 설정
            from geometry_msgs.msg import PoseStamped
            import math
            from scipy.spatial.transform import Rotation
            
            pose_goal = PoseStamped()
            pose_goal.header.frame_id = "base_link"  # 또는 "world", "base_footprint" 등
            pose_goal.pose.position.x = target_x
            pose_goal.pose.position.y = target_y
            pose_goal.pose.position.z = target_z
            
            # Roll, Pitch, Yaw를 Quaternion으로 변환
            r = Rotation.from_euler('xyz', [target_roll, target_pitch, target_yaw])
            quat = r.as_quat()  # [x, y, z, w]
            
            pose_goal.pose.orientation.x = quat[0]
            pose_goal.pose.orientation.y = quat[1]
            pose_goal.pose.orientation.z = quat[2]
            pose_goal.pose.orientation.w = quat[3]
            
            self.planning_component.set_goal_state(pose_stamped_msg=pose_goal, pose_link="tool0")  # end-effector 링크명 확인 필요
            
            # 경로 계획
            self.get_logger().info('경로 계획 중...')
            plan_result = self.planning_component.plan()
            
            if plan_result:
                self.get_logger().info('✓ 경로 계획 생성 성공')
                
                # 계획 실행
                self.get_logger().info('계획된 경로 실행 중...')
                execute_result = self.planning_component.execute()
                
                if execute_result:
                    self.get_logger().info('✓ 목표 좌표로 이동 완료!')
                    return True
                else:
                    self.get_logger().error('✗ 경로 실행 실패')
                    return False
            else:
                self.get_logger().error('✗ 경로 계획 생성 실패')
                return False
                
        except Exception as e:
            self.get_logger().error(f'✗ 좌표 이동 테스트 중 오류: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            return False


def main(args=None):
    rclpy.init(args=args)
    
    print("\n" + "="*50)
    print("MoveIt 동작 확인 테스트")
    print("="*50 + "\n")
    
    try:
        node = MoveItTestNode()
        
        # 경로 계획 테스트 (선택사항)
        print("\n경로 계획 테스트를 수행하시겠습니까? (y/n): ", end='')
        # 자동으로 테스트 진행
        node.test_plan_to_pose()
        
        print("\n" + "="*50)
        print("테스트 완료!")
        print("="*50 + "\n")
        
    except KeyboardInterrupt:
        print("\n테스트 중단됨")
    except Exception as e:
        print(f"\n오류 발생: {str(e)}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
