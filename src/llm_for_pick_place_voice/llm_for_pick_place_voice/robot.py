"""여기에 두산 api를 써도 되고 moveit을 써도 됩니다."""
"""Moveit을 사용한 예시로 토픽을 사용한 알고리즘으로 작성한 코드입니다."""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    JointConstraint
)
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
import ast
import math


class DoosanRobotController(Node):
    def __init__(self):
        super().__init__('doosan_robot_controller')
        
        # get_keyword_client로부터 토픽 구독
        self.subscription = self.create_subscription(
            String,
            'keyword_command',  # 토픽 이름은 실제 환경에 맞게 수정
            self.command_callback,
            10
        )
        
        # MoveIt Action Client 설정
        self._action_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'  # Doosan E0509의 MoveGroup action 이름
        )
        
        self.get_logger().info('Doosan Robot Controller initialized')
        
    def command_callback(self, msg):
        """get_keyword_client로부터 받은 토픽 처리"""
        try:
            # String 토픽 데이터를 파싱
            command_dict = ast.literal_eval(msg.data)
            
            self.get_logger().info(f'Received command: {command_dict}')
            
            # 데이터 추출
            object_name = command_dict.get('object', '')
            x = command_dict.get('x', 0)
            y = command_dict.get('y', 0)
            z = command_dict.get('z', 0)
            angle = command_dict.get('angle', 0)
            
            # 로봇 이동 실행
            self.move_to_position(x, y, z, angle, object_name)
            
        except Exception as e:
            self.get_logger().error(f'Failed to parse command: {e}')
    
    def move_to_position(self, x, y, z, angle, object_name):
        """MoveIt을 사용하여 로봇을 목표 위치로 이동"""
        self.get_logger().info(
            f'Moving to pick {object_name} at position: x={x}, y={y}, z={z}, angle={angle}'
        )
        
        # Action 서버가 준비될 때까지 대기
        self._action_client.wait_for_server()
        
        # Goal 메시지 생성
        goal_msg = MoveGroup.Goal()
        
        # MotionPlanRequest 설정
        motion_plan_request = MotionPlanRequest()
        motion_plan_request.group_name = 'arm'  # Doosan E0509의 planning group 이름
        motion_plan_request.num_planning_attempts = 10
        motion_plan_request.allowed_planning_time = 5.0
        motion_plan_request.max_velocity_scaling_factor = 0.5
        motion_plan_request.max_acceleration_scaling_factor = 0.5
        
        # 목표 Pose 설정
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'base_link'  # 또는 'world'
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        
        # 위치 설정 (cm를 m로 변환)
        pose_stamped.pose.position.x = x / 100.0
        pose_stamped.pose.position.y = y / 100.0
        pose_stamped.pose.position.z = z / 100.0
        
        # 각도를 쿼터니언으로 변환 (Z축 회전)
        angle_rad = math.radians(angle)
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = math.sin(angle_rad / 2.0)
        pose_stamped.pose.orientation.w = math.cos(angle_rad / 2.0)
        
        # Constraints 설정
        constraints = Constraints()
        
        # Position Constraint
        position_constraint = PositionConstraint()
        position_constraint.header = pose_stamped.header
        position_constraint.link_name = 'tool0'  # End effector link 이름
        position_constraint.target_point_offset.x = 0.0
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0
        
        # Constraint region 설정
        solid_primitive = SolidPrimitive()
        solid_primitive.type = SolidPrimitive.SPHERE
        solid_primitive.dimensions = [0.01]  # 1cm tolerance
        position_constraint.constraint_region.primitives.append(solid_primitive)
        position_constraint.constraint_region.primitive_poses.append(pose_stamped.pose)
        position_constraint.weight = 1.0
        
        # Orientation Constraint
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = pose_stamped.header
        orientation_constraint.link_name = 'tool0'
        orientation_constraint.orientation = pose_stamped.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        orientation_constraint.weight = 1.0
        
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)
        
        motion_plan_request.goal_constraints.append(constraints)
        
        # Goal에 설정 적용
        goal_msg.request = motion_plan_request
        goal_msg.planning_options.plan_only = False  # 계획하고 실행
        
        # Action 전송
        self.get_logger().info('Sending goal to MoveIt...')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Goal 응답 콜백"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """결과 콜백"""
        result = future.result().result
        if result.error_code.val == 1:  # SUCCESS
            self.get_logger().info('Motion completed successfully!')
        else:
            self.get_logger().error(f'Motion failed with error code: {result.error_code.val}')
    
    def feedback_callback(self, feedback_msg):
        """피드백 콜백"""
        self.get_logger().info(f'Feedback: {feedback_msg.feedback.state}')


def main(args=None):
    rclpy.init(args=args)
    
    robot_controller = DoosanRobotController()
    
    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        pass
    finally:
        robot_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
