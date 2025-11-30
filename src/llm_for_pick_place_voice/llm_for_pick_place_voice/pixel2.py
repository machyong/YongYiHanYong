#!/usr/bin/env python3
"""
RealSense D455f 카메라 화면에서 마우스 클릭 시 두산 e0509 로봇이 해당 좌표로 이동
Ubuntu 24.04, Doosan e0509 로봇용
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import sys
import threading

# 두산 로봇 제어 라이브러리
try:
    import DR_init
    DRCF = DR_init.__dsr__
except ImportError:
    print("⚠️  두산 로봇 라이브러리를 찾을 수 없습니다. 데모 모드로 실행합니다.")
    DRCF = None


class ClickToRobot(Node):
    def __init__(self):
        super().__init__('click_to_robot')

        # ROS 파라미터 설정
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('robot_id', 'dsr01')
        self.declare_parameter('robot_model', 'e0509')
        self.declare_parameter('demo_mode', DRCF is None)  # 라이브러리 없으면 자동 데모 모드

        self.depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        self.robot_model = self.get_parameter('robot_model').get_parameter_value().string_value
        self.demo_mode = self.get_parameter('demo_mode').get_parameter_value().bool_value

        self.get_logger().info(f'ClickToRobot 노드 시작 (Demo Mode: {self.demo_mode})')

        # 데이터 저장용 변수
        self.bridge = CvBridge()
        self.camera_info = None
        self.depth_img = None
        self.rgb_img = None
        self.clicked_point = None  # (u, v)
        self.target_world_coord = None  # 목표 world 좌표

        # 카메라 위치 (world 프레임 기준, 미터 단위)
        self.camera_position = np.array([0.52, 0.20, 0.89])

        # 회전 행렬 (카메라 -> world)
        theta_x = np.pi          # 180도
        theta_y = 0.0
        theta_z = np.pi / 2.0    # 90도

        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(theta_x), -np.sin(theta_x)],
            [0, np.sin(theta_x),  np.cos(theta_x)]
        ])

        R_y = np.array([
            [np.cos(theta_y), 0, np.sin(theta_y)],
            [0, 1, 0],
            [-np.sin(theta_y), 0, np.cos(theta_y)]
        ])

        R_z = np.array([
            [np.cos(theta_z), -np.sin(theta_z), 0],
            [np.sin(theta_z),  np.cos(theta_z), 0],
            [0, 0, 1]
        ])

        self.rotation_matrix = R_z @ R_y @ R_x

        # 두산 로봇 초기화
        self.robot = None
        if not self.demo_mode:
            self.init_doosan_robot()
        else:
            self.get_logger().info('🤖 데모 모드: 로봇 명령은 출력만 됩니다.')

        # Subscribers
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_cb, 10)
        self.create_subscription(Image, self.depth_topic, self.depth_cb, 10)
        self.create_subscription(Image, self.image_topic, self.image_cb, 10)

        # 마우스 콜백 등록
        cv2.namedWindow("Click to Move Robot")
        cv2.setMouseCallback("Click to Move Robot", self.mouse_callback)

    def init_doosan_robot(self):
        """두산 로봇 초기화"""
        try:
            self.get_logger().info(f'두산 로봇 연결 중: {self.robot_id}, {self.robot_model}')
            self.robot = DRCF.DRFLEx(self.robot_id, self.robot_model)
            
            # 로봇 초기화
            self.robot.SetRobotMode(DRCF.ROBOT_MODE_MANUAL)
            self.robot.SetRobotSystem(DRCF.ROBOT_SYSTEM_REAL)
            
            self.get_logger().info('✅ 로봇 연결 성공')
        except Exception as e:
            self.get_logger().error(f'❌ 로봇 연결 실패: {e}')
            self.demo_mode = True

    def camera_info_cb(self, msg: CameraInfo):
        """카메라 내부 파라미터 수신"""
        if self.camera_info is None:
            self.get_logger().info('카메라 정보 수신됨')
        self.camera_info = msg

    def image_cb(self, msg: Image):
        """RGB 이미지 수신"""
        try:
            self.rgb_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.display_image()
        except Exception as e:
            self.get_logger().error(f'RGB 이미지 변환 실패: {e}')

    def depth_cb(self, msg: Image):
        """Depth 이미지 수신"""
        try:
            self.depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Depth 이미지 변환 실패: {e}')

    def mouse_callback(self, event, x, y, flags, param):
        """마우스 클릭 이벤트 처리"""
        if event == cv2.EVENT_LBUTTONDOWN:
            self.clicked_point = (x, y)
            self.get_logger().info(f'클릭한 픽셀: ({x}, {y})')
            world_coord = self.compute_world_point(x, y)
            
            if world_coord is not None:
                self.target_world_coord = world_coord
                # 별도 스레드에서 로봇 이동 (ROS spin 블로킹 방지)
                threading.Thread(target=self.move_robot_to_target, args=(world_coord,), daemon=True).start()

    def compute_world_point(self, u, v):
        """픽셀 좌표를 world 좌표로 변환"""
        if self.depth_img is None or self.camera_info is None:
            self.get_logger().warn('카메라 데이터가 준비되지 않음')
            return None

        h, w = self.depth_img.shape[:2]
        
        # 범위 체크
        if not (0 <= u < w and 0 <= v < h):
            self.get_logger().error(f'픽셀이 범위를 벗어남: ({u}, {v}), 이미지 크기: ({w}, {h})')
            return None

        # Depth 값 가져오기
        raw_depth = self.depth_img[v, u]

        # Depth 유효성 체크
        if raw_depth == 0 or (isinstance(raw_depth, float) and math.isnan(raw_depth)):
            self.get_logger().warn(f'유효하지 않은 depth: pixel=({u}, {v})')
            return None

        # Depth 단위 변환 (16bit -> 미터)
        depth_val = float(raw_depth) * 0.001

        if depth_val <= 0:
            self.get_logger().warn(f'Depth 값이 0 이하: {depth_val}')
            return None

        # 카메라 내부 파라미터
        K = self.camera_info.k
        fx = K[0]
        fy = K[4]
        cx = K[2]
        cy = K[5]

        # 픽셀 -> 카메라 좌표계 (미터)
        x_cam = (u - cx) * depth_val / fx
        y_cam = (v - cy) * depth_val / fy
        z_cam = depth_val

        cam_coord = np.array([x_cam, y_cam, z_cam])

        # 카메라 좌표 -> world 좌표
        world_coord = self.rotation_matrix @ cam_coord + self.camera_position

        self.get_logger().info(
            f'🎯 World 좌표: X={world_coord[0]:.4f}m, Y={world_coord[1]:.4f}m, Z={world_coord[2]:.4f}m'
        )

        return world_coord

    def move_robot_to_target(self, world_coord):
        """로봇을 목표 좌표로 이동"""
        # world 좌표를 밀리미터로 변환 (두산 로봇은 mm 단위 사용)
        x_mm = world_coord[0] * 1000
        y_mm = world_coord[1] * 1000
        z_mm = world_coord[2] * 1000

        # 기본 자세 (roll, pitch, yaw - degrees)
        rx, ry, rz = 0, 180, 0

        target_pos = [x_mm, y_mm, z_mm, rx, ry, rz]

        if self.demo_mode:
            self.get_logger().info(f'🤖 [DEMO] 로봇 이동 명령: {target_pos}')
        else:
            try:
                self.get_logger().info(f'🤖 로봇 이동 시작: X={x_mm:.1f}, Y={y_mm:.1f}, Z={z_mm:.1f}')
                
                # 두산 로봇 MoveL 명령 (직선 이동)
                self.robot.MoveL(target_pos, vel=100, acc=100)
                
                self.get_logger().info('✅ 로봇 이동 완료')
            except Exception as e:
                self.get_logger().error(f'❌ 로봇 이동 실패: {e}')

    def display_image(self):
        """이미지 화면에 표시"""
        if self.rgb_img is None:
            return

        display_img = self.rgb_img.copy()

        # 클릭한 지점 표시
        if self.clicked_point is not None:
            u, v = self.clicked_point
            cv2.circle(display_img, (u, v), 10, (0, 255, 0), -1)
            cv2.circle(display_img, (u, v), 15, (255, 255, 0), 2)
            
            # 좌표 텍스트 표시
            if self.target_world_coord is not None:
                text = f"Target: ({self.target_world_coord[0]:.3f}, {self.target_world_coord[1]:.3f}, {self.target_world_coord[2]:.3f})m"
                cv2.putText(display_img, text, (u + 20, v - 20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        # 사용 안내 표시
        mode_text = "[DEMO MODE]" if self.demo_mode else "[ROBOT CONNECTED]"
        cv2.putText(display_img, f"Click to Move Robot {mode_text}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(display_img, "Press 'q' to quit", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)

        cv2.imshow("Click to Move Robot", display_img)
        
        # 'q' 키로 종료
        key = cv2.waitKey(1)
        if key == ord('q'):
            self.get_logger().info('종료 요청됨')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ClickToRobot()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
