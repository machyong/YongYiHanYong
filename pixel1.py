#!/usr/bin/env python3
"""
RealSense D455f 카메라 화면에서 마우스 클릭한 픽셀을 World 좌표로 변환하여 출력
Ubuntu 24.04, Doosan e0509 로봇용
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class ClickToWorld(Node):
    def __init__(self):
        super().__init__('click_to_world')

        # ROS 파라미터 설정
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')

        self.depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value

        self.get_logger().info('ClickToWorld 노드 시작')

        # 데이터 저장용 변수
        self.bridge = CvBridge()
        self.camera_info = None
        self.depth_img = None
        self.rgb_img = None
        self.clicked_point = None  # (u, v)

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

        # Subscribers
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_cb, 10)
        self.create_subscription(Image, self.depth_topic, self.depth_cb, 10)
        self.create_subscription(Image, self.image_topic, self.image_cb, 10)

        # 마우스 콜백 등록
        cv2.namedWindow("Click to World")
        cv2.setMouseCallback("Click to World", self.mouse_callback)

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
            self.compute_world_point(x, y)

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
            text = f"Clicked: ({u}, {v})"
            cv2.putText(display_img, text, (u + 20, v - 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # 사용 안내 표시
        cv2.putText(display_img, "Click to get World coordinates", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(display_img, "Press 'q' to quit", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)

        cv2.imshow("Click to World", display_img)
        
        # 'q' 키로 종료
        key = cv2.waitKey(1)
        if key == ord('q'):
            self.get_logger().info('종료 요청됨')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ClickToWorld()
    
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
