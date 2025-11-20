import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, TransformStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import tf2_ros  # 누락 시 추가
from tf2_ros import TransformBroadcaster

class PixelToWorld(Node):
    def __init__(self):
        super().__init__('pixel_to_world')

        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('pixel_u', 685)
        self.declare_parameter('pixel_v', 95) #95
        self.declare_parameter('target_frame', 'world')

        self.depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.pixel_u = int(self.get_parameter('pixel_u').get_parameter_value().integer_value)
        self.pixel_v = int(self.get_parameter('pixel_v').get_parameter_value().integer_value)
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value

        self.get_logger().info(f'PixelToWorld 시작: pixel=({self.pixel_u},{self.pixel_v}), target_frame={self.target_frame}')

        self.bridge = CvBridge()
        self.camera_info = None
        self.depth_msg = None
        self.image_msg = None

        # tf buffer/listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # 카메라 위치: 월드 원점에서 (52, 0, 71) 떨어짐
        # self.camera_position = np.array([0, 0, 0.71])
        self.camera_position = np.array([0.662, 0.00, 0.662])
        self.publish_camera_transform()

        # subscribers
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_cb, 10)
        self.create_subscription(Image, self.depth_topic, self.depth_cb, 10)
        self.create_subscription(Image, self.image_topic, self.image_cb, 10)

    def publish_camera_transform(self):
        """카메라의 고정된 transform을 발행"""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.target_frame

        # depth_msg가 아직 없을 수 있으니, 확실한 frame_id를 쓰거나 파라미터로 빼는 것도 좋음
        transform.child_frame_id = (
            self.depth_msg.header.frame_id if hasattr(self, 'depth_msg') and self.depth_msg
            else 'camera_depth_frame'
        )

        # 카메라 위치 (월드 기준)
        transform.transform.translation.x = float(self.camera_position[0])
        transform.transform.translation.y = float(self.camera_position[1])
        transform.transform.translation.z = float(self.camera_position[2])

        # ✅ try_compute_world_point와 동일한 회전 사용
        theta_x = np.pi          # 180 deg
        theta_y = 0.0
        theta_z = np.pi / 2.0    # 90 deg

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

        R_world_cam = R_z @ R_y @ R_x

        from scipy.spatial.transform import Rotation as R
        quat = R.from_matrix(R_world_cam).as_quat()  # [x, y, z, w]

        transform.transform.rotation.x = float(quat[0])
        transform.transform.rotation.y = float(quat[1])
        transform.transform.rotation.z = float(quat[2])
        transform.transform.rotation.w = float(quat[3])

        self.tf_broadcaster.sendTransform(transform)

    def camera_info_cb(self, msg: CameraInfo):
        if self.camera_info is None:
            self.get_logger().info('camera_info 수신됨')
        self.camera_info = msg

    def image_cb(self, msg: Image):
        """원본 이미지 저장"""
        self.image_msg = msg
        self.try_compute_world_point()

    def depth_cb(self, msg: Image):
        """depth 이미지 저장"""
        self.depth_msg = msg
        self.try_compute_world_point()

    def try_compute_world_point(self):
        """원본 이미지와 depth 이미지를 사용하여 월드 좌표 계산"""
        # 필요 데이터 체크
        if self.depth_msg is None or self.camera_info is None or self.image_msg is None:
            return

        u = self.pixel_u
        v = self.pixel_v

        # depth 이미지 -> numpy (z값만 사용)
        try:
            depth_img = self.bridge.imgmsg_to_cv2(self.depth_msg, desired_encoding='passthrough')
            rgb_img = self.bridge.imgmsg_to_cv2(self.image_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'depth CvBridge 변환 실패: {e}')
            return

        # 범위 체크
        h, w = rgb_img.shape[:2]
        if not (0 <= u < w and 0 <= v < h):
            self.get_logger().error(f'픽셀 좌표가 이미지 범위를 벗어남: ({u},{v}), 이미지 크기=({w},{h})')
            return

        raw_depth = depth_img[v, u]

        # depth 값 유효성 체크
        if raw_depth == 0 or (isinstance(raw_depth, float) and math.isnan(raw_depth)):
            self.get_logger().warn('해당 픽셀의 depth 값이 0 또는 NaN입니다.')
            return

        # depth_msg.encoding 정보로 단위 변환
        enc = getattr(self.depth_msg, 'encoding', None)
        if enc is not None and '16' in enc:
            # 16-bit unsigned -> mm to m
            depth_val = float(raw_depth) * 0.001
        else:
            # 32-bit float
            depth_val = float(raw_depth)

        if depth_val <= 0:
            self.get_logger().warn(f'유효하지 않은 depth: {depth_val}')
            return

        # 카메라 내부 파라미터
        K = self.camera_info.k  # 3x3 row-major
        fx = K[0]
        fy = K[4]
        cx = K[2]
        cy = K[5]

        # 픽셀 -> 카메라 좌표 (m)
        x_cam = (u - cx) * depth_val / fx
        y_cam = (v - cy) * depth_val / fy
        z_cam = depth_val

        self.get_logger().info(f'카메라 프레임 좌표: x={x_cam:.4f} y={y_cam:.4f} z={z_cam:.4f} (m)')

        cam_coord = np.array([x_cam, y_cam, z_cam])

        theta_x = np.pi
        theta_y = 0.0
        theta_z = np.pi / 2.0

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

        rotation_matrix = R_z @ R_y @ R_x

        world_coord = rotation_matrix @ cam_coord + self.camera_position

        self.get_logger().info(
            f'월드 좌표 ({self.target_frame}): x={world_coord[0]:.4f} y={world_coord[1]:.4f} z={world_coord[2]:.4f}'
        )

        # ----------- 이미지 위에 점과 텍스트 표시 -----------
        try:
            cv_img = self.bridge.imgmsg_to_cv2(self.image_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'이미지 CvBridge 변환 실패: {e}')
            return

        # 점 그리기
        cv2.circle(cv_img, (u, v), 6, (0, 0, 255), -1)
        # 텍스트 준비
        text = f"({world_coord[0]:.2f}, {world_coord[1]:.2f}, {world_coord[2]:.2f})"
        # 텍스트 위치 (점 오른쪽 위)
        text_pos = (u + 10, v - 10)
        cv2.putText(cv_img, text, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # 윈도우에 표시
        cv2.imshow("Pixel to World", cv_img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = PixelToWorld()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()