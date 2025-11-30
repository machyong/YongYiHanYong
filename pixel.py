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
import torch
import warnings

# PyTorch FutureWarning 숨기기
warnings.filterwarnings('ignore', category=FutureWarning)

class PixelToWorld(Node):
    def __init__(self):
        super().__init__('pixel_to_world')

        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('pixel_u', 685)
        self.declare_parameter('pixel_v', 95) #95
        self.declare_parameter('target_frame', 'world')
        self.declare_parameter('model_name', 'yolov5s')
        self.declare_parameter('model_type', 'yolov5')
        self.declare_parameter('use_yolo', True)
        self.declare_parameter('confidence_threshold', 0.5)

        self.depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.pixel_u = int(self.get_parameter('pixel_u').get_parameter_value().integer_value)
        self.pixel_v = int(self.get_parameter('pixel_v').get_parameter_value().integer_value)
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.use_yolo = self.get_parameter('use_yolo').get_parameter_value().bool_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        model_name = self.get_parameter('model_name').get_parameter_value().string_value
        model_type = self.get_parameter('model_type').get_parameter_value().string_value

        self.get_logger().info(f'PixelToWorld 시작: pixel=({self.pixel_u},{self.pixel_v}), target_frame={self.target_frame}')

        self.bridge = CvBridge()
        self.camera_info = None
        self.depth_msg = None
        self.image_msg = None
        
        # YOLO 모델 로드
        self.model = None
        self.model_type = model_type
        if self.use_yolo:
            try:
                self.get_logger().info(f"Loading YOLO model: {model_name} (Type: {model_type})")
                if model_type == 'yolov8':
                    from ultralytics import YOLO
                    self.model = YOLO(f'{model_name}.pt')
                else:
                    self.model = torch.hub.load('ultralytics/yolov5', model_name, pretrained=True)
                self.get_logger().info(f"✅ YOLO Model Loaded: {model_name}")
            except Exception as e:
                self.get_logger().error(f"Failed to load YOLO model: {e}")
                self.use_yolo = False

        # tf buffer/listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # 카메라 위치: 월드 원점에서 (52, 0, 71) 떨어짐
        # self.camera_position = np.array([0, 0, 0.71])
        self.camera_position = np.array([0.52, 0.20, 0.89])
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

        # depth 이미지 -> numpy (z값만 사용)
        try:
            depth_img = self.bridge.imgmsg_to_cv2(self.depth_msg, desired_encoding='passthrough')
            rgb_img = self.bridge.imgmsg_to_cv2(self.image_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'depth CvBridge 변환 실패: {e}')
            return

        h, w = rgb_img.shape[:2]
        
        # YOLO 감지 또는 고정된 픽셀 사용
        detections = []
        
        if self.use_yolo and self.model is not None:
            try:
                results = self.model(rgb_img)
                
                # YOLOv5와 YOLOv8 결과 형식이 다름
                if self.model_type == 'yolov8':
                    for result in results:
                        boxes = result.boxes
                        for box in boxes:
                            conf = float(box.conf[0])
                            if conf >= self.confidence_threshold:
                                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                                class_id = int(box.cls[0])
                                class_name = result.names[class_id]
                                
                                # 바운딩 박스의 중심 픽셀
                                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                                detections.append({
                                    'class': class_name,
                                    'confidence': conf,
                                    'bbox': (x1, y1, x2, y2),
                                    'center': (cx, cy)
                                })
                else:
                    # YOLOv5
                    for det in results.xyxy[0]:
                        x1, y1, x2, y2, conf, class_id = det
                        conf = float(conf)
                        if conf >= self.confidence_threshold:
                            x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
                            class_name = self.model.names[int(class_id)]
                            
                            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                            detections.append({
                                'class': class_name,
                                'confidence': conf,
                                'bbox': (x1, y1, x2, y2),
                                'center': (cx, cy)
                            })
                            
                self.get_logger().info(f"🔍 YOLO detected {len(detections)} objects")
                
            except Exception as e:
                self.get_logger().error(f'YOLO 감지 실패: {e}')
                detections = []
        else:
            # 고정된 픽셀 사용
            if not (0 <= self.pixel_u < w and 0 <= self.pixel_v < h):
                self.get_logger().error(f'픽셀 좌표가 이미지 범위를 벗어남: ({self.pixel_u},{self.pixel_v}), 이미지 크기=({w},{h})')
                return
            detections = [{
                'class': 'manual',
                'confidence': 1.0,
                'bbox': None,
                'center': (self.pixel_u, self.pixel_v)
            }]
        
        # 각 감지 객체에 대해 월드 좌표 계산
        cv_img = rgb_img.copy()
        
        for det in detections:
            u, v = det['center']
            
            # 범위 체크
            if not (0 <= u < w and 0 <= v < h):
                continue
            
            raw_depth = depth_img[v, u]

            # depth 값 유효성 체크
            if raw_depth == 0 or (isinstance(raw_depth, float) and math.isnan(raw_depth)):
                self.get_logger().warn(f"유효하지 않은 depth for {det['class']}: pixel=({u},{v})")
                continue

            # depth_msg.encoding 정보로 단위 변환
            enc = getattr(self.depth_msg, 'encoding', None)
            if enc is not None and '16' in enc:
                depth_val = float(raw_depth) * 0.001
            else:
                depth_val = float(raw_depth)

            if depth_val <= 0:
                continue

            # 카메라 내부 파라미터
            K = self.camera_info.k
            fx = K[0]
            fy = K[4]
            cx = K[2]
            cy = K[5]

            # 픽셀 -> 카메라 좌표 (m)
            x_cam = (u - cx) * depth_val / fx
            y_cam = (v - cy) * depth_val / fy
            z_cam = depth_val

            cam_coord = np.array([x_cam, y_cam, z_cam])

            # 회전 변환
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
                f"{det['class']} | confidence={det['confidence']:.2f} | "
                f"월드 좌표: x={world_coord[0]:.3f} y={world_coord[1]:.3f} z={world_coord[2]:.3f}"
            )

            # 이미지에 그리기
            color = (0, 255, 0) if det['class'] != 'manual' else (255, 255, 0)
            cv2.circle(cv_img, (u, v), 8, color, -1)
            
            text = f"{det['class']} | ({world_coord[0]:.2f}, {world_coord[1]:.2f}, {world_coord[2]:.2f})"
            text_pos = (u + 10, v - 10)
            cv2.putText(cv_img, text, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # 윈도우에 표시
        cv2.imshow("Pixel to World (with YOLO)", cv_img)
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