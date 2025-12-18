import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, TransformStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import tf2_ros
from tf2_ros import TransformBroadcaster
import warnings
# Publisher 추가
from geometry_msgs.msg import Point
from std_msgs.msg import String


warnings.filterwarnings('ignore', category=FutureWarning)

from ultralytics import YOLO  # YOLO11 사용

class PixelToWorld(Node):
    def __init__(self):
        super().__init__('pixel_to_world')
        
        # ROI 영역 (x, y, w, h)
        self.declare_parameter('roi_x', 200)
        self.declare_parameter('roi_y', 50)
        self.declare_parameter('roi_w', 800)
        self.declare_parameter('roi_h', 700)

        self.roi_x = int(self.get_parameter('roi_x').value)
        self.roi_y = int(self.get_parameter('roi_y').value)
        self.roi_w = int(self.get_parameter('roi_w').value)
        self.roi_h = int(self.get_parameter('roi_h').value)



        # ------------------------
        # Parameters
        # ------------------------
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        
        
        # Publisher 코드 추가
        self.priority_pub = self.create_publisher(String, "/priority_object", 10)



        # Publisher 추가
        self.target_pub = self.create_publisher(PointStamped, "/target_object", 10)
        self.target_class_pub = self.create_publisher(String, "/target_object_class", 10)

        # pixel fallback
        self.declare_parameter('pixel_u', 685)
        self.declare_parameter('pixel_v', 95)

        self.declare_parameter('target_frame', 'world')

        # YOLO11-OBB model
        self.declare_parameter(
            'model_name',
            '/home/hun/YongYiHanYong/src/yolo_realsense/yolo_realsense/best.pt'
        )
        self.declare_parameter('use_yolo', True)
        self.declare_parameter('confidence_threshold', 0.5)

        # ------------------------
        # Load parameters
        # ------------------------
        self.depth_topic = self.get_parameter('depth_topic').value
        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value

        self.pixel_u = int(self.get_parameter('pixel_u').value)
        self.pixel_v = int(self.get_parameter('pixel_v').value)
        self.target_frame = self.get_parameter('target_frame').value
        self.use_yolo = self.get_parameter('use_yolo').value
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)

        model_name = self.get_parameter('model_name').value

        self.get_logger().info(f"PixelToWorld 시작 | backup pixel=({self.pixel_u}, {self.pixel_v})")

        self.bridge = CvBridge()
        self.camera_info = None
        self.depth_msg = None
        self.image_msg = None

        # ------------------------
        # Load YOLO11 OBB model
        # ------------------------
        self.model = None
        if self.use_yolo:
            try:
                self.get_logger().info(f"YOLO11 OBB 모델 로드 중: {model_name}")
                self.model = YOLO(model_name)
                self.get_logger().info("✅ YOLO11 OBB Model Loaded")
            except Exception as e:
                self.get_logger().error(f"YOLO 로드 실패: {e}")
                self.use_yolo = False

        # ------------------------
        # TF Setup
        # ------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # World 기준 카메라 위치
        self.camera_position = np.array([0.52, 0.20, 0.89])

        self.publish_camera_transform()

        # ------------------------
        # Subscribers
        # ------------------------
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_cb, 10)
        self.create_subscription(Image, self.depth_topic, self.depth_cb, 10)
        self.create_subscription(Image, self.image_topic, self.image_cb, 10)

    # ------------------------------------------------------
    # Publish camera TF
    # ------------------------------------------------------
    def publish_camera_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.target_frame
        transform.child_frame_id = 'camera_depth_frame'

        transform.transform.translation.x = float(self.camera_position[0])
        transform.transform.translation.y = float(self.camera_position[1])
        transform.transform.translation.z = float(self.camera_position[2])

        # 회전은 기존 코드 그대로 (x=180°, z=90°)
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

        R_world_cam = R_z @ R_y @ R_x

        from scipy.spatial.transform import Rotation as R
        quat = R.from_matrix(R_world_cam).as_quat()

        transform.transform.rotation.x = float(quat[0])
        transform.transform.rotation.y = float(quat[1])
        transform.transform.rotation.z = float(quat[2])
        transform.transform.rotation.w = float(quat[3])

        self.tf_broadcaster.sendTransform(transform)

    # ------------------------------------------------------
    # Subscribers callbacks
    # ------------------------------------------------------
    def camera_info_cb(self, msg):
        if self.camera_info is None:
            self.get_logger().info("CameraInfo 수신됨")
        self.camera_info = msg

    def image_cb(self, msg):
        self.image_msg = msg
        self.try_compute_world_point()

    def depth_cb(self, msg):
        self.depth_msg = msg
        self.try_compute_world_point()

    # ------------------------------------------------------
    # Core Logic
    # ------------------------------------------------------
    def try_compute_world_point(self):
        if self.depth_msg is None or self.camera_info is None or self.image_msg is None:
            return

        try:
            depth_img = self.bridge.imgmsg_to_cv2(self.depth_msg, desired_encoding='passthrough')
            rgb_img = self.bridge.imgmsg_to_cv2(self.image_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge 변환 실패: {e}")
            return

        h, w = rgb_img.shape[:2]
        detections = []

        # ------------------------------------------------------
        # YOLO11 OBB detection (각도 + 회전 박스 포함)
        # ------------------------------------------------------
        if self.use_yolo and self.model is not None:
            try:
                # 모델에 이미지 전달
                results = self.model(rgb_img)

                for result in results:
                    boxes = result.obb
                    if boxes is None:
                        continue

                    # 안전하게 텐서/넘파이/리스트 처리
                    try:
                        xyxy = boxes.xyxy.cpu().numpy()
                    except Exception:
                        xyxy = np.array(boxes.xyxy)

                    # corners: may be named xyxyxyxy or xyxyxyxyn — try both
                    corners_flat = None
                    if hasattr(boxes, 'xyxyxyxy'):
                        try:
                            corners_flat = boxes.xyxyxyxy.cpu().numpy()
                        except Exception:
                            corners_flat = np.array(boxes.xyxyxyxy)
                    elif hasattr(boxes, 'xyxyxyxyn'):
                        try:
                            corners_flat = boxes.xyxyxyxyn.cpu().numpy()
                        except Exception:
                            corners_flat = np.array(boxes.xyxyxyxyn)
                    else:
                        # fallback: no rotated corners provided
                        corners_flat = None

                    # xywhr contains [xc, yc, w, h, rotation]
                    try:
                        xywhr = boxes.xywhr.cpu().numpy()
                    except Exception:
                        xywhr = np.array(boxes.xywhr)

                    try:
                        confs = boxes.conf.cpu().numpy()
                    except Exception:
                        confs = np.array(boxes.conf)

                    try:
                        classes = boxes.cls.cpu().numpy()
                    except Exception:
                        classes = np.array(boxes.cls)

                    num = len(classes)
                    for i in range(num):
                        conf = float(confs[i])
                        if conf < self.confidence_threshold:
                            continue

                        class_id = int(classes[i])
                        class_name = result.names[class_id] if hasattr(result, "names") else str(class_id)

                        # axis-aligned box (xyxy)
                        x1, y1, x2, y2 = map(int, xyxy[i].tolist())

                        # center pixel (used for depth)
                        cx = int((x1 + x2) / 2)
                        cy = int((y1 + y2) / 2)

                        # rotation (radian) from xywhr's 5th element
                        angle_rad = None
                        angle_deg = None
                        if xywhr is not None and xywhr.shape[0] > i and xywhr.shape[1] >= 5:
                            angle_rad = float(xywhr[i][4])
                            angle_deg = angle_rad * 180.0 / np.pi

                        # rotated corners -> reshape to (4,2) if available
                        obb_corners = None
                        if corners_flat is not None and corners_flat.shape[0] > i:
                            # corners_flat[i] could be normalized or absolute; ensure shape (8,)
                            arr = corners_flat[i].astype(np.float32)
                            try:
                                pts = arr.reshape(4, 2)
                                obb_corners = pts.tolist()
                            except Exception:
                                obb_corners = None

                        detections.append({
                            'class': class_name,
                            'confidence': conf,
                            'bbox': (x1, y1, x2, y2),
                            'obb_corners': obb_corners,
                            'angle_rad': angle_rad,
                            'angle_deg': angle_deg,
                            'center': (cx, cy)
                        })

                self.get_logger().info(f"YOLO11 OBB 감지 객체 수: {len(detections)}")

            except Exception as e:
                self.get_logger().error(f"YOLO OBB 감지 실패: {e}")
                detections = []

        else:
            # 수동 픽셀 fallback
            detections = [{
                'class': 'manual',
                'confidence': 1.0,
                'bbox': None,
                'center': (self.pixel_u, self.pixel_v)
            }]

        # ------------------------------------------------------
        # 월드 좌표 계산 및 시각화
        # ------------------------------------------------------
        cv_img = rgb_img.copy()



        # ------------------------------------------------------
        # 🔥 우선순위 기반 대상 선택
        # ------------------------------------------------------
        priority_list = ["cutlery", "dish", "cup"]  # 원하는 우선순위
        selected_det = None

        for cls_name in priority_list:
            for det in detections:
                if det['class'] == cls_name:
                    selected_det = det
                    break
            if selected_det:
                break

        # 아무 우선순위 객체도 없으면 publish 없음
        if selected_det is None:
            return

        # 이제 forward 과정은 선택된 det만 사용
        det = selected_det
        detections = [det]  # 이후 기존 코드 구조 유지 위해 하나만 남김



        for det in detections:
            u, v = det['center']

            # --------------------------------------------------
            # 회전 바운딩박스(OBB) 시각화 (먼저 중심 구한 후 사용)
            # --------------------------------------------------
            if det.get('obb_corners') is not None:
                try:
                    pts = np.array(det['obb_corners'], np.int32).reshape((-1, 1, 2))
                    cv2.polylines(cv_img, [pts], True, (0, 0, 255), 2)
                    # 각도 표시 (있으면)
                    if det.get('angle_deg') is not None:
                        angle_txt = f"{det['angle_deg']:.1f} deg"
                        cv2.putText(cv_img, angle_txt, (u + 15, v + 20),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                except Exception:
                    # 안전하게 통과
                    pass

            if not (0 <= u < w and 0 <= v < h):
                continue

            raw_depth = depth_img[v, u]

            if raw_depth == 0:
                self.get_logger().warn(f"Depth 0 → 건너뜀: ({u},{v})")
                continue

            # depth 단위 변환
            enc = getattr(self.depth_msg, 'encoding', None)
            if enc and '16' in enc:
                depth_val = float(raw_depth) * 0.001
            else:
                depth_val = float(raw_depth)

            if depth_val <= 0:
                continue

            # 카메라 내부 파라미터
            K = self.camera_info.k
            fx, fy, cx_k, cy_k = K[0], K[4], K[2], K[5]

            # Pixel → Camera
            x_cam = (u - cx_k) * depth_val / fx
            y_cam = (v - cy_k) * depth_val / fy
            z_cam = depth_val
            cam_coord = np.array([x_cam, y_cam, z_cam])

            # 회전 변환 (원래 코드의 회전 사용)
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

            R_total = R_z @ R_y @ R_x
            world_coord = R_total @ cam_coord + self.camera_position



            # --------------------------------------------------
            # 🔥 선택된 객체만 퍼블리시
            # --------------------------------------------------
            pt_msg = PointStamped()
            pt_msg.header.frame_id = self.target_frame
            pt_msg.header.stamp = self.get_clock().now().to_msg()
            pt_msg.point.x = float(world_coord[0])
            pt_msg.point.y = float(world_coord[1])
            pt_msg.point.z = float(world_coord[2])

            self.target_pub.publish(pt_msg)

            class_msg = String()
            class_msg.data = det['class']
            self.target_class_pub.publish(class_msg)
            
        
            # 로그 출력
            self.get_logger().info(
                f"{det['class']} | conf={det['confidence']:.2f} | "
                f"world: x={world_coord[0]:.3f}, y={world_coord[1]:.3f}, z={world_coord[2]:.3f} | "
                f"angle_deg={det.get('angle_deg')}"
            )

            
            # 🔥 priority_object 퍼블리시 추가
            msg = String()
            msg.data = f"{det['class']},{world_coord[0]:.3f},{world_coord[1]:.3f},{world_coord[2]:.3f}"
            self.priority_pub.publish(msg)


            # 이미지 표시: 중심/텍스트/AABB
            color = (0, 255, 0) if det['class'] != 'manual' else (255, 255, 0)
            cv2.circle(cv_img, (u, v), 6, color, -1)


            """""
            초록색 바운딩 박스 불필요해서 주석처리

            bbox = det.get('bbox')
            if bbox is not None:
                x1, y1, x2, y2 = bbox
                cv2.rectangle(cv_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            """""


            text = f"{det['class']} ({world_coord[0]:.2f},{world_coord[1]:.2f},{world_coord[2]:.2f})"
            cv2.putText(cv_img, text, (u + 10, v - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        cv2.imshow("Pixel to World - YOLO11 OBB", cv_img)
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
