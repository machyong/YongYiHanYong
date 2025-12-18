#!/usr/bin/env python3
# 전체 통합 노드: ROI에서 YOLO(ultralytics) 검출 → depth로 월드좌표 계산 → 여러 토픽 퍼블리시
# 한국어 주석이 상세히 들어있음 (요청사항 반영)

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, TransformStamped
from std_msgs.msg import String
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import tf2_ros
from tf2_ros import TransformBroadcaster
import warnings
import json
from ultralytics import YOLO
from interface_pkg.srv import GetTargetPoint
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger

warnings.filterwarnings('ignore', category=FutureWarning)


class PixelToWorld(Node):
    """ROI 기반 YOLO 검출 + depth->world 좌표 변환 통합 노드
    - ROI는 런타임에 파라미터로 조절 가능 (set_parameters로 변경하면 즉시 반영)
    - 신뢰도 threshold 기본 0.7 (0.7 미만은 무시)
    - OBB(xywhr) 가능 모델을 우선 사용, 없으면 axis-aligned boxes 사용
    - 우선순위 클래스 목록에 따라 대상 선택 및 퍼블리시
    """

    def __init__(self):
        super().__init__('pixel_to_world')

        # ------------------------
        # 파라미터 (런치 또는 런타임에서 변경 가능)
        # ------------------------
        # ROI (이미지 좌상단 기준)
        self.declare_parameter('roi_x', 700)
        self.declare_parameter('roi_y', 20)
        self.declare_parameter('roi_w', 450)
        self.declare_parameter('roi_h', 570)

        # 토픽 설정
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')

        # fallback 픽셀 (ROI 내 검출 없을 경우 사용)
        self.declare_parameter('pixel_u', 685)
        self.declare_parameter('pixel_v', 95)

        # YOLO 관련
        # model_name 은 .pt 파일의 전체 경로를 권장 (예: /home/min/proj_ws/.../best.pt)
        self.declare_parameter('model_path', '/home/up/YongYiHanYong/src/yolo_realsense/yolo_realsense/best.pt')
        self.declare_parameter('use_yolo', True)
        # 기본 confidence threshold 0.7 (사용자 요청: 70% 이하 필터링)
        self.declare_parameter('confidence_threshold', 0.7)

        # world/frame 파라미터
        self.declare_parameter('target_frame', 'world')
        # camera position in world (m)
        self.declare_parameter('camera_position', [0.52, 0.20, 0.89])

        # 우선순위 리스트 (필요시 파라미터로 바꿀 수 있음)
        # 기본: cutlery > dish > cup
        self.priority_list = ["cutlery", "dish", "cup"]

        # ------------------------
        # 파라미터 로드 (초기값)
        # ------------------------
        self.roi_x = int(self.get_parameter('roi_x').value)
        self.roi_y = int(self.get_parameter('roi_y').value)
        self.roi_w = int(self.get_parameter('roi_w').value)
        self.roi_h = int(self.get_parameter('roi_h').value)

        self.depth_topic = self.get_parameter('depth_topic').value
        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value

        self.pixel_u = int(self.get_parameter('pixel_u').value)
        self.pixel_v = int(self.get_parameter('pixel_v').value)

        self.model_path = self.get_parameter('model_path').value
        self.use_yolo = bool(self.get_parameter('use_yolo').value)
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)

        self.target_frame = self.get_parameter('target_frame').value
        cam_pos_list = self.get_parameter('camera_position').value
        self.camera_position = np.array(cam_pos_list, dtype=float)

        self.get_logger().info(f"Node 시작 | ROI=({self.roi_x},{self.roi_y},{self.roi_w},{self.roi_h}), "
                               f"model={self.model_path}, conf_thr={self.confidence_threshold:.2f}")

        # ------------------------
        # 상태 변수
        # ------------------------
        self.bridge = CvBridge()
        self.camera_info = None
        self.depth_msg = None
        self.image_msg = None
        self.latest_angle_info = None  # 추가: 최신 angle_info 저장 변수

        # ------------------------
        # 퍼블리셔들 (요청사항에 맞춘 토픽들)
        # ------------------------
        # 1) 잘라낸 ROI 이미지를 발행 (다른 노드/디버깅용)
        self.roi_image_pub = self.create_publisher(Image, '/roi/image', 5)

        # 2) ROI에서 얻은 모든 검출 결과(JSON 문자열)
        self.yolo_detections_pub = self.create_publisher(String, '/yolo/detections', 5)

        # 3) 우선순위로 선택된 객체의 정보 (CSV or JSON) - 기존 요구 유지
        self.priority_pub = self.create_publisher(String, '/priority_object', 10)

        # 4) 선택된 객체의 월드 포인트 (PointStamped)
        self.target_pub = self.create_publisher(PointStamped, '/target_object', 10)

        # 5) 선택된 객체 클래스
        self.target_class_pub = self.create_publisher(String, '/target_object_class', 10)

        # 6) 각도 정보 (Float32로 소수점 2자리까지)
        self.angle_info_pub = self.create_publisher(Float32, '/target_angle_info', 10)

        # ------------------------
        # YOLO 모델 로드 (ultralytics YOLO)
        # ------------------------
        self.model = None
        if self.use_yolo:
            try:
                self.get_logger().info(f"YOLO 모델 로드 시도: {self.model_path}")
                # ultralytics YOLO (v11/v8) 지원: .pt 파일 전체 경로 사용
                self.model = YOLO(self.model_path)
                self.get_logger().info("✅ YOLO 모델 로드 성공")
            except Exception as e:
                self.get_logger().error(f"YOLO 모델 로드 실패: {e}")
                self.use_yolo = False

        # ------------------------
        # TF 브로드캐스터 (카메라 transform 고정 발행)
        # ------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.publish_camera_transform()

        # ------------------------
        # subscribers: camera info, depth, color
        # ------------------------
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_cb, 10)
        self.create_subscription(Image, self.depth_topic, self.depth_cb, 10)
        self.create_subscription(Image, self.image_topic, self.image_cb, 10)

        # ------------------------
        # 파라미터 런타임 업데이트(ROI 등) 콜백 등록
        # ------------------------
        self.add_on_set_parameters_callback(self.on_parameter_update)

        # ------------------------
        # 서비스: get_target_point (비어있는 요청 -> geometry_msgs/Point 응답)
        # ------------------------
        self.srv = self.create_service(Trigger, '/get_target_point', self.handle_get_target_point)
        self.get_logger().info('GetTargetPoint service is ready to receive requests.')

    # ------------------------
    # 파라미터 업데이트 콜백 (실시간 ROI 조절 등)
    # ------------------------
    def on_parameter_update(self, params):
        """사용자가 rclpy param set으로 변경하면 여기서 반영"""
        for p in params:
            if p.name == 'roi_x':
                self.roi_x = int(p.value)
            elif p.name == 'roi_y':
                self.roi_y = int(p.value)
            elif p.name == 'roi_w':
                self.roi_w = int(p.value)
            elif p.name == 'roi_h':
                self.roi_h = int(p.value)
            elif p.name == 'confidence_threshold':
                self.confidence_threshold = float(p.value)
            elif p.name == 'model_path':
                # 모델 교체 시 동적으로 로드 시도 (주의: 느릴 수 있음)
                self.model_path = str(p.value)
                try:
                    self.model = YOLO(self.model_path)
                    self.get_logger().info(f"모델 교체 성공: {self.model_path}")
                except Exception as e:
                    self.get_logger().error(f"모델 교체 실패: {e}")
                    self.model = None
                    self.use_yolo = False
            elif p.name == 'camera_position':
                self.camera_position = np.array(p.value, dtype=float)
        # 반환값: 변경 허용
        return rclpy.parameter.ParameterEvent()

    # ------------------------
    # 고정 카메라 transform 발행 (world -> camera_depth_frame)
    # ------------------------
    def publish_camera_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.target_frame
        transform.child_frame_id = 'camera_depth_frame'

        transform.transform.translation.x = float(self.camera_position[0])
        transform.transform.translation.y = float(self.camera_position[1])
        transform.transform.translation.z = float(self.camera_position[2])

        # 원래 코드와 동일한 회전 (x=180deg, z=90deg)
        theta_x = np.pi
        theta_y = 0.0
        theta_z = np.pi / 2.0

        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(theta_x), -np.sin(theta_x)],
            [0, np.sin(theta_x), np.cos(theta_x)]
        ])
        R_y = np.array([
            [np.cos(theta_y), 0, np.sin(theta_y)],
            [0, 1, 0],
            [-np.sin(theta_y), 0, np.cos(theta_y)]
        ])
        R_z = np.array([
            [np.cos(theta_z), -np.sin(theta_z), 0],
            [np.sin(theta_z), np.cos(theta_z), 0],
            [0, 0, 1]
        ])

        R_world_cam = R_z @ R_y @ R_x

        from scipy.spatial.transform import Rotation as R
        quat = R.from_matrix(R_world_cam).as_quat()  # x,y,z,w

        transform.transform.rotation.x = float(quat[0])
        transform.transform.rotation.y = float(quat[1])
        transform.transform.rotation.z = float(quat[2])
        transform.transform.rotation.w = float(quat[3])

        self.tf_broadcaster.sendTransform(transform)

    # ------------------------
    # Subscriber callbacks: 저장 후 처리 함수 호출
    # ------------------------
    def camera_info_cb(self, msg: CameraInfo):
        if self.camera_info is None:
            self.get_logger().info("CameraInfo 수신됨")
        self.camera_info = msg

    def depth_cb(self, msg: Image):
        self.depth_msg = msg
        self.try_compute_world_point()

    def image_cb(self, msg: Image):
        self.image_msg = msg
        self.try_compute_world_point()

    # ------------------------
    # 핵심 처리: ROI 자르고 YOLO 수행 → 월드좌표 계산 → 퍼블리시
    # ------------------------
    def try_compute_world_point(self):
        # 필요한 메시지 확인
        if self.depth_msg is None or self.camera_info is None or self.image_msg is None:
            return

        # CvBridge 변환
        try:
            depth_img = self.bridge.imgmsg_to_cv2(self.depth_msg, desired_encoding='passthrough')
            rgb_img = self.bridge.imgmsg_to_cv2(self.image_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge 변환 실패: {e}")
            return

        h, w = rgb_img.shape[:2]

        # ROI 값 안전하게 클램프
        x = max(0, min(self.roi_x, w - 1))
        y = max(0, min(self.roi_y, h - 1))
        w_roi = max(1, min(self.roi_w, w - x))
        h_roi = max(1, min(self.roi_h, h - y))

        # ROI 이미지 복사 (YOLO에 ROI만 전달)
        roi_img = rgb_img[y:y + h_roi, x:x + w_roi].copy()

        # (1) ROI 이미지 토픽 발행 (디버깅/다른 노드 사용)
        try:
            roi_msg = self.bridge.cv2_to_imgmsg(roi_img, encoding='bgr8')
            roi_msg.header.stamp = self.get_clock().now().to_msg()
            roi_msg.header.frame_id = self.image_topic
            self.roi_image_pub.publish(roi_msg)
        except Exception as e:
            self.get_logger().warn(f"ROI 이미지 퍼블리시 실패: {e}")

        # (2) YOLO 검출 (ROI 기반)
        detections = []  # 리스트 of dict
        if self.use_yolo and self.model is not None:
            try:
                results = self.model(roi_img)  # ultralytics 결과
                for result in results:
                    # OBB 우선 사용
                    boxes = getattr(result, 'obb', None)
                    if boxes is None:
                        # axis-aligned fallback
                        try:
                            xyxy = result.boxes.xyxy.cpu().numpy()
                            confs = result.boxes.conf.cpu().numpy()
                            cls_arr = result.boxes.cls.cpu().numpy()
                            names = getattr(result, "names", None)
                            for i in range(len(confs)):
                                conf = float(confs[i])
                                # 신뢰도 필터(요청: 70% 이하 무시)
                                if conf < self.confidence_threshold:
                                    continue
                                x1, y1, x2, y2 = map(int, xyxy[i].tolist())
                                cx = int((x1 + x2) / 2) + x  # full-image coords
                                cy = int((y1 + y2) / 2) + y
                                class_id = int(cls_arr[i])
                                class_name = names[class_id] if names is not None else None
                                detections.append({
                                    'class': class_name,
                                    'confidence': conf,
                                    'bbox': (x1 + x, y1 + y, x2 + x, y2 + y),
                                    'obb_corners': None,
                                    'angle_rad': None,
                                    'angle_deg': None,
                                    'center': (cx, cy)
                                })
                        except Exception:
                            # axis aligned parsing 실패 시 무시
                            continue
                    else:
                        # OBB 처리 (robust)
                        try:
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

                            # corners (있다면)
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

                            names = getattr(result, "names", None)

                            for i in range(len(classes)):
                                conf = float(confs[i])
                                if conf < self.confidence_threshold:
                                    continue
                                class_id = int(classes[i])
                                class_name = names[class_id] if names is not None else str(class_id)

                                # axis-aligned box on ROI if available
                                try:
                                    axyxy = result.boxes.xyxy.cpu().numpy()
                                    x1r, y1r, x2r, y2r = map(int, axyxy[i].tolist())
                                except Exception:
                                    # fallback 계산
                                    x1r = int(round(xywhr[i][0] - xywhr[i][2] / 2))
                                    y1r = int(round(xywhr[i][1] - xywhr[i][3] / 2))
                                    x2r = int(round(xywhr[i][0] + xywhr[i][2] / 2))
                                    y2r = int(round(xywhr[i][1] + xywhr[i][3] / 2))

                                # ROI-local -> full-image coords
                                x1_full = x1r + x
                                y1_full = y1r + y
                                x2_full = x2r + x
                                y2_full = y2r + y
                                cx = int((x1_full + x2_full) / 2)
                                cy = int((y1_full + y2_full) / 2)

                                angle_rad = None
                                angle_deg = None
                                if xywhr is not None and xywhr.shape[1] >= 5:
                                    angle_rad = float(xywhr[i][4])
                                    angle_deg = angle_rad * 180.0 / np.pi

                                obb_corners = None
                                if corners_flat is not None and corners_flat.shape[0] > i:
                                    arr = corners_flat[i].astype(np.float32)
                                    try:
                                        pts = arr.reshape(4, 2)
                                        # ROI-offset 적용
                                        pts[:, 0] += x
                                        pts[:, 1] += y
                                        obb_corners = pts.tolist()
                                    except Exception:
                                        obb_corners = None

                                detections.append({
                                    'class': class_name,
                                    'confidence': conf,
                                    'bbox': (x1_full, y1_full, x2_full, y2_full),
                                    'obb_corners': obb_corners,
                                    'angle_rad': angle_rad,
                                    'angle_deg': angle_deg,
                                    'center': (cx, cy)
                                })
                        except Exception as e:
                            self.get_logger().warn(f"OBB 파싱 실패: {e}")
                            continue

                self.get_logger().debug(f"YOLO 검출개수(ROI필터후): {len(detections)}")
            except Exception as e:
                self.get_logger().error(f"YOLO 감지 실패: {e}")
                detections = []
        else:
            # YOLO 미사용시(또는 모델 없음) 수동 픽셀 fallback
            detections = [{
                'class': 'manual',
                'confidence': 1.0,
                'bbox': None,
                'obb_corners': None,
                'angle_rad': None,
                'angle_deg': None,
                'center': (self.pixel_u, self.pixel_v)
            }]

        # (3) 모든 검출 결과를 JSON 문자열로 퍼블리시 (외부 노드가 필요하면 subscribe)
        try:
            dets_for_msg = []
            for d in detections:
                dets_for_msg.append({
                    'class': d['class'],
                    'confidence': float(d['confidence']),
                    'center': d['center'],
                    'angle_deg': (d['angle_deg'] if d['angle_deg'] is not None else None),
                    'bbox': d['bbox']
                })
            msg = String()
            msg.data = json.dumps(dets_for_msg)
            self.yolo_detections_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"검출 결과 퍼블리시 실패: {e}")

        # 준비: 화면 그리기 복사본
        cv_img = rgb_img.copy()

        # ROI 박스 표시
        cv2.rectangle(cv_img, (x, y), (x + w_roi, y + h_roi), (255, 0, 0), 2)

        # ------------------------
        # 우선순위 기반 선택 (priority_list에 있는 클래스가 먼저)
        # ------------------------
        selected_det = None
        for pname in self.priority_list:
            for det in detections:
                if det['class'] == pname:
                    selected_det = det
                    break
            if selected_det:
                break

        # 만약 우선순위에 해당하는 객체가 없으면 confidence 기준 최고 선택
        if selected_det is None and len(detections) > 0:
            detections = sorted(detections, key=lambda d: d['confidence'], reverse=True)
            selected_det = detections[0]

        if selected_det is None:
            # 검출된 대상이 없으면 'None' 퍼블리시
            class_msg = String()
            class_msg.data = "None"
            self.target_class_pub.publish(class_msg)
            
            # 화면만 업데이트
            cv2.imshow("Pixel to World - ROI + YOLO", cv_img)
            cv2.waitKey(1)
            return

        # 선택된 대상만 퍼블리시/시각화 (원래 동작과 동일)
        det = selected_det
        u, v = det['center']

        # OBB 시각화 (있다면)
        if det.get('obb_corners') is not None:
            try:
                pts = np.array(det['obb_corners'], np.int32).reshape((-1, 1, 2))
                cv2.polylines(cv_img, [pts], True, (0, 0, 255), 2)
                if det.get('angle_deg') is not None:
                    angle_txt = f"{det['angle_deg']:.1f} deg"
                    cv2.putText(cv_img, angle_txt, (u + 15, v + 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            except Exception:
                pass

        # bounds 체크
        if not (0 <= u < w and 0 <= v < h):
            self.get_logger().warn(f"선택된 중심이 이미지 밖: {(u, v)}")
            return

        # depth 안전히 읽기
        try:
            raw_depth = depth_img[v, u]
        except Exception as e:
            self.get_logger().warn(f"Depth 읽기 실패 at ({u},{v}): {e}")
            return

        if raw_depth == 0 or (isinstance(raw_depth, float) and math.isnan(raw_depth)):
            self.get_logger().warn(f"Depth 무효: ({u},{v})")
            return

        # depth 단위 변환 (16-bit -> mm)
        enc = getattr(self.depth_msg, 'encoding', None)
        if enc is not None and '16' in enc:
            depth_val = float(raw_depth) * 0.001  # mm -> m
        else:
            depth_val = float(raw_depth)
        # 혹시 굉장히 큰 값(예: >1000)일 경우 mm로 가정
        if depth_val > 1000.0:
            depth_val *= 0.001

        if depth_val <= 0:
            self.get_logger().warn("계산된 depth가 0 또는 음수")
            return

        # 카메라 내부 파라미터 (CameraInfo.k)
        K = self.camera_info.k
        fx = float(K[0]); fy = float(K[4]); cx_k = float(K[2]); cy_k = float(K[5])

        # 픽셀 -> 카메라 좌표 (m)
        x_cam = (u - cx_k) * depth_val / fx
        y_cam = (v - cy_k) * depth_val / fy
        z_cam = depth_val
        cam_coord = np.array([x_cam, y_cam, z_cam])

        # 회전 변환 (원래 pixel2world과 동일)
        theta_x = np.pi
        theta_y = 0.0
        theta_z = np.pi / 2.0

        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(theta_x), -np.sin(theta_x)],
            [0, np.sin(theta_x), np.cos(theta_x)]
        ])
        R_y = np.array([
            [np.cos(theta_y), 0, np.sin(theta_y)],
            [0, 1, 0],
            [-np.sin(theta_y), 0, np.cos(theta_y)]
        ])
        R_z = np.array([
            [np.cos(theta_z), -np.sin(theta_z), 0],
            [np.sin(theta_z), np.cos(theta_z), 0],
            [0, 0, 1]
        ])

        R_total = R_z @ R_y @ R_x
        world_coord = R_total @ cam_coord + self.camera_position

        # 각도 계산: 카메라/월드 기준 yaw, pitch (원하면 확장 가능)
        yaw_cam = math.degrees(math.atan2(x_cam, z_cam))
        pitch_cam = math.degrees(math.atan2(y_cam, z_cam))
        # world vector (camera -> target)
        vx, vy, vz = world_coord - self.camera_position
        yaw_world = math.degrees(math.atan2(vx, vz))
        pitch_world = math.degrees(math.atan2(vy, vz))

        # 추가: 단일 yaw angle (xy 평면)
        angle_rad = math.atan2(world_coord[1] - self.camera_position[1],
                               world_coord[0] - self.camera_position[0])
        angle_deg = math.degrees(angle_rad)

        # ------------------------
        # 퍼블리시: PointStamped, class, priority, angle info 등
        # ------------------------
        # PointStamped (world coords)
        pt_msg = PointStamped()
        pt_msg.header.frame_id = self.target_frame
        pt_msg.header.stamp = self.get_clock().now().to_msg()
        pt_msg.point.x = float(world_coord[0])
        pt_msg.point.y = float(world_coord[1])
        pt_msg.point.z = float(world_coord[2])
        self.target_pub.publish(pt_msg)

        # class publish
        class_msg = String()
        class_msg.data = det['class']
        self.target_class_pub.publish(class_msg)

        # priority publish: CSV (class,x,y,z,angle)
        pri_msg = String()
        pri_msg.data = f"{det['class']},{world_coord[0]:.3f},{world_coord[1]:.3f},{world_coord[2]:.3f},{angle_deg:.1f}"
        self.priority_pub.publish(pri_msg)

        # angle info publish: 각도값만 Float32로 (소수점 2자리)
        angle_msg = Float32()
        angle_msg.data = round(float(angle_deg), 2)
        self.angle_info_pub.publish(angle_msg)

        # 최신 angle_info 저장 (서비스 응답용)
        # geometry_msgs/Point 형식으로 x, y, z 포함
        self.latest_angle_info = {
            'class': det['class'],
            'x': round(float(world_coord[0]), 2),
            'y': round(float(world_coord[1]), 2),
            'z': round(float(world_coord[2]), 2),
            'angle_deg': round(float(angle_deg), 2),
            'yaw_cam': round(float(yaw_cam), 2),
            'pitch_cam': round(float(pitch_cam), 2),
            'yaw_world': round(float(yaw_world), 2),
            'pitch_world': round(float(pitch_world), 2),
            'confidence': round(float(det.get('confidence', 0.0)), 2)
        }

        # ------------------------
        # 화면에 표시: 클래스, 신뢰도, 좌표, 각도
        # ------------------------
        # 중심점과 텍스트 그리기
        color = (0, 255, 0) if det['class'] != 'manual' else (255, 255, 0)
        cv2.circle(cv_img, (u, v), 6, color, -1)

        # 신뢰도와 각도, 좌표 텍스트
        label = f"{det['class']} {det['confidence']*100:.0f}%"
        coord_txt = f"({world_coord[0]:.2f},{world_coord[1]:.2f},{world_coord[2]:.2f})"
        angle_txt = f"{angle_deg:.1f}deg"

        # 텍스트 위치 보정 (이미지 밖으로 나가지 않게)
        tx = max(10, min(u, w - 200))
        ty = max(30, min(v - 10, h - 30))

        cv2.putText(cv_img, label, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        cv2.putText(cv_img, coord_txt, (tx, ty + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(cv_img, angle_txt, (tx, ty + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # 최종 이미지 출력
        cv2.imshow("Pixel to World - ROI + YOLO", cv_img)
        cv2.waitKey(1)

        # (로그) 선택된 객체 정보
        self.get_logger().info(
            f"{det['class']} | conf={det['confidence']:.2f} | "
            f"world=({world_coord[0]:.3f},{world_coord[1]:.3f},{world_coord[2]:.3f}) | angle={angle_deg:.1f}"
        )

    # ------------------------
    # 서비스 핸들러: get_target_point
    # 요청: 비어있음, 응답: geometry_msgs/Point + class_name + confidence
    # ------------------------
    def handle_get_target_point(self, request, response):
        self.get_logger().info('Received request for /get_target_point')

        if self.latest_angle_info:
            payload = {
                'class': self.latest_angle_info['class'],
                'center' : [self.latest_angle_info['x'], self.latest_angle_info['y'], self.latest_angle_info['z']],
                'x': self.latest_angle_info['x'],
                'y': self.latest_angle_info['y'],
                'z': self.latest_angle_info['z'],
                'angle_deg': self.latest_angle_info['angle_deg'],
                'yaw_cam': self.latest_angle_info['yaw_cam'],
                'pitch_cam': self.latest_angle_info['pitch_cam'],
                'yaw_world': self.latest_angle_info['yaw_world'],
                'pitch_world': self.latest_angle_info['pitch_world'],
                'confidence': self.latest_angle_info['confidence']
            }

            response.success = True
            response.message = json.dumps(payload)

            self.get_logger().info(
                f"Sent target info: {response.message}"
            )
        else:
            response.success = False
            response.message = "None"

            self.get_logger().warn("No target available, returning None")

        return response


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
        cv2.destroy_all_windows()


if __name__ == '__main__':
    main()
