#!/usr/bin/env python3
# ROI+YOLO+Depth→World 좌표 계산을 서비스 Trigger 호출 시에만 수행하는 구조
# 기존 코드의 동작을 유지하면서 "트리거 기반"으로 바꿈

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import json
from ultralytics import YOLO

class PixelToWorldTrigger(Node):
    """Trigger 호출 시 YOLO -> depth -> world 좌표 계산을 수행하는 노드"""

    def __init__(self):
        super().__init__('pixel_to_world_trigger')

        # ---------------------------
        # 파라미터 선언
        # ---------------------------
        self.declare_parameter('roi_x', 700)
        self.declare_parameter('roi_y', 20)
        self.declare_parameter('roi_w', 450)
        self.declare_parameter('roi_h', 570)

        self.declare_parameter('confidence_threshold', 0.7)
        self.declare_parameter('model_path', '/home/min/proj_ws/src/yolo_realsense/yolo_realsense/best.pt')

        # 파라미터 로드
        self.roi_x = self.get_parameter('roi_x').value
        self.roi_y = self.get_parameter('roi_y').value
        self.roi_w = self.get_parameter('roi_w').value
        self.roi_h = self.get_parameter('roi_h').value
        self.conf_thr = self.get_parameter('confidence_threshold').value

        # YOLO 모델 로드
        self.model = YOLO(self.get_parameter('model_path').value)

        # 상태 변수
        self.bridge = CvBridge()
        self.depth_msg = None
        self.image_msg = None
        self.camera_info = None

        # ---------------------------
        # Subscribers
        # ---------------------------
        self.create_subscription(Image, "/camera/camera/color/image_raw", self.image_cb, 10)
        self.create_subscription(Image, "/camera/camera/aligned_depth_to_color/image_raw", self.depth_cb, 10)
        self.create_subscription(CameraInfo, "/camera/camera/color/camera_info", self.camera_info_cb, 10)

        # ---------------------------
        # Publishers (결과 전송)
        # ---------------------------
        self.result_pub = self.create_publisher(String, "/trigger_detection_result", 10)

        # ---------------------------
        # Trigger Service 생성
        # ---------------------------
        self.srv = self.create_service(Trigger, "/run_detection", self.trigger_callback)

        self.get_logger().info("🚀 PixelToWorld Trigger Node Ready (서비스 호출 시에만 YOLO 실행)")

    # --------------------------------------------------------
    # 콜백 - 카메라 이미지 저장
    # --------------------------------------------------------
    def image_cb(self, msg):
        self.image_msg = msg

    def depth_cb(self, msg):
        self.depth_msg = msg

    def camera_info_cb(self, msg):
        self.camera_info = msg

    # --------------------------------------------------------
    # Trigger 서비스 콜백 (여기서 YOLO → depth → world 계산 실행)
    # --------------------------------------------------------
    def trigger_callback(self, request, response):

        if self.image_msg is None or self.depth_msg is None or self.camera_info is None:
            response.success = False
            response.message = "카메라 데이터가 아직 준비되지 않음"
            return response

        # YOLO 실행
        rgb = self.bridge.imgmsg_to_cv2(self.image_msg, "bgr8")
        depth = self.bridge.imgmsg_to_cv2(self.depth_msg, "passthrough")

        # ROI 설정
        x, y, w, h = self.roi_x, self.roi_y, self.roi_w, self.roi_h
        roi = rgb[y:y+h, x:x+w]

        results = self.model(roi)
        detections = []

        names = results[0].names
        for r in results:
            xyxy = r.boxes.xyxy.cpu().numpy()
            confs = r.boxes.conf.cpu().numpy()
            cls_arr = r.boxes.cls.cpu().numpy()

            for i in range(len(confs)):
                if confs[i] < self.conf_thr:
                    continue

                x1, y1, x2, y2 = map(int, xyxy[i])
                cx = int((x1 + x2) / 2) + x
                cy = int((y1 + y2) / 2) + y

                detections.append({
                    "class": names[int(cls_arr[i])],
                    "confidence": float(confs[i]),
                    "center": (cx, cy)
                })

        # 검출 없으면 실패 반환
        if len(detections) == 0:
            response.success = False
            response.message = "검출된 객체 없음"
            return response

        det = detections[0]  # 우선 1개만 사용

        # ---------------------------
        # depth → world 계산
        # ---------------------------
        u, v = det["center"]
        depth_val = float(depth[v, u]) * 0.001  # mm → m

        K = self.camera_info.k
        fx, fy = K[0], K[4]
        cx_k, cy_k = K[2], K[5]

        X = (u - cx_k) * depth_val / fx
        Y = (v - cy_k) * depth_val / fy
        Z = depth_val

        # world 좌표(카메라 위치 더해줄 수 있음)
        world_x = round(X, 2)
        world_y = round(Y, 2)
        world_z = round(Z, 2)

        # 각도 계산
        angle_rad = math.atan2(Y, X)
        angle_deg = round(math.degrees(angle_rad), 2)

        # ---------------------------
        # JSON 결과 생성
        # ---------------------------
        result = {
            "class": det["class"],
            "x": world_x,
            "y": world_y,
            "z": world_z,
            "angle_deg": angle_deg
        }

        # 퍼블리시
        msg = String()
        msg.data = json.dumps(result)
        self.result_pub.publish(msg)

        # 서비스 응답
        response.success = True
        response.message = json.dumps(result)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PixelToWorldTrigger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
