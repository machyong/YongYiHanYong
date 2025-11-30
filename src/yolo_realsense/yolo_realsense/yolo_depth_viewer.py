#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import json
import warnings

import cv2
import numpy as np
import torch

# PyTorch FutureWarning 숨기기
warnings.filterwarnings('ignore', category=FutureWarning)


class YOLODepthViewer(Node):
    def __init__(self):
        super().__init__('yolo_depth_viewer')

        self.bridge = CvBridge()
        self.color_frame = None
        self.depth_frame = None
        self.camera_info = None

        self.publisher_ = self.create_publisher(String, 'yolo_depth_info', 10)
        self.get_logger().info("🚀 YOLO + Depth Viewer Started")

        # 파라미터 선언
        self.declare_parameter('model_name', 'yolov5s')
        self.declare_parameter('model_path', '')
        self.declare_parameter('model_type', 'yolov5')
        
        model_name = self.get_parameter('model_name').value
        model_path = self.get_parameter('model_path').value
        model_type = self.get_parameter('model_type').value

        # YOLO 모델 로드
        try:
            if model_path:
                self.get_logger().info(f"Loading custom model from: {model_path}")
                if model_type == 'yolov8':
                    from ultralytics import YOLO
                    self.model = YOLO(model_path)
                    self.model_type = 'yolov8'
                else:
                    self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)
                    self.model_type = 'yolov5'
            else:
                self.get_logger().info(f"Loading pretrained model: {model_name}")
                if model_type == 'yolov8' or model_name.startswith('yolov8'):
                    from ultralytics import YOLO
                    self.model = YOLO(f'{model_name}.pt')
                    self.model_type = 'yolov8'
                else:
                    self.model = torch.hub.load('ultralytics/yolov5', model_name, pretrained=True)
                    self.model_type = 'yolov5'
            
            self.get_logger().info(f"✅ YOLO Model Loaded: {model_name if not model_path else model_path} (Type: {self.model_type})")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            raise

        # ROS 토픽 구독
        self.create_subscription(Image, '/camera/camera/color/image_raw',
                                 self.color_callback, 10)
        self.create_subscription(Image, '/camera/camera/depth/image_rect_raw',
                                 self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info',
                                 self.info_callback, 10)
        
        self.get_logger().info("📡 Subscribed to camera topics")
        self.timer = self.create_timer(0.1, self.process)

    def color_callback(self, msg):
        try:
            self.color_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Color conversion failed: {e}")

    def depth_callback(self, msg):
        try:
            self.depth_frame = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        except Exception as e:
            self.get_logger().error(f"Depth conversion failed: {e}")

    def info_callback(self, msg):
        if self.camera_info is None:
            self.get_logger().info("📐 CameraInfo received")
        self.camera_info = msg

    def process(self):
        if self.color_frame is None or self.depth_frame is None:
            return

        try:
            color_img = self.color_frame.copy()
            depth_img = self.depth_frame.copy()

            # 중앙 픽셀 깊이 정보
            h, w = depth_img.shape[:2]
            cx, cy = w // 2, h // 2
            depth_raw = depth_img[cy, cx]
            center_depth_m = depth_raw * 0.001

            # YOLO detection
            results = self.model(color_img)
            detections = []

            # YOLOv5와 YOLOv8 결과 형식이 다름
            if self.model_type == 'yolov8':
                for result in results:
                    boxes = result.boxes
                    for box in boxes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                        conf = float(box.conf[0])
                        class_id = int(box.cls[0])
                        class_name = result.names[class_id]

                        roi = depth_img[y1:y2, x1:x2]
                        if roi.size == 0:
                            continue

                        distance_mm = float(np.median(roi))
                        distance_m = distance_mm * 0.001

                        detections.append({
                            "class": class_name,
                            "confidence": float(conf),
                            "distance_m": round(distance_m, 3),
                            "bbox": [x1, y1, x2, y2]
                        })

                        # 바운딩 박스 그리기
                        cv2.rectangle(color_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        label = f"{class_name} {conf:.2f} | {distance_m:.2f}m"
                        label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
                        cv2.rectangle(color_img, (x1, y1 - label_size[1] - 10), 
                                     (x1 + label_size[0], y1), (0, 255, 0), -1)
                        cv2.putText(color_img, label, (x1, y1 - 5),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            else:
                # YOLOv5
                for det in results.xyxy[0]:
                    x1, y1, x2, y2, conf, class_id = det
                    x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
                    class_name = self.model.names[int(class_id)]

                    roi = depth_img[y1:y2, x1:x2]
                    if roi.size == 0:
                        continue

                    distance_mm = float(np.median(roi))
                    distance_m = distance_mm * 0.001

                    detections.append({
                        "class": class_name,
                        "confidence": float(conf),
                        "distance_m": round(distance_m, 3),
                        "bbox": [x1, y1, x2, y2]
                    })

                    cv2.rectangle(color_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    label = f"{class_name} {conf:.2f} | {distance_m:.2f}m"
                    label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
                    cv2.rectangle(color_img, (x1, y1 - label_size[1] - 10), 
                                 (x1 + label_size[0], y1), (0, 255, 0), -1)
                    cv2.putText(color_img, label, (x1, y1 - 5),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

            # 중앙 깊이 표시
            cv2.circle(color_img, (cx, cy), 6, (0, 0, 255), -1)
            cv2.putText(color_img, f"Center Depth: {center_depth_m:.3f} m",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            cv2.putText(color_img, f"Objects: {len(detections)}",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # Depth 시각화
            depth_vis = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX)
            depth_vis = depth_vis.astype(np.uint8)
            depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
            
            # Color 이미지 크기에 맞춰 Depth 리사이즈
            color_h, color_w = color_img.shape[:2]
            depth_vis = cv2.resize(depth_vis, (color_w, color_h))
            
            # 중앙 표시 (리사이즈 후 다시 계산)
            cv2.circle(depth_vis, (cx, cy), 6, (255, 255, 255), -1)

            # 두 화면을 나란히 표시
            combined = np.hstack((color_img, depth_vis))
            cv2.imshow("YOLO + Depth Viewer (Color | Depth)", combined)
            cv2.waitKey(1)

            # JSON 메시지 퍼블리시
            msg = String()
            msg.data = json.dumps(detections)
            self.publisher_.publish(msg)

            if detections:
                self.get_logger().info(f"Detected {len(detections)} objects", throttle_duration_sec=2.0)
                
        except Exception as e:
            self.get_logger().error(f"Processing error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = YOLODepthViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
