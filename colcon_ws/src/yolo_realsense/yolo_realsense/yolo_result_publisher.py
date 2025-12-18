#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from ultralytics import YOLO
import warnings

warnings.filterwarnings('ignore', category=FutureWarning)


class YoloResultWithWorldPublisher(Node):
    def __init__(self):
        super().__init__('yolo_result_with_world_publisher')

        # -------------------------
        # Parameters
        # -------------------------
        self.declare_parameter('model_path', '/home/min/proj_ws/src/yolo_realsense/yolo_realsense/best.pt')
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('target_frame', 'world')
        self.declare_parameter('camera_position', [0.52, 0.20, 0.89])

        self.model_path = self.get_parameter('model_path').value
        self.image_topic = self.get_parameter('image_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.target_frame = self.get_parameter('target_frame').value
        self.camera_position = np.array(self.get_parameter('camera_position').value, dtype=float)

        # -------------------------
        # State
        # -------------------------
        self.bridge = CvBridge()
        self.camera_info = None
        self.latest_depth = None

        # -------------------------
        # Load YOLO
        # -------------------------
        try:
            self.get_logger().info(f"Loading YOLO model: {self.model_path}")
            self.model = YOLO(self.model_path)
            self.get_logger().info("YOLO model loaded")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            raise

        # -------------------------
        # Publishers
        # -------------------------
        self.result_pub = self.create_publisher(Image, '/yolo/result_image', 10)
        self.target_pub = self.create_publisher(PointStamped, '/target_object', 10)
        self.target_class_pub = self.create_publisher(String, '/target_object_class', 10)
        self.priority_pub = self.create_publisher(String, '/priority_object', 10)

        # -------------------------
        # Subscribers
        # -------------------------
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_cb, 10)
        self.create_subscription(Image, self.depth_topic, self.depth_cb, 10)
        self.create_subscription(Image, self.image_topic, self.image_cb, 10)

        self.get_logger().info("YOLO world publisher started")

    # ============================================================
    # CALLBACKS
    # ============================================================

    def camera_info_cb(self, msg: CameraInfo):
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info("CameraInfo received")

    def depth_cb(self, msg: Image):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f"Depth conversion failed: {e}")
            self.latest_depth = None

    def image_cb(self, msg: Image):
        try:
            color_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Color conversion failed: {e}")
            return

        if self.camera_info is None or self.latest_depth is None:
            self.get_logger().warn("Waiting for depth or camera info...")
            return

        # YOLO inference
        try:
            results = self.model(color_bgr)
        except Exception as e:
            self.get_logger().error(f"YOLO inference failed: {e}")
            return

        out_img = color_bgr.copy()
        h, w = out_img.shape[:2]

        detections = []
        names = results[0].names

        # ========== YOLO box parsing ==========
        for res in results:
            boxes = res.boxes

            # -----------------------------
            # (1) 박스가 없는 경우 방어 처리
            # -----------------------------
            if boxes is None or boxes.xyxy is None:
                continue  # 다음 프레임 처리

            xyxy = boxes.xyxy.cpu().numpy()
            confs = boxes.conf.cpu().numpy()
            cls = boxes.cls.cpu().numpy()

            # -----------------------------
            # (2) 박스 배열이 비어 있는 경우
            # -----------------------------
            if len(xyxy) == 0:
                continue


            for i in range(len(confs)):
                if confs[i] < self.confidence_threshold:
                    continue

                x1, y1, x2, y2 = xyxy[i].tolist()
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                detections.append({
                    'class': names[int(cls[i])],
                    'confidence': float(confs[i]),
                    'bbox': (int(x1), int(y1), int(x2), int(y2)),
                    'center': (cx, cy)
                })

        if len(detections) == 0:
            plotted = results[0].plot()
            ros_img = self.bridge.cv2_to_imgmsg(plotted, encoding='bgr8')
            self.result_pub.publish(ros_img)
            return

        # priority selection
        priority_list = ["cutlery", "dish", "cup"]
        selected_det = None

        for item in priority_list:
            for det in detections:
                if det['class'] == item:
                    selected_det = det
                    break
            if selected_det:
                break

        if selected_det is None:
            detections = sorted(detections, key=lambda d: d['confidence'], reverse=True)
            selected_det = detections[0]

        # draw detections
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            cx, cy = det['center']
            cv2.rectangle(out_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(out_img, (cx, cy), 4, (255, 0, 0), -1)
            cv2.putText(out_img, det['class'], (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        # ===== world coordinate computation =====
        u, v = selected_det['center']
        raw_depth = float(self.latest_depth[v, u])

        if raw_depth == 0:
            self.get_logger().warn("Depth=0")
            return

        depth_m = raw_depth * 0.001  # mm→m

        K = self.camera_info.k
        fx, fy = K[0], K[4]
        cx_k, cy_k = K[2], K[5]

        x_cam = (u - cx_k) * depth_m / fx
        y_cam = (v - cy_k) * depth_m / fy
        z_cam = depth_m

        cam_coord = np.array([x_cam, y_cam, z_cam])

        # rotation (same as pixel2world)
        R_x = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        R_z = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        R_total = R_z @ R_x

        world_coord = R_total @ cam_coord + self.camera_position

        # ===== Publish =====
        pt = PointStamped()
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.header.frame_id = self.target_frame
        pt.point.x, pt.point.y, pt.point.z = world_coord
        self.target_pub.publish(pt)

        cls_msg = String()
        cls_msg.data = selected_det['class']
        self.target_class_pub.publish(cls_msg)

        pri_msg = String()
        pri_msg.data = f"{selected_det['class']},{world_coord[0]:.3f},{world_coord[1]:.3f},{world_coord[2]:.3f}"
        self.priority_pub.publish(pri_msg)

        # image publish
        ros_img = self.bridge.cv2_to_imgmsg(out_img, encoding='bgr8')
        self.result_pub.publish(ros_img)


def main(args=None):
    rclpy.init(args=args)
    node = YoloResultWithWorldPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
