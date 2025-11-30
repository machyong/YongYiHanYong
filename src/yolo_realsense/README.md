# yolo_realsense

ROS2 + YOLO + RealSense 통합 객체 검출 및 거리 측정 패키지

## 🎯 주요 기능

- **YOLO 객체 검출**: YOLOv5/YOLOv8 지원
- **깊이 측정**: RealSense Depth 카메라로 객체까지 거리 측정
- **실시간 시각화**: Color + Depth 이미지를 나란히 표시
- **ROS2 통합**: 검출 결과를 ROS 토픽으로 퍼블리시
- **모델 교체 가능**: 커스텀 학습 모델 쉽게 적용

## 📦 패키지 구성

### 노드
- **yolo_depth_viewer**: YOLO + RealSense 통합 뷰어 (메인 노드)

### 메시지
- `Detection.msg`: 단일 객체 검출 정보 (클래스, 신뢰도, 거리, bbox)
- `Detections.msg`: Detection 배열
- `ObjectDepth.msg`: 단일 객체 깊이 정보
- `ObjectDepthArray.msg`: ObjectDepth 배열

### Launch 파일
- `yolo_all.launch.py`: RealSense 카메라 + YOLO 통합 실행

### 설정 파일
- `config/yolo_params.yaml`: YOLO 모델 설정

## 🔧 의존성

### ROS2 패키지
```bash
sudo apt install ros-${ROS_DISTRO}-cv-bridge \
                 ros-${ROS_DISTRO}-sensor-msgs \
                 ros-${ROS_DISTRO}-std-msgs \
                 ros-${ROS_DISTRO}-realsense2-camera
```

### Python 패키지
```bash
pip3 install opencv-python \
             torch torchvision \
             ultralytics
```

## 🚀 빌드 및 설치

```bash
cd ~/ros2_ws
colcon build --packages-select yolo_realsense
source install/setup.bash
```

## 📖 사용법

### 🎬 빠른 시작 (권장)

카메라와 YOLO를 한번에 실행:

```bash
# 기본 실행 (YOLOv5s)
ros2 launch yolo_realsense yolo_all.launch.py

# YOLOv8 nano 사용
ros2 launch yolo_realsense yolo_all.launch.py model_type:=yolov8 model_name:=yolov8n

# YOLOv8 medium 사용
ros2 launch yolo_realsense yolo_all.launch.py model_type:=yolov8 model_name:=yolov8m

# 커스텀 학습 모델 사용
ros2 launch yolo_realsense yolo_all.launch.py model_path:=/path/to/your/best.pt
```

### 🔧 노드만 개별 실행

RealSense 카메라가 이미 실행 중일 때:

```bash
# 기본 YOLOv5s
ros2 run yolo_realsense yolo_depth_viewer

# YOLOv5 large
ros2 run yolo_realsense yolo_depth_viewer --ros-args -p model_name:=yolov5l

# YOLOv8 사용
ros2 run yolo_realsense yolo_depth_viewer --ros-args -p model_type:=yolov8 -p model_name:=yolov8n

# 커스텀 모델
ros2 run yolo_realsense yolo_depth_viewer --ros-args -p model_path:=/home/user/models/best.pt
```

### ⚙️ 설정 파일로 모델 변경

`config/yolo_params.yaml` 수정:

```yaml
yolo_depth_node:
  ros__parameters:
    model_type: 'yolov8'       # yolov5 또는 yolov8
    model_name: 'yolov8n'      # yolov5s, yolov8n, yolov8m 등
    model_path: ''             # 커스텀 모델 경로
```

수정 후 빌드하고 실행:
```bash
colcon build --packages-select yolo_realsense
source install/setup.bash
ros2 launch yolo_realsense yolo_all.launch.py
```

## �️ 화면 구성

실행 시 하나의 창에 다음 정보가 표시됩니다:

- **왼쪽**: Color 이미지 + YOLO 검출 결과
  - 초록색 바운딩 박스
  - 클래스명 + 신뢰도 + 거리 (예: "person 0.85 | 1.23m")
  - 중앙 픽셀 깊이 정보
  - 검출된 객체 수
  
- **오른쪽**: Depth 이미지 (컬러맵)
  - JET 컬러맵으로 거리 시각화
  - 가까운 곳: 파란색
  - 먼 곳: 빨간색

## 📡 토픽 정보

### 구독 (Subscribe)
- `/camera/camera/color/image_raw` (sensor_msgs/Image) - Color 이미지
- `/camera/camera/depth/image_rect_raw` (sensor_msgs/Image) - Depth 이미지
- `/camera/camera/color/camera_info` (sensor_msgs/CameraInfo) - 카메라 정보

### 발행 (Publish)
- `/yolo_depth_info` (std_msgs/String) - YOLO 검출 결과 JSON

검출 결과 예시:
```json
[
  {
    "class": "person",
    "confidence": 0.85,
    "distance_m": 1.234,
    "bbox": [100, 200, 300, 400]
  }
]
```

### 결과 확인
```bash
ros2 topic echo /yolo_depth_info
```

## 🤖 지원 YOLO 모델

### YOLOv5 (torch.hub)
- `yolov5n` - Nano (가장 빠름)
- `yolov5s` - Small (기본값)
- `yolov5m` - Medium
- `yolov5l` - Large
- `yolov5x` - Extra Large (가장 정확)

### YOLOv8 (ultralytics)
- `yolov8n` - Nano (가장 빠름)
- `yolov8s` - Small
- `yolov8m` - Medium
- `yolov8l` - Large
- `yolov8x` - Extra Large (가장 정확)

### 커스텀 모델
직접 학습한 `.pt` 파일을 사용할 수 있습니다:
```bash
ros2 launch yolo_realsense yolo_all.launch.py model_path:=/path/to/custom.pt
```

## 🛠️ 트러블슈팅

### 카메라 토픽이 안 보일 때
```bash
# RealSense 카메라 확인
rs-enumerate-devices

# ROS2 토픽 확인
ros2 topic list | grep camera
```

### YOLO 모델 로딩 실패
- 인터넷 연결 확인 (처음 실행 시 모델 다운로드)
- PyTorch 설치 확인: `python3 -c "import torch; print(torch.__version__)"`
- Ultralytics 설치 확인: `python3 -c "import ultralytics"`

### 화면이 안 뜰 때
- X11 forwarding 확인 (SSH 사용 시)
- OpenCV 설치 확인: `python3 -c "import cv2"`

## 📝 라이선스
Apache License 2.0

## 👤 메인테이너
- csw (csw@todo.todo)

## 🙏 감사
- [Ultralytics YOLOv5](https://github.com/ultralytics/yolov5)
- [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics)
- [Intel RealSense](https://github.com/IntelRealSense/realsense-ros)
