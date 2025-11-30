# RealSense D455f 카메라 기반 로봇 제어 시스템

Ubuntu 24.04 환경에서 RealSense D455f 카메라와 두산 로봇 e0509를 사용하여 마우스 클릭으로 로봇을 제어하는 시스템입니다.

## 📁 파일 설명

### pixel1.py - 기본 마우스 클릭 및 좌표 변환
- RealSense 카메라 화면에서 마우스 클릭
- 픽셀 좌표를 World 좌표로 변환하여 출력
- 로봇 제어 없이 좌표 변환만 테스트

**실행 방법:**
```bash
python3 pixel1.py
```

**기능:**
- 카메라 화면에서 마우스 클릭으로 위치 선택
- 클릭한 픽셀의 World 좌표 계산 및 출력
- 시각적 피드백 제공

---

### pixel2.py - 두산 로봇 제어 추가
- pixel1.py의 모든 기능 포함
- 클릭한 좌표로 두산 e0509 로봇 이동
- 데모 모드 지원 (로봇 없이 테스트 가능)

**실행 방법:**
```bash
# 로봇 연결 시
python3 pixel2.py

# 데모 모드 (로봇 없이)
python3 pixel2.py --ros-args -p demo_mode:=true
```

**기능:**
- 마우스 클릭으로 로봇 이동 명령
- 두산 로봇 MoveL 직선 이동
- 비동기 로봇 제어 (UI 블로킹 방지)
- 데모 모드 지원

**주요 파라미터:**
- `robot_id`: 로봇 ID (기본값: 'dsr01')
- `robot_model`: 로봇 모델 (기본값: 'e0509')
- `demo_mode`: 데모 모드 활성화 (기본값: false)

---

### pixel3.py - 안전 기능 및 시각화 개선 ⭐ 추천
- pixel2.py의 모든 기능 포함
- 안전 범위 체크
- 이동 궤적 표시
- 향상된 시각적 피드백

**실행 방법:**
```bash
# 로봇 연결 시
python3 pixel3.py

# 데모 모드 + 커스텀 안전 범위
python3 pixel3.py --ros-args \
  -p demo_mode:=true \
  -p min_x:=0.2 -p max_x:=0.8 \
  -p min_y:=-0.3 -p max_y:=0.3 \
  -p min_z:=0.1 -p max_z:=0.8
```

**기능:**
- ✅ 안전 범위 체크 (X, Y, Z 제한)
- 📍 이동 궤적 표시 (최근 50개)
- 🎨 시각적 피드백 개선
  - 안전/위험 영역 색상 표시 (녹색/빨간색)
  - 로봇 상태 표시 (이동 중/대기)
  - 안전 범위 정보 표시
- 🔒 중복 클릭 방지 (로봇 이동 중)
- ⌨️ 키보드 단축키
  - `q`: 종료
  - `c`: 궤적 초기화

**주요 파라미터:**
- 안전 범위 (미터 단위):
  - `min_x`, `max_x`: X축 범위 (기본값: 0.0 ~ 1.0)
  - `min_y`, `max_y`: Y축 범위 (기본값: -0.5 ~ 0.5)
  - `min_z`, `max_z`: Z축 범위 (기본값: 0.0 ~ 1.0)
- 로봇 속도:
  - `velocity`: 이동 속도 (기본값: 100)
  - `acceleration`: 가속도 (기본값: 100)

---

## 🔧 시스템 요구사항

### 하드웨어
- Intel RealSense D455f 카메라
- 두산 로봇 e0509
- Ubuntu 24.04 PC

### 소프트웨어
```bash
# ROS 2 Humble
sudo apt install ros-humble-desktop

# RealSense SDK
sudo apt install ros-humble-realsense2-camera

# Python 의존성
pip3 install opencv-python numpy scipy
```

### 두산 로봇 라이브러리
```bash
# 두산 로봇 Python SDK 설치 필요
# DR_init 모듈이 시스템에 설치되어 있어야 합니다
```

---

## 🚀 빠른 시작

### 1. RealSense 카메라 실행
```bash
ros2 run realsense2_camera realsense2_camera_node
```

### 2. 좌표 변환 테스트 (로봇 제어 없이)
```bash
python3 pixel1.py
```

### 3. 로봇 제어 (데모 모드)
```bash
python3 pixel3.py --ros-args -p demo_mode:=true
```

### 4. 실제 로봇 제어
```bash
# 안전 범위를 실제 작업 환경에 맞게 설정
python3 pixel3.py --ros-args \
  -p demo_mode:=false \
  -p min_x:=0.3 -p max_x:=0.7 \
  -p min_y:=-0.2 -p max_y:=0.2 \
  -p min_z:=0.2 -p max_z:=0.6 \
  -p velocity:=50 \
  -p acceleration:=50
```

---

## 📐 좌표계 정보

### 카메라 위치
- World 프레임 기준: `(0.52m, 0.20m, 0.89m)`
- 회전: X축 180°, Z축 90°

### 좌표 변환 과정
1. **픽셀 → 카메라 좌표**: 내부 파라미터(fx, fy, cx, cy) 사용
2. **깊이 값**: RealSense Depth 이미지 (16bit, mm 단위)
3. **카메라 → World**: 회전 행렬 + 이동 벡터
4. **World → 로봇**: 밀리미터(mm) 단위로 변환

---

## ⚠️ 안전 주의사항

1. **처음 사용 시**
   - 반드시 데모 모드로 테스트
   - 안전 범위를 좁게 설정
   - 로봇 비상정지 버튼 확인

2. **운영 중**
   - 로봇 작업 영역에 사람이 없는지 확인
   - 안전 펜스 설치 권장
   - 로봇 속도를 낮게 설정 (velocity < 100)

3. **이상 발생 시**
   - `q` 키로 즉시 프로그램 종료
   - 로봇 비상정지 버튼 사용

---

## 🐛 문제 해결

### 카메라가 인식되지 않음
```bash
# RealSense 카메라 확인
rs-enumerate-devices

# ROS 2 노드 재시작
ros2 run realsense2_camera realsense2_camera_node
```

### 로봇 연결 실패
- 두산 로봇 SDK 설치 확인
- 로봇 IP 주소 및 네트워크 연결 확인
- 데모 모드로 우선 테스트

### Depth 값이 0으로 표시
- 표면이 너무 반사적이거나 어두운 경우
- 카메라와 대상 거리 조정 (D455f 권장: 0.6m ~ 6m)
- 조명 개선

---

## 📝 커스터마이징

### 카메라 위치 변경
```python
# 파일 내 camera_position 수정
self.camera_position = np.array([0.52, 0.20, 0.89])  # X, Y, Z (미터)
```

### 로봇 자세 변경
```python
# move_robot_to_target 함수 내
rx, ry, rz = 0, 180, 0  # Roll, Pitch, Yaw (도)
```

### ROS Topic 변경
```bash
python3 pixel3.py --ros-args \
  -p depth_topic:=/my_camera/depth/image_raw \
  -p image_topic:=/my_camera/color/image_raw \
  -p camera_info_topic:=/my_camera/color/camera_info
```

---

## 📞 기술 지원

문제가 발생하거나 질문이 있으시면 이슈를 등록해주세요.

---

**작성일**: 2025년 11월 27일  
**환경**: Ubuntu 24.04, ROS 2 Humble, RealSense D455f, Doosan e0509
