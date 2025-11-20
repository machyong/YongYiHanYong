cmake_minimum_required(VERSION 3.5)
project(yolo_realsense)

# 메시지 생성 도구 포함
find_package(rosidl_default_generators REQUIRED)

# 메시지 파일 빌드
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  "msg/Detections.msg"
  "msg/ObjectDepth.msg"
  "msg/ObjectDepthArray.msg"
)

# 파이썬 설치 설정
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

# 메시지 런타임 의존성
ament_export_dependencies(rosidl_default_runtime)

ament_package()

