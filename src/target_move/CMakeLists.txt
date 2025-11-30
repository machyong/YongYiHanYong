cmake_minimum_required(VERSION 3.8)
project(target_move)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yolo_depth_interfaces REQUIRED)

# 실행 파일 이름 변경: target_move -> target_move_node
add_executable(target_move_node src/target_move.cpp)
target_include_directories(target_move_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(target_move_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  target_move_node
  "moveit_ros_planning_interface"
  "rclcpp"
  "geometry_msgs"
  "yolo_depth_interfaces"
)

install(TARGETS target_move_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ament 패키지 선언 (반드시 마지막에)
ament_package()
