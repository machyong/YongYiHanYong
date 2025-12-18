#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # -----------------------------
    # 기본 경로 설정
    # -----------------------------
    pkg_share = get_package_share_directory('yolo_realsense')

    # RealSense 패키지 launch 경로
    realsense_pkg = get_package_share_directory('realsense2_camera')
    realsense_launch = os.path.join(realsense_pkg, "launch", "rs_launch.py")

    # -----------------------------
    # Launch Arguments
    # -----------------------------
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/home/up/YongYiHanYong/src/yolo_realsense/yolo_realsense/best.pt',
        description='YOLO 학습 가중치(.pt)'
    )

    align_depth_arg = DeclareLaunchArgument(
        'align_depth_enable',
        default_value='true',
        description='Align Depth 활성화 여부'
    )

    # -----------------------------
    # RealSense 카메라 노드 실행
    # -----------------------------
    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'true',
            'align_depth.enable': LaunchConfiguration('align_depth_enable'),
        }.items()
    )

    # -----------------------------
    # PixelToWorld 노드 실행
    # -----------------------------
    pixel_to_world_node = Node(
        package='yolo_realsense',
        executable='priority_object',   # ← 너의 python 노드 이름
        name='priority_object_node',
        output='screen',
        parameters=[
            {
                "model_path": LaunchConfiguration("model_path"),
                "use_yolo": True,
                "confidence_threshold": 0.7,

                # ROI 기본값 (원하면 launch에서 수정 가능)
                "roi_x": 700,
                "roi_y": 20,
                "roi_w": 450,
                "roi_h": 570,

                # 카메라 토픽 이름
                "depth_topic": "/camera/camera/aligned_depth_to_color/image_raw",
                "image_topic": "/camera/camera/color/image_raw",
                "camera_info_topic": "/camera/camera/color/camera_info",
            }
        ]
    )

    return LaunchDescription([
        model_path_arg,
        align_depth_arg,
        realsense_node,
        pixel_to_world_node
    ])
