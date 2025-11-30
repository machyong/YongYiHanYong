from setuptools import setup
import os
from glob import glob

package_name = 'yolo_realsense'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='csw',
    maintainer_email='csw@todo.todo',
    description='YOLO + RealSense Depth object detection and distance measurement',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'yolo_depth_viewer = yolo_realsense.yolo_depth_viewer:main',
            'picxel2world = yolo_realsense.picxel2world:main',
            # yolo_result_publisher 추가 (이미지 퍼블리셔 + 좌표 + 각도)
            'yolo_result_publisher = yolo_realsense.yolo_result_publisher:main',
            # priority_object 추가 (우선순위 객체 퍼블리셔)
            'priority_object = yolo_realsense.priority_object:main',
            # world_trigger 추가 (트리거 기반 월드좌표 계산)
            'world_trigger = yolo_realsense.world_trigger:main',

        ],
    },
)

