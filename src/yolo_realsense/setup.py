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
        ],
    },
)

