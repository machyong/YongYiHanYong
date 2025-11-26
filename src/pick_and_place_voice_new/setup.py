from setuptools import find_packages, setup

package_name = 'pick_and_place_voice_new'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # .env 파일을 package data에 포함
        ('share/' + package_name + '/resource', ['resource/.env']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey4090',
    maintainer_email='rokey4090@example.com',
    description='Pick and Place Voice Control Package',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'get_keyword = pick_and_place_voice_new.get_keyword:main',
            'cli.py = pick_and_place_voice_new.cli:main',
            'arrive_test = pick_and_place_voice_new.arrive_test:main',
            'ros_web_bridge = pick_and_place_voice_new.ros_web_bridge:main',
        ],
    },
)
