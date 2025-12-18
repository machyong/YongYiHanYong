from setuptools import find_packages, setup

package_name = 'llm_for_pick_place_voice'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # .env 파일, wakeupword 모델 파일을 package data에 포함
        ('share/' + package_name + '/resource', ['resource/alexa.onnx', 'resource/hey_yong_yihan.onnx']),
        # launch 파일 포함
        ('share/' + package_name + '/launch', ['launch/' + f for f in __import__('os').listdir('launch') if f.endswith('.py')]),
    ],
    install_requires=['setuptools', 'interface_pkg'],
    zip_safe=True,
    maintainer='hun',
    maintainer_email='jihun3333@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'get_keyword = llm_for_pick_place_voice.get_keyword:main',
            'get_keyword_client = llm_for_pick_place_voice.get_keyword_client:main',
            'stt = llm_for_pick_place_voice.stt:main',
            'wakeup_word = llm_for_pick_place_voice.wakeup_word:main',
            'ros_web_bridge = llm_for_pick_place_voice.ros_web_bridge:main',
            'arrive_test = llm_for_pick_place_voice.arrive_test:main',
            'test_robot = llm_for_pick_place_voice.test_robot:main',
            'test_bridge = llm_for_pick_place_voice.test_bridge:main',
            'pick_rl_node = llm_for_pick_place_voice.pick_rl_node:main',
            'place_rl_node = llm_for_pick_place_voice.place_rl_node:main',
        ],
    },
)
